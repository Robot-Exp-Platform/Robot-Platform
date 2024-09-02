use core::str;
use crossbeam::queue::SegQueue;
use message::constraint::Constraint;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use solver::solver_trait::Solver;
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::planner_trait::{CfsTrait, OptimizationBasedPlanner, Planner, PlannerN};
use crate::utilities;
use message::problem::QuadraticProgramming;
use message::{Target, TrackN};
#[cfg(feature = "recode")]
use recoder::*;
use robot::SeriesRobot;
use robot_macros_derive::*;
use sensor::Sensor;
use solver::OsqpSolver;
use task_manager::{ROSThread, StateCollector};

pub struct Cfs<R: SeriesRobot<N>, const N: usize> {
    name: String,
    path: String,

    state: CfsState,
    params: CfsParams,

    node: CfsNode<N>,
    robot: Arc<RwLock<R>>,
}

#[derive(Default)]
pub struct CfsState {
    solve_suppressor: bool,
}

#[derive(Deserialize, Default)]
pub struct CfsParams {
    period: f64,
    interpolation: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct CfsNode<const N: usize> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<TrackN<N>>>,
    state_collector: StateCollector,
}

impl<R: SeriesRobot<N>, const N: usize> Cfs<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Self {
        Cfs::from_params(name, path, CfsParams::default(), robot)
    }
    pub fn from_params(
        name: String,
        path: String,
        params: CfsParams,
        robot: Arc<RwLock<R>>,
    ) -> Self {
        Cfs {
            name,
            path,
            state: CfsState::default(),
            params,
            node: CfsNode::default(),
            robot,
        }
    }
}

impl<R: SeriesRobot<N>, const N: usize> CfsTrait for Cfs<R, N> {}

impl<R: SeriesRobot<N>, const N: usize> OptimizationBasedPlanner for Cfs<R, N> {
    fn supperssion(&mut self) {
        self.state.solve_suppressor = true;
    }
}

impl<R: SeriesRobot<N>, const N: usize> PlannerN<N> for Cfs<R, N> {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<TrackN<N>>>) {
        self.node.track_queue = track_queue;
    }
}

impl<R: SeriesRobot<N>, const N: usize> Planner for Cfs<R, N> {
    generate_planner_method!();

    fn as_cfs_planner(&self) -> Option<&dyn CfsTrait> {
        Some(self)
    }
}

impl<R: SeriesRobot<N>, const N: usize> ROSThread for Cfs<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn start(&mut self) {
        // 进入启动状态，并通知所有线程
        let (state, cvar) = &*self.node.state_collector;
        state.lock().unwrap().start();
        cvar.notify_all();

        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name()
            ))
            .unwrap();
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "./data/{}/{}/{}/cfs.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }

        // 进入循环状态，并通知所有线程
        state.lock().unwrap().update();
        cvar.notify_all();
    }

    fn update(&mut self) {
        // 更新 target
        let target = match self.node.target_queue.pop() {
            // 根据不同的 target 类型，执行不同的任务，也可以将不同的 Target 类型处理为相同的类型
            Some(Target::Joint(joint)) => joint,
            None => {
                // 任务已经全部完成，进入结束状态，并通知所有线程
                let (state, cvar) = &*self.node.state_collector;
                state.lock().unwrap().finish();
                cvar.notify_all();

                return;
            }
            _ => unimplemented!("CFS planner does not support target of type."),
        };
        let target = na::SVector::from_vec(target);
        println!("{} get target: {:?}", self.name, target);

        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.get_q();
        let q_min_bound = robot_read.get_q_min_bound().as_slice().to_vec();
        let q_max_bound = robot_read.get_q_max_bound().as_slice().to_vec();

        // 执行CFS逻辑
        let q_ref_list = utilities::interpolation::<N>(&q, &target, self.params.interpolation);
        let collision_objects = self
            .node
            .sensor
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .get_collision();
        let mut solver_result = Vec::new();
        let mut last_result = Vec::new();

        // 初始化二次规划的目标函数矩阵
        // 矩阵待修改，实际上为q1 为对角矩阵，q2 为离散拉普拉斯算子， q3 为

        let dim = (self.params.interpolation + 1) * N;

        let q1 = na::DMatrix::<f64>::identity(dim, dim);

        let mut v_diff = na::DMatrix::<f64>::identity(dim - N, dim);
        for i in 0..dim - N {
            v_diff[(i, i + N)] = -1.0;
        }
        let q2 = v_diff.transpose() * v_diff;

        let mut a_diff = na::DMatrix::<f64>::identity(dim - 2 * N, dim);
        for i in 0..dim - 2 * N {
            a_diff[(i, i + N)] = -2.0;
            a_diff[(i, i + 2 * N)] = 1.0;
        }
        let q3 = a_diff.transpose() * a_diff;

        let h = q1 * self.params.cost_weight[0]
            + q2 * self.params.cost_weight[1]
            + q3 * self.params.cost_weight[2];

        let f = na::DVector::<f64>::zeros(dim);

        // 线性插值时，f为0

        for _ in 0..self.params.niter {
            // 提前分配空间
            let mut combined_constraint = Constraint::CartesianProduct(
                0,
                0,
                Vec::with_capacity(self.params.interpolation + 1),
            );

            // 生成约束 包括三部分，起点终点约束、障碍物约束、关节边界约束
            combined_constraint.push(Constraint::Equared(q.as_slice().to_vec()));

            for q_ref in q_ref_list.iter() {
                let mut obstacle_constraint =
                    Constraint::Intersection(0, 0, Vec::with_capacity(collision_objects.len()));

                for (distance, gradient) in collision_objects.iter().map(|collision| {
                    (
                        robot_read.get_distance_with_joint(q_ref, collision),
                        robot_read.get_distance_diff_with_joint(q_ref, collision),
                    )
                }) {
                    obstacle_constraint.push(Constraint::Intersection(
                        1 + N,
                        N,
                        vec![
                            Constraint::Halfspace(
                                (-gradient).as_slice().to_vec(),
                                distance - (gradient.transpose() * q_ref)[(0, 0)],
                            ),
                            Constraint::Rectangle(q_min_bound.clone(), q_max_bound.clone()),
                        ],
                    ));
                }

                combined_constraint.push(obstacle_constraint);
            }

            combined_constraint.push(Constraint::Equared(target.as_slice().to_vec()));

            let problem = QuadraticProgramming {
                h: &h,
                f: &f,
                constraints: combined_constraint,
            };

            // 获得优化方程及其梯度
            // 求解优化问题
            solver_result = match self.params.solver.as_str() {
                "osqp" if !self.state.solve_suppressor => {
                    let mut osqp_solver = OsqpSolver::from_problem(problem);
                    osqp_solver.solve()
                }
                _ => unimplemented!(),
            };

            // 检查是否收敛，更新 q_ref_list

            if last_result.is_empty() {
                last_result = solver_result.clone();
            } else {
                // 一个比较严苛的收敛条件，学长使用的是平均值，而我采用的是轨迹的误差的和，可能会导致部分情况下收敛不通过
                let diff: f64 = solver_result
                    .iter()
                    .zip(last_result.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();
                println!("diff: {}", diff);
                if diff.abs() < 1e-1 {
                    break;
                }
                last_result = solver_result.clone();
            }
        }

        // 生成 track
        let mut track_list = Vec::new();
        for i in 0..self.params.interpolation + 1 {
            track_list.push(TrackN::Joint(na::SVector::from_column_slice(
                &solver_result[i * N..(i + 1) * N],
            )));
        }

        // 记录 track
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            for track in track_list.iter() {
                recode!(recoder, track);
            }
        }

        // 发送 track
        for track in track_list {
            self.node.track_queue.push(track);
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
