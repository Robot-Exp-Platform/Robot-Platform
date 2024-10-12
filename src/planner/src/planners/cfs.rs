use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{utilities::*, DPlanner, Planner};
use generate_tools::{get_fn, set_fn};
use manager::Node;
use message::{Constraint, DTrack, QuadraticProgramming, Target};
#[cfg(feature = "recode")]
use recoder::*;
use robot::DRobot;
use sensor::Sensor;
use solver::{OsqpSolver, Solver};

pub struct DCfs<R> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    state: DCfsState,
    /// The parameters of the planner.
    params: DCfsParams,
    /// The node of the planner.
    node: DCfsNode,
    /// The robot that the planner is controlling.
    robot: Arc<RwLock<R>>,
}

#[derive(Default)]
pub struct DCfsState {
    target: Option<Target>,
}

#[derive(Deserialize, Default)]
pub struct DCfsParams {
    period: f64,
    ninterp: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct DCfsNode {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<DTrack>>,
}

impl<R: DRobot> DCfs<R> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> DCfs<R> {
        DCfs::from_params(name, DCfsParams::default(), robot)
    }

    pub fn from_params(name: String, params: DCfsParams, robot: Arc<RwLock<R>>) -> DCfs<R> {
        DCfs {
            name,
            state: DCfsState::default(),
            params,
            node: DCfsNode::default(),
            robot,
        }
    }
}

impl<R: DRobot> DPlanner for DCfs<R> {
    set_fn!((set_track_queue, track_queue: Arc<SegQueue<DTrack>>, node));
}

impl<R: DRobot> Planner for DCfs<R> {
    get_fn!((name: String));
    set_fn!((set_target_queue, target_queue: Arc<SegQueue<Target>>, node));

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl<R: DRobot> Node for DCfs<R> {
    

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let ndof = robot_read.dof();
        let q = robot_read.q();
        let q_min_bound = robot_read.q_min_bound().as_slice().to_vec();
        let q_max_bound = robot_read.q_max_bound().as_slice().to_vec();

        // TODO 检查target是否抵达

        // 检查上次任务是否完成, 若完成则获取新的 target
        // TODO 任务完成的判断条件
        let target = self
            .state
            .target
            .clone()
            .unwrap_or(self.node.target_queue.pop().unwrap());
        let target = match target {
            // 根据不同的 target 类型，执行不同的任务，也可以将不同的 Target 类型处理为相同的类型
            Target::Joint(joint) => joint,
            _ => unimplemented!("CFS planner does not support target of type."),
        };
        println!("{} get target: {:?}", self.name, target);

        // 执行CFS逻辑
        let q_ref_list = lerp(&q, &vec![target.clone()], self.params.ninterp);
        let collision_objects = self
            .node
            .sensor
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .collision();
        let mut solver_result = Vec::new();
        // TODO 检验参考轨迹是否安全可靠,取不可靠段做重规划
        // TODO 理论上最新轨迹应该为被重新规划的段落
        let mut last_result = Vec::new();

        // 初始化二次规划的目标函数矩阵
        // 矩阵待修改，实际上为q1 为对角矩阵，q2 为离散拉普拉斯算子， q3 为

        let dim = (self.params.ninterp + 1) * ndof;

        let h = get_optimize_function(dim, ndof, self.params.cost_weight.clone());
        let f = na::DVector::<f64>::zeros(dim);

        // f 应该是什么呢？

        for _ in 0..self.params.niter {
            // 提前分配空间
            let mut constraints =
                Constraint::CartesianProduct(0, 0, Vec::with_capacity(self.params.ninterp + 1));

            // 生成约束 包括三部分，起点终点约束、障碍物约束、关节边界约束

            // 起点位置的约束，这是绝对约束
            constraints.push(Constraint::Equared(q.as_slice().to_vec()));

            for q_ref in q_ref_list.iter() {
                let mut obstacle_constraint =
                    Constraint::Intersection(0, 0, Vec::with_capacity(collision_objects.len()));

                for (dis, grad) in collision_objects.iter().map(|collision| {
                    (
                        robot_read.cul_dis_to_collision(q_ref, collision),
                        robot_read.cul_dis_grad_to_collision(q_ref, collision),
                    )
                }) {
                    // 过程中对每个障碍物的碰撞约束与关节角约束
                    obstacle_constraint.push(
                        Constraint::Halfspace(
                            (-&grad).as_slice().to_vec(),
                            dis - (&grad.transpose() * q_ref)[(0, 0)],
                        ) + Constraint::Rectangle(q_min_bound.clone(), q_max_bound.clone()),
                    );
                }
                constraints.push(obstacle_constraint);
            }

            // 终点位置的约束，这是绝对约束
            constraints.push(Constraint::Equared(target.as_slice().to_vec()));

            let problem = QuadraticProgramming {
                h: &h,
                f: f.as_slice(),
                constraints,
            };

            // 获得优化方程及其梯度
            // 求解优化问题
            solver_result = match self.params.solver.as_str() {
                "osqp" => {
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
        for i in 0..self.params.ninterp + 1 {
            track_list.push(DTrack::Joint(na::DVector::from_column_slice(
                &solver_result[i * ndof..(i + 1) * ndof],
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

    fn start(&mut self) {
        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().name()
            ))
            .unwrap();
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "./data/{}/{}/{}/cfs.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().name(),
                ))
                .unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
