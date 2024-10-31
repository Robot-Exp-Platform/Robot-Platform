use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use tracing::info;
// use serde_yaml::{from_value, Value};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{utilities::*, Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{
    Constraint, DNodeMessage, DNodeMessageQueue, NodeMessage, NodeMessageQueue,
    QuadraticProgramming,
};

use robot::{DRobot, DSeriseRobot, Robot, RobotType};
use sensor::Sensor;
use solver::{OsqpSolver, Solver};

pub struct Cfs<R, V> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    state: CfsState<V>,
    /// The parameters of the planner.
    params: CfsParams,
    /// The node of the planner.
    node: CfsNode<V>,
    /// The robot that the planner is controlling.
    robot: Option<Arc<RwLock<R>>>,
    /// The sensor that the planner is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DCfs = Cfs<DSeriseRobot, na::DVector<f64>>;
pub type SCfs<R, const N: usize> = Cfs<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct CfsState<V> {
    target: Option<NodeMessage<V>>,
}

#[derive(Deserialize, Default)]
pub struct CfsParams {
    period: f64,
    ninterp: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct CfsNode<V> {
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl DCfs {
    pub fn new(name: String) -> DCfs {
        DCfs::from_params(name, CfsParams::default())
    }

    pub fn from_json(name: String, json: Value) -> DCfs {
        DCfs::from_params(name, from_value(json).unwrap())
    }

    pub fn from_params(name: String, params: CfsParams) -> DCfs {
        DCfs {
            name,
            state: CfsState::default(),
            params,
            node: CfsNode::default(),
            robot: None,
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DCfs {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = Some(robot);
        }
    }
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DCfs {
    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let ndof = robot_read.dof();
        let q = robot_read.q();
        let q_min_bound = robot_read.q_min_bound().as_slice().to_vec();
        let q_max_bound = robot_read.q_max_bound().as_slice().to_vec();

        let currect_state = DNodeMessage::Joint(q.clone());

        if let Some(target) = self.state.target.clone() {
            info!(node = self.name.as_str(), input = ?target.as_slice());

            if (target / currect_state).abs() < 1e-2 {
                self.state.target = Some(self.node.input_queue.pop().unwrap());
            }
        } else {
            self.state.target = Some(self.node.input_queue.pop().unwrap());
        }

        // 获取 target，并辨析其内涵
        let target = match self.state.target.clone().unwrap() {
            // 根据不同的 target 类型，执行不同的任务，也可以将不同的 Target 类型处理为相同的类型
            DNodeMessage::Joint(joint) => joint,
            _ => unimplemented!("CFS planner does not support target of type."),
        };
        // println!("{} get target: {:?}", self.name, target);

        // 执行CFS逻辑
        let q_ref_list = lerp(&q, &vec![target.clone()], self.params.ninterp);
        let collision_objects = self.sensor.as_ref().unwrap().read().unwrap().collision();
        let mut solver_result = Vec::new();
        // TODO 检验参考轨迹是否安全可靠,取不可靠段做重规划
        // TODO 理论上最新轨迹应该为被重新规划的段落
        let mut last_result = Vec::new();

        // 初始化二次规划的目标函数矩阵
        // 矩阵待修改，实际上为q1 为对角矩阵，q2 为离散拉普拉斯算子， q3 为

        let dim = (self.params.ninterp + 2) * ndof;

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
                    Constraint::Intersection(0, ndof, Vec::with_capacity(collision_objects.len()));

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
                if diff.abs() < 1e-1 {
                    break;
                }
                last_result = solver_result.clone();
            }
        }

        // 单步轨迹发送
        // 生成 track
        let mut track_list = Vec::new();
        for i in 1..self.params.ninterp + 2 {
            track_list.push(DNodeMessage::Joint(na::DVector::from_column_slice(
                &solver_result[i * ndof..(i + 1) * ndof],
            )));
        }
        // 发送 track
        while let Some(_) = self.node.output_queue.pop() {}
        for track in track_list {
            self.node.output_queue.push(track);
        }

        // // 整体轨迹发送
        // // 生成 track
        // let mut track_list = Vec::new();
        // for i in 1..self.params.ninterp + 2 {
        //     track_list.push(na::DVector::from_column_slice(
        //         &solver_result[i * ndof..(i + 1) * ndof],
        //     ));
        // }
        // // 发送 track
        // while let Some(_) = self.node.output_queue.pop() {}
        // self.node
        //     .output_queue
        //     .push(DNodeMessage::JointList(track_list));
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
