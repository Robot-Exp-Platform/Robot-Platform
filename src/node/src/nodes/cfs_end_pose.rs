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
    iso_to_vec, Constraint, DNodeMessage, DNodeMessageQueue, NodeMessage, NodeMessageQueue,
    QuadraticProgramming,
};
use robot::{DRobot, DSeriseRobot, Robot, RobotType};
use sensor::Sensor;
use solver::{OsqpSolver, Solver};

pub struct CfsEndPose<R, V> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    state: CfsEndPoseState<V>,
    /// The parameters of the planner.
    params: CfsEndPoseParams,
    /// The node of the planner.
    node: CfsEndPoseNode<V>,
    /// The robot that the planner is controlling.
    robot: Option<Arc<RwLock<R>>>,
    /// The sensor that the planner is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DCfsEndPose = CfsEndPose<DSeriseRobot, na::DVector<f64>>;
pub type SCfsEndPose<R, const N: usize> = CfsEndPose<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct CfsEndPoseState<V> {
    target: Option<NodeMessage<V>>,
}

#[derive(Deserialize, Default)]
pub struct CfsEndPoseParams {
    period: f64,
    ninterp: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct CfsEndPoseNode<V> {
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl DCfsEndPose {
    pub fn new(name: String) -> DCfsEndPose {
        DCfsEndPose::from_params(name, CfsEndPoseParams::default())
    }

    pub fn from_json(name: String, params: Value) -> DCfsEndPose {
        DCfsEndPose::from_params(name, from_value(params).unwrap())
    }

    pub fn from_params(name: String, params: CfsEndPoseParams) -> DCfsEndPose {
        let state = CfsEndPoseState::default();
        let node = CfsEndPoseNode::default();
        DCfsEndPose {
            name,
            state,
            params,
            node,
            robot: None,
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DCfsEndPose {
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

impl NodeBehavior for DCfsEndPose {
    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let ndof = robot_read.dof();
        let q = robot_read.q();
        let q_min_bound = robot_read.q_min_bound().as_slice().to_vec();
        let q_max_bound = robot_read.q_max_bound().as_slice().to_vec();

        let currect_state = DNodeMessage::Pose(robot_read.end_pose());

        if let Some(target) = self.state.target.clone() {
            info!(node = self.name.as_str(), input = ?target.as_slice());
            println!("cfs_end_pose: target: {:?}", target);
            println!("cfs_end_pose: current: {:?}", currect_state);
            println!(
                "cfs_end_pose: diff: {:?}",
                (target.clone() / currect_state.clone()).abs()
            );
            if (target / currect_state).abs() < 1e-1 {
                self.state.target = Some(self.node.input_queue.pop().unwrap());
            }
        } else {
            self.state.target = Some(self.node.input_queue.pop().unwrap());
        }

        // 获取 target
        let target = match self.state.target.clone().unwrap() {
            DNodeMessage::Pose(target) => target,
            _ => panic!("The target is not a EndPose message."),
        };

        // 执行CFS逻辑
        let mut q_end_ref = q.clone();
        let mut solver_result = Vec::new();
        let mut last_result = Vec::new();

        let dim = (self.params.ninterp + 2) * ndof;

        let h = get_optimize_function(dim, ndof, self.params.cost_weight.clone());
        let f = na::DVector::<f64>::zeros(dim);

        for k in 0..self.params.niter {
            println!("第 i 次迭代 {}", k);

            // 提前分配空间
            let mut constraints =
                Constraint::CartesianProduct(0, 0, Vec::with_capacity(self.params.ninterp + 1));

            // 生成约束 包括三部分，起点约束、末端、关节边界约束
            constraints.push(Constraint::Equared(q.as_slice().to_vec()));

            for _ in 0..self.params.ninterp {
                constraints.push(Constraint::Rectangle(
                    q_min_bound.clone(),
                    q_max_bound.clone(),
                ))
            }

            // let end_pose = iso_to_vec(robot_read.cul_end_pose(&q_end_ref));
            // let target_pose = iso_to_vec(target);
            let grad = robot_read.cul_end_pose_grad(&q_end_ref);

            // println!("cfs_end_pose: end_pose: {:?}", end_pose);
            // println!("cfs_end_pose: target_pose: {:?}", target_pose);
            // println!("cfs_end_pose: grad: {:?}", grad);
            println!("cfs_end_pose: q_end_ref: {:?}", q_end_ref);

            let mut end_pose_constraint = Constraint::Intersection(0, ndof, Vec::with_capacity(6));
            for i in 0..3 {
                let grad = grad.row(i).clone_owned();
                end_pose_constraint.push(Constraint::Hyperplane(
                    grad.as_slice().to_vec(),
                    iso_to_vec(target / robot_read.cul_end_pose(&q_end_ref))[i]
                        + (grad * &q_end_ref)[(0, 0)],
                ));
            }
            end_pose_constraint.push(Constraint::Rectangle(
                q_min_bound.clone(),
                q_max_bound.clone(),
            ));
            constraints.push(end_pose_constraint);

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

            q_end_ref.copy_from_slice(&solver_result[solver_result.len() - ndof..]);

            if last_result.is_empty() {
                last_result = solver_result.clone();
            } else {
                // 一个比较严苛的收敛条件，学长使用的是平均值，而我采用的是轨迹的误差的和，可能会导致部分情况下收敛不通过
                let diff: f64 = solver_result
                    .iter()
                    .zip(last_result.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();
                // println!("cfs_end_pose: last_result: {:?}", last_result);
                // println!("cfs_end_pose: solver_result: {:?}", solver_result);
                println!("cfs_end_pose: diff: {:?}", diff);
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
    }
    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
