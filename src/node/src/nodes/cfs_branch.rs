use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use tracing::info;
// use serde_yaml::{from_value, Value};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{utilities::*, Node, NodeBehavior, NodeState};
use generate_tools::{get_fn, set_fn};
use message::{
    iso_to_vec, Constraint, DNodeMessage, DNodeMessageQueue, NodeMessage, NodeMessageQueue, Pose,
    QuadraticProgramming,
};
use robot::{DRobot, DSeriseRobot, Robot, RobotBranch, RobotType};
use sensor::Sensor;
use solver::{OsqpSolver, Solver};

pub struct CfsBranch<R, V> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    state: CfsBranchState<V>,
    /// The parameters of the planner.
    params: CfsBranchParams,
    /// The node of the planner.
    node: CfsBranchNode<V>,
    /// The robot that the planner is controlling.
    robot: RobotBranch<R>,
    // fake_robot: RobotBranch<FaekPoseRobot>,
    /// The sensor that the planner is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DCfsBranch = CfsBranch<DSeriseRobot, na::DVector<f64>>;

#[derive(Default)]
pub struct CfsBranchState<V> {
    target: Option<NodeMessage<V>>,
    node_state: NodeState,
}

#[derive(Deserialize, Default)]
pub struct CfsBranchParams {
    period: f64,
    ninterp: usize,
    niter: usize,
    cost_weight: Vec<f64>,
    solver: String,
}

#[derive(Default)]
pub struct CfsBranchNode<V> {
    input_queue: NodeMessageQueue<V>,
    output_queue: Vec<NodeMessageQueue<V>>,
}

impl DCfsBranch {
    pub fn new(name: String) -> DCfsBranch {
        DCfsBranch::from_params(name, CfsBranchParams::default())
    }

    pub fn from_json(name: String, json: Value) -> DCfsBranch {
        Self::from_params(name, from_value(json).unwrap())
    }

    pub fn from_params(name: String, params: CfsBranchParams) -> DCfsBranch {
        let node = CfsBranchNode::default();
        let state = CfsBranchState::default();
        let robot = RobotBranch::new();
        // let fake_robot = RobotBranch::new();
        DCfsBranch {
            name,
            state,
            params,
            node,
            robot,
            // fake_robot,
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DCfsBranch {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node));

    fn set_output_queue(
        &mut self,
        output_queue: Arc<crossbeam::queue::SegQueue<NodeMessage<na::DVector<f64>>>>,
    ) {
        self.node.output_queue.push(output_queue);
    }

    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot.push(robot);
        }
    }
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DCfsBranch {
    /// 多臂的CFS规划器主逻辑与单臂的CFS的是相似的。只是在取变量的过程中需要将多个机器人的状态拼在一起。
    /// TODO 是否考虑为多臂的节点设计一个机器人类型，而不是直接使用Vec<Robot>。虽然干的是同一件事情，但是可以吧组合状态和发布状态的步骤分开来。
    /// 1. 检查机器人状态并获取目标
    /// 2. 优化器初始化
    /// 3. 循环：建立约束。 起始位置的绝对约束、过程中的约束、末端点的约束。
    /// 4. 循环：优化器求解并检查是否收敛，收敛则退出循环。
    /// 5. 向下级节点发布消息
    fn update(&mut self) {
        // 获取 robot 状态
        let ndof = self.robot.dof();
        let q = self.robot.q();
        let q_min_bound = self.robot.q_min_bound().as_slice().to_vec();
        let q_max_bound = self.robot.q_max_bound().as_slice().to_vec();

        let currect_state = DNodeMessage::Joint(q.clone());

        if let Some(target) = self.state.target.clone() {
            if (target / currect_state).abs() < 1e-2 {
                self.state.target = None
            }
        }

        // 获取 target
        let target = self
            .state
            .target
            .clone()
            .or_else(|| self.node.input_queue.pop());
        if target.is_none() {
            self.state.node_state = NodeState::RelyRelease;
            return;
        }
        let target = target.unwrap();
        info!(node = self.name.as_str(), input = ?target.as_slice());

        // 初始化轨迹，一般来说用插值或者逆解，优秀的初始化轨迹应当是验证安全性之后就是最优的的轨迹，但是这似乎很难做到
        let q_ref_list = match &target {
            NodeMessage::Joint(q_target) => lerp(&q, &vec![q_target.clone()], self.params.ninterp),
            _ => lerp(&q, &vec![q.clone()], self.params.ninterp),
        };
        let collision_objects = self.sensor.as_ref().unwrap().read().unwrap().collision();
        let mut last_result = Vec::new();
        let dim = (self.params.ninterp + 2) * ndof;

        for _ in 0..self.params.niter {
            // =======  建立约束  =======

            // 提前分配空间
            let mut constraints =
                Constraint::CartesianProduct(0, 0, Vec::with_capacity(self.params.ninterp + 1));

            // 起点位置的约束，一般来说就是当前位置，这是绝对约束
            constraints.push(Constraint::Equared(q.as_slice().to_vec()));

            // 中间过程的约束
            for q_ref in q_ref_list.iter() {
                // 边界约束，最最基础的约束
                let mut process_constraint =
                    Constraint::Rectangle(q_min_bound.clone(), q_max_bound.clone());

                // 如果有障碍物的话，增加碰撞约束
                for collision in &collision_objects {
                    let func = |q: &na::DVector<f64>| self.robot.cul_dis_to_collision(q, collision);
                    let (dis, grad) = self.robot.cul_func(q_ref, &func);
                    // 过程中对每个障碍物的碰撞约束与关节角约束
                    process_constraint += Constraint::Halfspace(
                        (-&grad).as_slice().to_vec(),
                        (dis - (&grad * q_ref))[(0, 0)],
                    );
                    // TODO 实际上虚构机器人也应该有碰撞检测，这代表约束实体的碰撞空间
                    // TODO 机器人之间的碰撞检测
                }

                // 过程中的任务约束
                match target {
                    NodeMessage::Process(
                        box NodeMessage::Relative(id_1, id_2, relative_pose),
                        _,
                    ) => {
                        let func = |pose_1: Pose, pose_2: Pose| iso_to_vec(pose_1.inv_mul(&pose_2));
                        let (velue, grad) =
                            self.robot.cul_relative_func((id_1, id_2), q_ref, &func);
                        let b_bar = iso_to_vec(relative_pose) - velue + &grad * q_ref;
                        process_constraint += Constraint::Hyperplane(
                            grad.nrows(),
                            grad.ncols(),
                            grad.as_slice().to_vec(),
                            b_bar.as_slice().to_vec(),
                        );
                    }
                    _ => (),
                }

                constraints.push(process_constraint);
            }

            // 终点位置的约束
            // TODO 根据联合约束目标建立约束
            match target {
                NodeMessage::Joint(ref q_target) => {
                    constraints.push(Constraint::Equared(q_target.as_slice().to_vec()));
                }
                _ => panic!("Cfs: Unsupported message type"),
            }

            // =======  优化器求解  =======
            let h = get_optimize_function(dim, ndof, self.params.cost_weight.clone());
            let f = na::DVector::<f64>::zeros(dim);
            let problem = QuadraticProgramming {
                h: &h,
                f: f.as_slice(),
                constraints,
            };

            let solver_result = match self.params.solver.as_str() {
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
        // =======  轨迹发送  =======
        // 生成 track
        let mut track_list = Vec::new();
        for i in 1..self.params.ninterp + 2 {
            track_list.push(na::DVector::from_column_slice(
                &last_result[i * ndof..(i + 1) * ndof],
            ));
        }
        // 根据分割 发送 track

        // 清空输出队列
        for output_queue in self.node.output_queue.iter() {
            while output_queue.pop().is_some() {}
        }

        // 发送轨迹
        for track in track_list {
            let tracks = self.robot.split(track);
            for (track, output_queue) in tracks.iter().zip(self.node.output_queue.iter()) {
                output_queue.push(NodeMessage::Joint(track.clone()));
            }
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }

    fn state(&mut self) -> NodeState {
        self.state.node_state
    }
}
