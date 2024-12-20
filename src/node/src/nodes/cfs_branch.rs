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
    Constraint, DNodeMessage, DNodeMessageQueue, NodeMessage, NodeMessageQueue,
    QuadraticProgramming,
};
use robot::{DSeriseRobot, Robot, RobotBranch, RobotType};
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
    output_queue: NodeMessageQueue<V>,
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
        DCfsBranch {
            name,
            state,
            params,
            node,
            robot,
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DCfsBranch {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

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

        // 准备
        let q_ref_list = match &target {
            NodeMessage::Joint(q_target) => lerp(&q, &vec![q_target.clone()], self.params.ninterp),
            NodeMessage::Pose(_) => lerp(&q, &vec![q.clone()], self.params.ninterp),
            _ => panic!("Cfs: Unsupported message type"),
        };
        let mut last_result = Vec::new();
        let dim = (self.params.ninterp + 2) * ndof;

        for _ in 0..self.params.niter {
            // =======  建立约束  =======

            // 提前分配空间
            let mut constraints =
                Constraint::CartesianProduct(0, 0, Vec::with_capacity(self.params.ninterp + 1));

            // 起点位置的约束，这是绝对约束
            constraints.push(Constraint::Equared(q.as_slice().to_vec()));

            // 中间过程的约束
            // TODO 根据约束方程和避障不等式建立约束
            for q_ref in q_ref_list.iter() {
                constraints.push(Constraint::Rectangle(
                    q_min_bound.clone(),
                    q_max_bound.clone(),
                ));
                constraints.push(Constraint::Equared(q_ref.as_slice().to_vec()));
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

            // =======  轨迹发送  =======
            // 生成 track
            let mut track_list = Vec::new();
            for i in 1..self.params.ninterp + 2 {
                track_list.push(DNodeMessage::Joint(na::DVector::from_column_slice(
                    &last_result[i * ndof..(i + 1) * ndof],
                )));
            }
            // 发送 track
            while self.node.output_queue.pop().is_some() {}
            for track in track_list {
                self.node.output_queue.push(track);
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
