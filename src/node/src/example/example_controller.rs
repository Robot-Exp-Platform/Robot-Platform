use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue};

use robot::{DSeriseRobot, Robot, RobotType};
use sensor::Sensor;

pub struct ExController {
    name: String,
    state: ExControllerState,
    params: ExControllerParams,
    node: ExControllerNode,
    robot: Option<Arc<RwLock<DSeriseRobot>>>,
}

#[derive(Default)]
pub struct ExControllerState {
    is_end: bool,
    ref_q: na::DVector<f64>,
    ref_q_dot: na::DVector<f64>,
    ref_q_ddot: na::DVector<f64>,
}

#[derive(Deserialize)]
pub struct ExControllerParams {
    period: f64,
    k: na::DVector<f64>,
    b: na::DVector<f64>,
}

#[derive(Default)]
pub struct ExControllerNode {
    input_queue: DNodeMessageQueue,
    output_queue: DNodeMessageQueue,
}

impl ExControllerState {
    pub fn new(dof: usize) -> ExControllerState {
        ExControllerState {
            is_end: false,
            ref_q: na::DVector::zeros(dof),
            ref_q_dot: na::DVector::zeros(dof),
            ref_q_ddot: na::DVector::zeros(dof),
        }
    }
}

impl ExController {
    pub fn from_json(name: String, json: Value) -> ExController {
        ExController::from_params(name, from_value(json).unwrap())
    }
    pub fn from_params(name: String, params: ExControllerParams) -> ExController {
        ExController {
            name,
            state: ExControllerState::default(),
            params,
            node: ExControllerNode::default(),
            robot: None,
        }
    }
}

impl Node<na::DVector<f64>> for ExController {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.state = ExControllerState::new(robot.read().unwrap().dof());
            self.robot = Some(robot);
        }
    }
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for ExController {
    fn update(&mut self) {
        // Demo begin
        {
            // 获取 robot 状态
            let robot_read = self.robot.as_ref().unwrap().read().unwrap();
            let q = robot_read.q();
            let q_dot = robot_read.q_dot();

            match self.node.input_queue.pop() {
                Some(DNodeMessage::Joint(ref_q)) => {
                    self.state.ref_q = ref_q;
                }
                Some(DNodeMessage::JointVel(ref_q, ref_q_dot)) => {
                    self.state.ref_q = ref_q;
                    self.state.ref_q_dot = ref_q_dot;
                }
                Some(DNodeMessage::JointVelAcc(ref_q, ref_q_dot, ref_q_ddot)) => {
                    self.state.ref_q = ref_q;
                    self.state.ref_q_dot = ref_q_dot;
                    self.state.ref_q_ddot = ref_q_ddot;
                }
                _ => return,
            };

            let k = na::DMatrix::from_diagonal(&self.params.k);
            let b = na::DMatrix::from_diagonal(&self.params.b);

            // 执行 impedance 逻辑
            let output = &k * (&self.state.ref_q - q) + &b * (-&q_dot);

            let control_message = DNodeMessage::Tau(output);

            // 发送控制指令
            if self.state.is_end {
                self.robot
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .set_control_message(control_message);
            } else {
                self.node.output_queue.push(control_message);
            }
        }
        // Demo end
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
