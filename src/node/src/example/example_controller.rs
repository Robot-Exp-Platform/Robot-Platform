use nalgebra as na;
use serde::Deserialize;
use std::time::Duration;

use crate::{Node, NodeBehavior};
use message::DNodeMessage;
use robot::{DSeriseRobot, Robot, RobotLock};

pub type ExController =
    Node<ExControllerState, ExControllerParams, RobotLock<DSeriseRobot>, na::DVector<f64>>;

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

impl NodeBehavior for ExController {
    fn init(&mut self) {
        let dof = self.robot.as_ref().unwrap().read().unwrap().dof();
        self.state.ref_q = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_dot = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_ddot = na::DVector::from_element(dof, 0.0);
    }
    fn update(&mut self) {
        // Demo begin
        {
            // 获取 robot 状态
            let robot_read = self.robot.as_ref().unwrap().read().unwrap();
            let q = robot_read.q();
            let q_dot = robot_read.q_dot();

            match self.input_queue.pop() {
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
                self.output_queue.push(control_message);
            }
        }
        // Demo end
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
