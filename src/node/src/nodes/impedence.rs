use nalgebra as na;
use serde::Deserialize;
use std::time::Duration;
use tracing::info;

use crate::{Node, NodeBehavior};
use message::DNodeMessage;

use robot::{DSeriseRobot, Robot, RobotLock};
pub type Impedence<R, V, M> = Node<ImpedenceState<V>, ImpedenceParams<M>, RobotLock<R>, V>;

pub type DImpedence = Impedence<DSeriseRobot, na::DVector<f64>, na::DMatrix<f64>>;
pub type SImpedence<R, const N: usize> = Impedence<R, na::SVector<f64, N>, na::SMatrix<f64, N, N>>;
pub type DImpedenceDiag = Impedence<DSeriseRobot, na::DVector<f64>, na::DVector<f64>>;

#[derive(Default)]
pub struct ImpedenceState<V> {
    ref_q: V,
    ref_q_dot: V,
    ref_q_ddot: V,
}

#[derive(Deserialize)]
pub struct ImpedenceParams<M> {
    period: f64,
    k: M,
    b: M,
    m: M,
}

impl NodeBehavior for DImpedence {
    fn init(&mut self) {
        let dof = self.robot.as_ref().unwrap().read().unwrap().dof();
        self.state.ref_q = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_dot = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_ddot = na::DVector::from_element(dof, 0.0);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        let q_dot = robot_read.q_dot();
        let q_ddot = robot_read.q_ddot();

        // TODO 检查任务是否完成

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

        // 执行 impedance 逻辑
        let output = &self.params.k * (&self.state.ref_q - q)
            + &self.params.b * (&self.state.ref_q_dot - &q_dot)
            + &self.params.m * (&self.state.ref_q_ddot - &q_ddot);

        let control_message = DNodeMessage::Tau(output);

        // 发送控制指令
        if self.is_end {
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

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}

impl NodeBehavior for DImpedenceDiag {
    fn init(&mut self) {
        let dof = self.robot.as_ref().unwrap().read().unwrap().dof();
        self.state.ref_q = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_dot = na::DVector::from_element(dof, 0.0);
        self.state.ref_q_ddot = na::DVector::from_element(dof, 0.0);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        let q_dot = robot_read.q_dot();
        drop(robot_read);

        // TODO 检查任务是否完成

        match self.input_queue.pop() {
            Some(DNodeMessage::Joint(ref_q)) => {
                info!(node = self.name.as_str(), input = ?ref_q.as_slice());
                self.state.ref_q = ref_q;
            }
            Some(DNodeMessage::JointVel(ref_q, ref_q_dot)) => {
                info!(node = self.name.as_str(), input = ?ref_q.as_slice());
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
            }
            Some(DNodeMessage::JointVelAcc(ref_q, ref_q_dot, ref_q_ddot)) => {
                info!(node = self.name.as_str(), input = ?ref_q.as_slice());
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
                self.state.ref_q_ddot = ref_q_ddot;
            }
            _ => return,
        };

        let k = na::DMatrix::from_diagonal(&self.params.k);
        let b = na::DMatrix::from_diagonal(&self.params.b);
        let m = na::DMatrix::from_diagonal(&self.params.m);

        // 执行 impedance 逻辑
        let output = k * (&self.state.ref_q - q)
            + b * (&self.state.ref_q_dot - &q_dot)
            + m * (&self.state.ref_q_ddot - &q_dot);

        let control_message = DNodeMessage::Tau(output);

        // 发送控制指令
        if self.is_end {
            todo!();
        } else {
            self.output_queue.push(control_message);
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
