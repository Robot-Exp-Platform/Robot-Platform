use kernel_macro::node_registration;
use nalgebra as na;
use serde::Deserialize;
use std::{f64, time::Duration};

use crate::{Node, NodeBehavior, NodeExtBehavior, NodeRegister};
use message::DNodeMessage;
use robot::{DSeriseRobot, Robot, RobotLock};

pub type Pid<R, M, V> = Node<PidState<V>, PidParams<M>, RobotLock<R>, V>;

#[node_registration("pid")]
pub type DPid = Pid<DSeriseRobot, na::DMatrix<f64>, na::DVector<f64>>;
pub type SPid<R, const N: usize> = Pid<R, na::SMatrix<f64, N, N>, na::SVector<f64, N>>;
pub type DPidDiag = Pid<DSeriseRobot, na::DVector<f64>, na::DVector<f64>>;

#[derive(Default)]
pub struct PidState<V> {
    track: V,
    error: V,
    integral: V,
    derivative: V,
}

#[derive(Deserialize)]
pub struct PidParams<M> {
    period: f64,
    kp: M,
    ki: M,
    kd: M,
}

impl NodeBehavior for DPid {
    fn init(&mut self) {
        let dof = self.robot.as_ref().unwrap().read().unwrap().dof();
        self.state.track = na::DVector::from_element(dof, 0.0);
        self.state.error = na::DVector::from_element(dof, 0.0);
        self.state.integral = na::DVector::from_element(dof, 0.0);
        self.state.derivative = na::DVector::from_element(dof, 0.0);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        drop(robot_read);
        // 获取输入
        if let Some(DNodeMessage::Joint(track)) = self.input_queue.pop() {
            self.state.track = track;
        }

        // 执行 pid 逻辑
        let new_error = &self.state.track - &q;
        self.state.integral += &new_error * self.params.period;
        self.state.derivative = (&new_error - &self.state.error) / self.params.period;
        self.state.error = new_error;

        let output = &self.params.kp * &self.state.error
            + &self.params.ki * &self.state.integral
            + &self.params.kd * &self.state.derivative;

        let control_message = DNodeMessage::Joint(output);

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

impl NodeBehavior for DPidDiag {
    fn init(&mut self) {
        let dof = self.robot.as_ref().unwrap().read().unwrap().dof();
        self.state.track = na::DVector::from_element(dof, 0.0);
        self.state.error = na::DVector::from_element(dof, 0.0);
        self.state.integral = na::DVector::from_element(dof, 0.0);
        self.state.derivative = na::DVector::from_element(dof, 0.0);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        drop(robot_read);

        if let Some(DNodeMessage::Joint(track)) = self.input_queue.pop() {
            self.state.track = track;
        }

        let kp = na::DMatrix::from_diagonal(&self.params.kp);
        let ki = na::DMatrix::from_diagonal(&self.params.ki);
        let kd = na::DMatrix::from_diagonal(&self.params.kd);

        // 执行 pid 逻辑
        let new_error = &self.state.track - &q;
        self.state.integral += &new_error * self.params.period;
        self.state.derivative = (&new_error - &self.state.error) / self.params.period;
        self.state.error = new_error;

        let output =
            &kp * &self.state.error + &ki * &self.state.integral + &kd * &self.state.derivative;

        let control_message = DNodeMessage::Joint(output);

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
