use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
// use serde_json::{from_value, Value};
use serde_yaml::{from_value, Value};
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::controller_trait::Controller;
use message::{control_command::ControlCommand, track::Track};
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

pub struct Pid<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    msgnode: PidNode,
    robot: Arc<RwLock<R>>,
}

#[derive(Clone, Copy)]
pub struct PidState<const N: usize> {
    target: na::SVector<f64, N>,
    error: na::SVector<f64, N>,
    integral: na::SVector<f64, N>,
    derivative: na::SVector<f64, N>,
}

#[derive(Deserialize)]
pub struct PidParams<const N: usize> {
    period: f64,
    kp: na::SMatrix<f64, N, N>,
    ki: na::SMatrix<f64, N, N>,
    kd: na::SMatrix<f64, N, N>,
}

pub struct PidNode {
    track_queue: Arc<SegQueue<Track>>,
    control_command_queue: Arc<SegQueue<ControlCommand>>,
}

impl<R: Robot + 'static, const N: usize> Pid<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Pid<R, N> {
        Pid::from_params(
            name,
            path,
            PidParams {
                period: 0.0,
                kp: na::SMatrix::from_element(0.0),
                ki: na::SMatrix::from_element(0.0),
                kd: na::SMatrix::from_element(0.0),
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        path: String,
        params: PidParams<N>,
        robot: Arc<RwLock<R>>,
    ) -> Pid<R, N> {
        Pid {
            name,
            path,

            state: PidState {
                target: na::SVector::from_element(0.0),
                error: na::SVector::from_element(0.0),
                integral: na::SVector::from_element(0.0),
                derivative: na::SVector::from_element(0.0),
            },
            params,

            msgnode: PidNode {
                track_queue: Arc::new(SegQueue::new()),
                control_command_queue: Arc::new(SegQueue::new()),
            },
            robot,
        }
    }
}

impl<R: Robot + 'static, const N: usize> Controller for Pid<R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn set_params(&mut self, params: Value) {
        let params: PidParams<N> = from_value(params).unwrap();
        self.params = params;
    }
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>) {
        self.msgnode.track_queue = track_queue;
    }
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    ) {
        self.msgnode.control_command_queue = controller_command_queue;
    }

    fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
}

impl<R: Robot + 'static, const N: usize> ROSThread for Pid<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }
    fn start(&mut self) {}

    fn update(&mut self) {
        let robot_read = self.robot.read().unwrap();
        let q = na::SVector::from_row_slice(&robot_read.get_q()[..N]);

        let new_error = self.state.target - q;
        self.state.integral += new_error * self.params.period;
        self.state.derivative = (new_error - self.state.error) / self.params.period;
        self.state.error = new_error;

        let _control_output = self.params.kp * self.state.error
            + self.params.ki * self.state.integral
            + self.params.kd * self.state.derivative;
    }

    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
