use crossbeam::queue::SegQueue;
use message::track::TrackN;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::controller_trait::{Controller, ControllerN};
use message::ControlCommandN;
#[cfg(feature = "recode")]
use recoder::*;
use robot::SeriesRobot;
use robot_macros_derive::*;
use sensor::Sensor;
use task_manager::ROSThread;

pub struct Pid<R: SeriesRobot<N>, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    node: PidNode<N>,
    robot: Arc<RwLock<R>>,
}

pub struct PidState<const N: usize> {
    track: na::SVector<f64, N>,
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

#[derive(Default)]
pub struct PidNode<const N: usize> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<TrackN<N>>>,
    control_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
}

impl<R: SeriesRobot<N>, const N: usize> Pid<R, N> {
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
                track: na::SVector::from_element(0.0),
                error: na::SVector::from_element(0.0),
                integral: na::SVector::from_element(0.0),
                derivative: na::SVector::from_element(0.0),
            },
            params,

            node: PidNode {
                sensor: None,
                recoder: None,
                track_queue: Arc::new(SegQueue::new()),
                control_command_queue: Arc::new(SegQueue::new()),
            },
            robot,
        }
    }
}

impl<R: SeriesRobot<N>, const N: usize> ControllerN<N> for Pid<R, N> {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<message::track::TrackN<N>>>) {
        self.node.track_queue = track_queue;
    }
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
    ) {
        self.node.control_command_queue = controller_command_queue;
    }
}

impl<R: SeriesRobot<N>, const N: usize> Controller for Pid<R, N> {
    generate_controller_method!();
}

impl<R: SeriesRobot<N>, const N: usize> ROSThread for Pid<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn start(&mut self) {
        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name()
            ))
            .unwrap();
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "data/{}/{}/{}/pid.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }
    }

    fn update(&mut self) {
        // 更新 track
        self.state.track = match self.node.track_queue.pop() {
            Some(TrackN::Joint(track)) => track,
            _ => return,
        };

        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.get_q();

        // 执行 pid 逻辑
        let new_error = self.state.track - q;
        self.state.integral += new_error * self.params.period;
        self.state.derivative = (new_error - self.state.error) / self.params.period;
        self.state.error = new_error;

        let control_output = self.params.kp * self.state.error
            + self.params.ki * self.state.integral
            + self.params.kd * self.state.derivative;

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, control_output);
        }

        // 发送控制指令
        let control_command = ControlCommandN::JointWithPeriod(self.params.period, control_output);
        self.node.control_command_queue.push(control_command);
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
