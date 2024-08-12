use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::controller_trait::Controller;
use message::{control_command::ControlCommand, track::Track};
#[cfg(feature = "recode")]
use recoder::*;
use robot::robot_trait::SeriesRobot;
use robot_macros_derive::*;
use task_manager::ros_thread::ROSThread;

pub struct Impedance<R: SeriesRobot<N> + 'static, const N: usize> {
    name: String,
    path: String,

    state: ImpedanceState<N>,
    params: ImpedanceParams<N>,

    msgnode: ImpedanceNode,
    robot: Arc<RwLock<R>>,
}

pub struct ImpedanceState<const N: usize> {
    ref_q: na::SVector<f64, N>,
    ref_q_dot: na::SVector<f64, N>,
    ref_q_ddot: na::SVector<f64, N>,
}

#[derive(Deserialize)]
pub struct ImpedanceParams<const N: usize> {
    period: f64,
    k: na::SMatrix<f64, N, N>,
    b: na::SMatrix<f64, N, N>,
    m: na::SMatrix<f64, N, N>,
}

pub struct ImpedanceNode {
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<Track>>,
    control_command_queue: Arc<SegQueue<ControlCommand>>,
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Impedance<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Impedance<R, N> {
        Impedance::from_params(
            name,
            path,
            ImpedanceParams {
                period: 0.0,
                k: na::SMatrix::from_element(0.0),
                b: na::SMatrix::from_element(0.0),
                m: na::SMatrix::from_element(0.0),
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        path: String,
        params: ImpedanceParams<N>,
        robot: Arc<RwLock<R>>,
    ) -> Impedance<R, N> {
        let msgnode = ImpedanceNode {
            recoder: None,
            track_queue: Arc::new(SegQueue::new()),
            control_command_queue: Arc::new(SegQueue::new()),
        };
        Impedance {
            name,
            path,

            state: ImpedanceState {
                ref_q: na::SVector::from_element(0.0),
                ref_q_dot: na::SVector::from_element(0.0),
                ref_q_ddot: na::SVector::from_element(0.0),
            },
            params,

            msgnode,
            robot,
        }
    }
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Controller for Impedance<R, N> {
    generate_controller_method!();
}

impl<R: SeriesRobot<N> + 'static, const N: usize> ROSThread for Impedance<R, N> {
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
            self.msgnode.recoder = Some(BufWriter::new(file));
        }
    }

    fn update(&mut self) {
        // 更新 track
        match self.msgnode.track_queue.pop() {
            Some(Track::Joint(ref_q)) => {
                self.state.ref_q = na::SVector::from_vec(ref_q);
                println!("{} get track: {:?}", self.name, self.state.ref_q);
            }
            Some(Track::JointVelocity(ref_q, ref_q_dot)) => {
                self.state.ref_q = na::SVector::from_vec(ref_q);
                self.state.ref_q_dot = na::SVector::from_vec(ref_q_dot);
                println!(
                    "{} get track: {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot
                );
            }
            Some(Track::JointVelocityAcceleration(ref_q, ref_q_dot, ref_q_ddot)) => {
                self.state.ref_q = na::SVector::from_vec(ref_q);
                self.state.ref_q_dot = na::SVector::from_vec(ref_q_dot);
                self.state.ref_q_ddot = na::SVector::from_vec(ref_q_ddot);
                println!(
                    "{} get track: {:?}, {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot, self.state.ref_q_ddot
                );
            }
            _ => return,
        };

        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.get_q_na();
        let q_dot = robot_read.get_q_dot_na();
        let q_ddot = robot_read.get_q_ddot_na();

        // 执行 impedance 逻辑

        let control_output = self.params.k * (self.state.ref_q - q)
            + self.params.b * (self.state.ref_q_dot - q_dot)
            + self.params.m * (self.state.ref_q_ddot - q_ddot);

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.msgnode.recoder {
            recode!(recoder, control_output);
        }

        // 发送控制指令
        let control_command =
            ControlCommand::TauWithPeriod(self.params.period, control_output.as_slice().to_vec());
        self.msgnode.control_command_queue.push(control_command);
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.msgnode.recoder {
            recoder.flush().unwrap();
        }
    }

    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
