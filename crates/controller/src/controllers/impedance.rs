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

use crate::{Controller, ControllerN};
use message::ControlCommandN;
#[cfg(feature = "recode")]
use recoder::*;
use robot::SeriesRobot;
use robot_macros_derive::*;
use sensor::Sensor;
use task_manager::ROSThread;

pub struct Impedance<R: SeriesRobot<N>, const N: usize> {
    name: String,
    path: String,

    state: ImpedanceState<N>,
    params: ImpedanceParams<N>,

    node: ImpedanceNode<N>,
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

#[derive(Default)]
pub struct ImpedanceNode<const N: usize> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<TrackN<N>>>,
    control_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
}

impl<R: SeriesRobot<N>, const N: usize> Impedance<R, N> {
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
        Impedance {
            name,
            path,

            state: ImpedanceState {
                ref_q: na::SVector::from_element(0.0),
                ref_q_dot: na::SVector::from_element(0.0),
                ref_q_ddot: na::SVector::from_element(0.0),
            },
            params,

            node: ImpedanceNode::default(),
            robot,
        }
    }
}

impl<R: SeriesRobot<N>, const N: usize> ControllerN<N> for Impedance<R, N> {
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<message::control_command::ControlCommandN<N>>>,
    ) {
        self.node.control_command_queue = controller_command_queue;
    }
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<TrackN<N>>>) {
        self.node.track_queue = track_queue
    }
}

impl<R: SeriesRobot<N>, const N: usize> Controller for Impedance<R, N> {
    generate_controller_method!();
}

impl<R: SeriesRobot<N>, const N: usize> ROSThread for Impedance<R, N> {
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
        match self.node.track_queue.pop() {
            Some(TrackN::Joint(ref_q)) => {
                self.state.ref_q = ref_q;
                println!("{} get track: {:?}", self.name, self.state.ref_q);
            }
            Some(TrackN::JointVel(ref_q, ref_q_dot)) => {
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
                println!(
                    "{} get track: {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot
                );
            }
            Some(TrackN::JointVelAcc(ref_q, ref_q_dot, ref_q_ddot)) => {
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
                self.state.ref_q_ddot = ref_q_ddot;
                println!(
                    "{} get track: {:?}, {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot, self.state.ref_q_ddot
                );
            }
            _ => return,
        };

        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.get_q();
        let q_dot = robot_read.get_q_dot();
        let q_ddot = robot_read.get_q_ddot();

        // 执行 impedance 逻辑

        let control_output = self.params.k * (self.state.ref_q - q)
            + self.params.b * (self.state.ref_q_dot - q_dot)
            + self.params.m * (self.state.ref_q_ddot - q_ddot);

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, control_output);
        }

        // 发送控制指令
        let control_command = ControlCommandN::TauWithPeriod(self.params.period, control_output);
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
