use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::controller_trait::{Controller, DController};
use generate_tools::{get_fn, set_fn};
use manager::Node;
use message::{DControlCommand, DTrack};
#[cfg(feature = "recode")]
use recoder::*;
use robot::DRobot;
use sensor::Sensor;

pub struct DImpedence<R> {
    /// The name of the controller.
    name: String,
    /// The path of the controller.
    state: DImpedenceState,
    /// The parameters of the controller.
    params: DImpedenceParams,
    /// The node of the controller.
    node: DImpedenceNode,
    /// The robot that the controller is controlling.
    robot: Arc<RwLock<R>>,
}

pub struct DImpedenceState {
    ref_q: na::DVector<f64>,
    ref_q_dot: na::DVector<f64>,
    ref_q_ddot: na::DVector<f64>,
}

#[derive(Deserialize)]
pub struct DImpedenceParams {
    period: f64,
    k: na::DMatrix<f64>,
    b: na::DMatrix<f64>,
    m: na::DMatrix<f64>,
}

#[derive(Default)]
pub struct DImpedenceNode {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<DTrack>>,
    control_cmd_queue: Arc<SegQueue<DControlCommand>>,
}

impl<R: DRobot> DImpedence<R> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> DImpedence<R> {
        let ndof = robot.read().unwrap().dof();
        DImpedence::from_params(
            name,
            DImpedenceParams {
                period: 0.0,
                k: na::DMatrix::zeros(ndof, ndof),
                b: na::DMatrix::zeros(ndof, ndof),
                m: na::DMatrix::zeros(ndof, ndof),
            },
            robot,
        )
    }

    pub fn from_params(
        name: String,
        params: DImpedenceParams,
        robot: Arc<RwLock<R>>,
    ) -> DImpedence<R> {
        let ndof = robot.read().unwrap().dof();
        DImpedence {
            name,
            state: DImpedenceState {
                ref_q: na::DVector::zeros(ndof),
                ref_q_dot: na::DVector::zeros(ndof),
                ref_q_ddot: na::DVector::zeros(ndof),
            },
            params,
            node: DImpedenceNode::default(),
            robot,
        }
    }
}

impl<R: DRobot> DController for DImpedence<R> {
    set_fn!((set_track_queue, track_queue: Arc<SegQueue<DTrack>>, node),
            (set_control_cmd_queue, control_cmd_queue: Arc<SegQueue<DControlCommand>>, node));
}

impl<R: DRobot> Controller for DImpedence<R> {
    get_fn!((name: String));

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl<R: DRobot> Node for DImpedence<R> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.q();
        let q_dot = robot_read.q_dot();
        let q_ddot = robot_read.q_ddot();

        // TODO 检查任务是否完成

        match self.node.track_queue.pop() {
            Some(DTrack::Joint(ref_q)) => {
                self.state.ref_q = ref_q;
                println!("{} get track: {:?}", self.name, self.state.ref_q);
            }
            Some(DTrack::JointVel(ref_q, ref_q_dot)) => {
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
                println!(
                    "{} get track: {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot
                );
            }
            Some(DTrack::JointVelAcc(ref_q, ref_q_dot, ref_q_ddot)) => {
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

        // 执行 impedance 逻辑

        let control_output = &self.params.k * (&self.state.ref_q - q)
            + &self.params.b * (&self.state.ref_q_dot - q_dot)
            + &self.params.m * (&self.state.ref_q_ddot - q_ddot);

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, control_output);
        }

        // 发送控制指令
        let control_command = DControlCommand::TauWithPeriod(self.params.period, control_output);
        self.node.control_cmd_queue.push(control_command);
    }

    fn start(&mut self) {
        #[cfg(feature = "recode")]
        {
            let file = fs::File::create(format!("{}.json", self.name)).unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
