use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::ops::{Mul, Sub};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::controller_trait::Controller;
use crate::TController;
use generate_tools::{get_fn, set_fn};
use manager::Node;
use message::{ControlCommand, Track};
#[cfg(feature = "recode")]
use recoder::*;
use robot::{DRobot, Robot};
use sensor::Sensor;

pub struct Impedence<R, V, M>
where
    V: Sub + Send + Sync,
    M: Mul<V, Output = V> + Sub + Send + Sync,
{
    /// The name of the controller.
    name: String,
    /// The path of the controller.
    state: ImpedenceState<V>,
    /// The parameters of the controller.
    params: ImpedenceParams<M>,
    /// The node of the controller.
    node: ImpedenceNode<V>,
    /// The robot that the controller is controlling.
    robot: Arc<RwLock<R>>,
}

pub type DImpedence<R> = Impedence<R, na::DVector<f64>, na::DMatrix<f64>>;
pub type SImpedence<R, const N: usize> = Impedence<R, na::SVector<f64, N>, na::SMatrix<f64, N, N>>;

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

#[derive(Default)]
pub struct ImpedenceNode<V> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<Track<V>>>,
    control_cmd_queue: Arc<SegQueue<ControlCommand<V>>>,
}

impl<R: DRobot> DImpedence<R> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> DImpedence<R> {
        let ndof = robot.read().unwrap().dof();
        DImpedence::from_params(
            name,
            ImpedenceParams {
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
        params: ImpedenceParams<na::DMatrix<f64>>,
        robot: Arc<RwLock<R>>,
    ) -> DImpedence<R> {
        let ndof = robot.read().unwrap().dof();
        Impedence {
            name,
            state: ImpedenceState {
                ref_q: na::DVector::zeros(ndof),
                ref_q_dot: na::DVector::zeros(ndof),
                ref_q_ddot: na::DVector::zeros(ndof),
            },
            params,
            node: ImpedenceNode::default(),
            robot,
        }
    }
}

impl<R: Robot<V>, V, M> TController<V> for Impedence<R, V, M>
where
    V: Sub + Send + Sync,
    M: Mul<V, Output = V> + Sub + Send + Sync,
{
    set_fn!((set_track_queue, track_queue: Arc<SegQueue<Track<V>>>, node),
            (set_control_cmd_queue, control_cmd_queue: Arc<SegQueue<ControlCommand<V>>>, node));
}

impl<R: Robot<V>, V, M> Controller for Impedence<R, V, M>
where
    V: Sub + Send + Sync,
    M: Mul<V, Output = V> + Sub + Send + Sync,
{
    get_fn!((name: String));

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl<R: Robot<V>, V, M> Node for Impedence<R, V, M>
where
    V: Sub + Send + Sync,
    M: Mul<V, Output = V> + Sub + Send + Sync,
{
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
            Some(Track::Joint(ref_q)) => {
                self.state.ref_q = ref_q;
                println!("{} get track: {:?}", self.name, self.state.ref_q);
            }
            Some(Track::JointVel(ref_q, ref_q_dot)) => {
                self.state.ref_q = ref_q;
                self.state.ref_q_dot = ref_q_dot;
                println!(
                    "{} get track: {:?}, {:?}",
                    self.name, self.state.ref_q, self.state.ref_q_dot
                );
            }
            Some(Track::JointVelAcc(ref_q, ref_q_dot, ref_q_ddot)) => {
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
            + &self.params.b * (&self.state.ref_q_dot - &q_dot)
            + &self.params.m * (&self.state.ref_q_ddot - &q_ddot);

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, control_output);
        }

        // 发送控制指令
        let control_command = ControlCommand::TauWithPeriod(self.params.period, control_output);
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
