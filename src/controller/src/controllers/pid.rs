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
use message::{DControlCommand, DTrack, SControlCommand, STrack};
#[cfg(feature = "recode")]
use recoder::*;
use robot::{DRobot, Robot};
use sensor::Sensor;

pub struct Pid<R: Robot<V>, V, M, TRK, CMD> {
    /// The name of the controller.
    name: String,
    /// The path of the controller.
    state: PidState<V>,
    /// The parameters of the controller.
    params: PidParams<M>,
    /// The node of the controller.
    node: PidNode<TRK, CMD>,
    /// The robot that the controller is controlling.
    robot: Arc<RwLock<R>>,
}

pub type DPid<R> = Pid<R, na::DVector<f64>, na::DMatrix<f64>, DTrack, DControlCommand>;
pub type SPid<R, const N: usize> =
    Pid<R, na::SVector<f64, N>, na::SMatrix<f64, N, N>, STrack<N>, SControlCommand<N>>;

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

#[derive(Default)]
pub struct PidNode<TRK, CMD> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    track_queue: Arc<SegQueue<TRK>>,
    control_cmd_queue: Arc<SegQueue<CMD>>,
}

impl<R: DRobot> DPid<R> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> DPid<R> {
        let ndof = robot.read().unwrap().dof();
        DPid::from_params(
            name,
            PidParams {
                period: 0.0,
                kp: na::DMatrix::zeros(ndof, ndof),
                ki: na::DMatrix::zeros(ndof, ndof),
                kd: na::DMatrix::zeros(ndof, ndof),
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        params: PidParams<na::DMatrix<f64>>,
        robot: Arc<RwLock<R>>,
    ) -> DPid<R> {
        let ndof = robot.read().unwrap().dof();
        DPid {
            name,
            state: PidState {
                track: na::DVector::zeros(ndof),
                error: na::DVector::zeros(ndof),
                integral: na::DVector::zeros(ndof),
                derivative: na::DVector::zeros(ndof),
            },
            params,

            node: PidNode::default(),
            robot,
        }
    }
}

impl<R: DRobot> DController for DPid<R> {
    set_fn!((set_track_queue, track_queue: Arc<SegQueue<DTrack>>, node),
            (set_control_cmd_queue, control_cmd_queue: Arc<SegQueue<DControlCommand>>, node));
}

impl<R: DRobot> Controller for DPid<R> {
    get_fn!((name: String));

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl<R: DRobot> Node for DPid<R> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.q();

        // 更新 Track
        self.state.track = match self.node.track_queue.pop() {
            Some(DTrack::Joint(track)) => track,
            _ => return,
        };

        println!("{} get track: {:?}", self.name, self.state.track);

        // 执行 pid 逻辑
        let new_error = &self.state.track - q;
        self.state.integral += &new_error * self.params.period;
        self.state.derivative = (&new_error - &self.state.error) / self.params.period;
        self.state.error = new_error;

        let output = &self.params.kp * &self.state.error
            + &self.params.ki * &self.state.integral
            + &self.params.kd * &self.state.derivative;

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, output);
        }

        // 发送控制指令
        self.node
            .control_cmd_queue
            .push(DControlCommand::JointWithPeriod(self.params.period, output));
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

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
