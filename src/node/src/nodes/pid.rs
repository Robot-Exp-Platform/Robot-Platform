use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;
use std::{f64, fs};

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue, NodeMessageQueue};
#[cfg(feature = "recode")]
use recoder::*;
use robot::{DSeriseRobot, Robot, RobotType, SRobot};
use sensor::Sensor;

pub struct Pid<R, V, M> {
    /// The name of the controller.
    name: String,
    /// The path of the controller.
    state: PidState<V>,
    /// The parameters of the controller.
    params: PidParams<M>,
    /// The node of the controller.
    node: PidNode<V>,
    /// The robot that the controller is controlling.
    robot: Option<Arc<RwLock<R>>>,
    /// The sensor that the controller is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DPid = Pid<DSeriseRobot, na::DVector<f64>, na::DMatrix<f64>>;
pub type SPid<R, const N: usize> = Pid<R, na::SVector<f64, N>, na::SMatrix<f64, N, N>>;

#[derive(Default)]
pub struct PidState<V> {
    track: V,
    error: V,
    integral: V,
    derivative: V,
}

#[derive(Deserialize, Default)]
pub struct PidParams<M> {
    period: f64,
    kp: M,
    ki: M,
    kd: M,
}

#[derive(Default)]
pub struct PidNode<V> {
    recoder: Option<BufWriter<fs::File>>,
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl DPid {
    pub fn from_json(name: String, json: Value) -> DPid {
        DPid::from_params(name, from_value(json).unwrap())
    }

    pub fn from_params(name: String, params: PidParams<na::DMatrix<f64>>) -> DPid {
        DPid {
            name,
            state: PidState::default(),
            params,

            node: PidNode::default(),
            robot: None,
            sensor: None,
        }
    }
}

impl<R: SRobot<N>, const N: usize> SPid<R, N> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> SPid<R, N> {
        SPid::from_params(
            name,
            PidParams {
                period: 0.0,
                kp: na::SMatrix::zeros(),
                ki: na::SMatrix::zeros(),
                kd: na::SMatrix::zeros(),
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        params: PidParams<na::SMatrix<f64, N, N>>,
        robot: Arc<RwLock<R>>,
    ) -> SPid<R, N> {
        SPid {
            name,
            state: PidState {
                track: na::SVector::zeros(),
                error: na::SVector::zeros(),
                integral: na::SVector::zeros(),
                derivative: na::SVector::zeros(),
            },
            params,

            node: PidNode {
                recoder: None,
                input_queue: Arc::new(SegQueue::new()),
                output_queue: Arc::new(SegQueue::new()),
            },
            robot: Some(robot),
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DPid {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = Some(robot);
        }
    }
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DPid {
    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();

        // 更新 Track
        self.state.track = match self.node.input_queue.pop() {
            Some(DNodeMessage::Joint(track)) => track,
            _ => return,
        };

        // 执行 pid 逻辑
        let new_error = &self.state.track - &q;
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
            .output_queue
            .push(DNodeMessage::JointWithPeriod(self.params.period, output));
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

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
