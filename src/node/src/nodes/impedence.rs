use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::ops::{Mul, Sub};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue, NodeMessageQueue};
#[cfg(feature = "recode")]
use recoder::*;
use robot::{DSeriseRobot, Robot, RobotType};
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
    robot: Option<Arc<RwLock<R>>>,
    /// The sensor that the controller is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DImpedence = Impedence<DSeriseRobot, na::DVector<f64>, na::DMatrix<f64>>;
pub type SImpedence<R, const N: usize> = Impedence<R, na::SVector<f64, N>, na::SMatrix<f64, N, N>>;

#[derive(Default)]
pub struct ImpedenceState<V> {
    is_end: bool,
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
    recoder: Option<BufWriter<fs::File>>,
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl ImpedenceState<na::DVector<f64>> {
    pub fn new(dof: usize) -> ImpedenceState<na::DVector<f64>> {
        ImpedenceState {
            is_end: false,
            ref_q: na::DVector::zeros(dof),
            ref_q_dot: na::DVector::zeros(dof),
            ref_q_ddot: na::DVector::zeros(dof),
        }
    }
}

impl DImpedence {
    pub fn from_json(name: String, json: Value) -> DImpedence {
        DImpedence::from_params(name, from_value(json).unwrap())
    }
    pub fn from_params(name: String, params: ImpedenceParams<na::DMatrix<f64>>) -> DImpedence {
        DImpedence {
            name,
            state: ImpedenceState::default(),
            params,
            node: ImpedenceNode::default(),
            robot: None,
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DImpedence {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.state = ImpedenceState::new(robot.read().unwrap().dof());
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

impl NodeBehavior for DImpedence {
    fn update(&mut self) {
        // 获取 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        let q_dot = robot_read.q_dot();
        let q_ddot = robot_read.q_ddot();

        // TODO 检查任务是否完成

        match self.node.input_queue.pop() {
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

        // 记录控制指令
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            recode!(recoder, output);
        }

        let control_message = DNodeMessage::Joint(output);

        // 发送控制指令
        if self.state.is_end {
            self.robot
                .as_ref()
                .unwrap()
                .write()
                .unwrap()
                .set_control_message(control_message);
        } else {
            self.node.output_queue.push(control_message);
        }
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
