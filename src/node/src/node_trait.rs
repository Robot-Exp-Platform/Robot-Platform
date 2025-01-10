use generate_tools::{get_fn, set_fn};
use sensor::Sensor;
use serde::de::DeserializeOwned;
use serde_json::{from_value, Value};
use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use message::NodeMessageQueue;
use robot::{DownCastRobot, RobotType};

pub trait NodeExt<V> {
    fn name(&self) -> String;
    fn set_input_queue(&mut self, input_queue: NodeMessageQueue<V>);
    fn set_output_queue(&mut self, output_queue: NodeMessageQueue<V>);
    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn set_robot(&mut self, robot: RobotType);
}

// TODO consider using state machine to manage the node
pub trait NodeBehavior: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn finalize(&mut self) {}

    fn state(&mut self) -> NodeState {
        NodeState::Running
    }
    fn period(&self) -> Duration {
        Duration::from_secs(0)
    }
    fn node_name(&self) -> String {
        String::from("unnamed_node")
    }
    fn node_type(&self) -> String {
        String::from("unnamed_type")
    }
}

pub struct Node<S, P, R, V>
where
    S: Default,
    P: DeserializeOwned,
{
    pub(crate) name: String,
    pub(crate) node_state: NodeState,
    pub(crate) is_end: bool,
    pub(crate) input_queue: NodeMessageQueue<V>,
    pub(crate) output_queue: NodeMessageQueue<V>,

    pub state: S,
    pub params: P,
    pub robot: R,
    pub sensor: Option<Arc<RwLock<Sensor>>>,
}

impl<S, P, R, V> Node<S, P, R, V>
where
    S: Default,
    P: DeserializeOwned,
    R: Default,
{
    pub fn from_params(name: String, params: Value) -> Self {
        Node {
            name,
            node_state: NodeState::Init,
            is_end: false,
            input_queue: NodeMessageQueue::default(),
            output_queue: NodeMessageQueue::default(),
            state: S::default(),
            params: from_value(params).unwrap(),
            robot: R::default(),
            sensor: None,
        }
    }
}

impl<S, P, R, V> NodeExt<V> for Node<S, P, R, V>
where
    S: Default,
    P: DeserializeOwned,
    R: DownCastRobot + Clone,
{
    get_fn!((name: String));
    set_fn!(
        (set_input_queue, input_queue: NodeMessageQueue<V>),
        (set_output_queue, output_queue: NodeMessageQueue<V>)
    );

    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }

    fn set_robot(&mut self, robot: RobotType) {
        self.robot = R::downcast_robot(robot, self.robot.clone());
    }

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum NodeState {
    Init,
    #[default]
    Running,
    RelyRelease,
    Finished,
}
