use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use tracing::info;
// use serde_yaml::{from_value, Value};
use std::f64;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessageQueue, NodeMessageQueue};
use robot::{DSeriseRobot, Robot, RobotType};
use sensor::Sensor;

pub struct Position<R, V> {
    /// The name of the controller.
    name: String,
    /// The state of the controller.
    state: PositionState,
    /// The parameters of the controller.
    params: PositionParams,
    /// The node of the controller.
    node: PositionNode<V>,
    /// The robot that the controller is controlling.
    robot: Option<Arc<RwLock<R>>>,
}

pub type DPosition = Position<DSeriseRobot, na::DVector<f64>>;
pub type SPosition<R, const N: usize> = Position<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct PositionState {
    is_end: bool,
}

#[derive(Deserialize, Default)]
pub struct PositionParams {
    period: f64,
}

#[derive(Default)]
pub struct PositionNode<V> {
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl DPosition {
    pub fn from_json(name: String, json: Value) -> DPosition {
        Self::from_params(name, from_value(json).unwrap())
    }

    pub fn from_params(name: String, params: PositionParams) -> DPosition {
        DPosition {
            name,
            state: PositionState::default(),
            params,
            node: PositionNode::default(),
            robot: None,
        }
    }
}

impl Node<na::DVector<f64>> for DPosition {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = Some(robot);
        }
    }
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DPosition {
    fn update(&mut self) {
        if let Some(control_message) = self.node.input_queue.pop() {
            info!(node = self.name.as_str(), input = ?control_message.as_slice());
            if self.state.is_end {
                self.robot
                    .as_ref()
                    .unwrap()
                    .write()
                    .unwrap()
                    .set_control_message(control_message);
            }
        }
    }
    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
