use crossbeam::queue::SegQueue;
use serde_json::Value;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use message::NodeMessage;
use robot::RobotType;
use sensor::Sensor;

pub trait Node<V>: NodeBehavior {
    /// Get the name of the node.
    fn name(&self) -> String;

    /// Set the parameters of the node from a JSON object.
    fn set_params(&mut self, params: Value);
    /// Set the robot that the node is controlling.
    fn set_robot(&mut self, robot: RobotType);
    /// Set the sensor that the node is using.
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);

    /// Set the input queue of the node.Each node has a input queue.
    fn set_input_queue(&mut self, input_queue: Arc<SegQueue<NodeMessage<V>>>);
    /// Set the output queue of the node.Each nod has at least one output queue.
    /// Every use of this function will add a new output queue for the node.
    fn set_output_queue(&mut self, output_queue: Arc<SegQueue<NodeMessage<V>>>);

    fn is_end(&mut self) {}
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
        String::from("unnamed_thread")
    }
    fn node_type(&self) -> String {
        String::from("unnamed_type")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum NodeState {
    #[default]
    Init,
    Running,
    RelyRelease,
    Finished,
}
