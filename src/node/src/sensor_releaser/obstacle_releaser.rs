use crossbeam::queue::SegQueue;
use nalgebra as na;
use robot::RobotType;
use serde_json::{from_value, Value};
use std::sync::{Arc, RwLock};

use crate::{Node, NodeBehavior, NodeState};
use generate_tools::*;
use sensor::Sensor;
use serde::Deserialize;

pub struct ObstacleReleaser {
    /// The name of the releaser.
    name: String,
    /// The state of the releaser.
    state: ObstacleReleaserState,
    /// The parameters of the releaser.
    params: ObstacleReleaserParams,
    // /// The node of the releaser.
    // node: ObstacleReleaserNode,
    /// The sensor that the releaser is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

struct ObstacleReleaserState {
    is_end: bool,
    node_state: NodeState,
}

#[derive(Deserialize)]
pub struct ObstacleReleaserParams {
    period: f64,
}

// struct ObstacleReleaserNode {}

impl ObstacleReleaser {
    pub fn new(name: String) -> ObstacleReleaser {
        ObstacleReleaser::from_params(name, ObstacleReleaserParams { period: 0.0 })
    }

    pub fn from_json(name: String, params: serde_json::Value) -> ObstacleReleaser {
        ObstacleReleaser::from_params(name, serde_json::from_value(params).unwrap())
    }

    pub fn from_params(name: String, params: ObstacleReleaserParams) -> ObstacleReleaser {
        ObstacleReleaser {
            name,
            state: ObstacleReleaserState {
                is_end: false,
                node_state: NodeState::default(),
            },
            params,
            // node: ObstacleReleaserNode {},
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for ObstacleReleaser {
    get_fn!((name: String));

    fn set_input_queue(&mut self, _: Arc<SegQueue<message::NodeMessage<na::DVector<f64>>>>) {}
    fn set_output_queue(&mut self, _: Arc<SegQueue<message::NodeMessage<na::DVector<f64>>>>) {}

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, _: RobotType) {}

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }

    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for ObstacleReleaser {
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }

    fn state(&mut self) -> crate::NodeState {
        self.state.node_state
    }
}
