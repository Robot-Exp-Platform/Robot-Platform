use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use nalgebra as na;
use std::sync::{Arc, RwLock};

use node::NodeBehavior;
use message::{ControlCommand, Track};
use sensor::Sensor;

pub trait Controller: NodeBehavior {
    fn name(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
}

pub trait TController<V>: Controller {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track<V>>>);
    fn set_control_cmd_queue(&mut self, control_cmd_queue: Arc<SegQueue<ControlCommand<V>>>);
}

pub trait DController = TController<na::DVector<f64>>;
pub trait SController<const N: usize> = TController<na::SVector<f64, N>>;
