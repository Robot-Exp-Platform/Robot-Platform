use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, RwLock};

use manager::Node;
use message::{DControlCommand, DTrack, SControlCommand, STrack};
use sensor::Sensor;

pub trait Controller: Node {
    fn name(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
}

pub trait SController<const N: usize>: Controller {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<STrack<N>>>);
    fn set_control_cmd_queue(&mut self, control_cmd_queue: Arc<SegQueue<SControlCommand<N>>>);
}

pub trait DController: Controller {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<DTrack>>);
    fn set_control_cmd_queue(&mut self, control_cmd_queue: Arc<SegQueue<DControlCommand>>);
}
