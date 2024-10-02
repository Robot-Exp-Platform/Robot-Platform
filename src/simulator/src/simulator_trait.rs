use crossbeam::channel::{Receiver, Sender};
use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, RwLock};

use manager::Node;
use message::{DControlCommand, SControlCommand};
use sensor::Sensor;

pub trait Simulator: Node {
    fn name(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn subscribe_post_office(
        &mut self,
        sender: Sender<(String, String)>,
        receiver: Receiver<String>,
    );
}

pub trait SSimulator<const N: usize>: Simulator {
    fn set_controller_cmd_queue(&mut self, control_cmd_queue: Arc<SegQueue<SControlCommand<N>>>);
}

pub trait DSimulator: Simulator {
    fn set_control_cmd_queue(&mut self, control_cmd_queue: Arc<SegQueue<DControlCommand>>);
}