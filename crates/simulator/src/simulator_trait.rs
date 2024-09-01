use crossbeam::channel::{Receiver, Sender};
use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use message::ControlCommandN;
use sensor::Sensor;
use task_manager::ROSThread;

pub trait Simulator: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_simulator(&self) -> &Vec<Arc<Mutex<dyn Simulator>>> {
        unimplemented!()
    }

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn subscribe_post_office(
        &mut self,
        sender: Sender<(String, String)>,
        receiver: Receiver<String>,
    );

    fn add_simulator(&mut self, _: Arc<Mutex<dyn Simulator>>) {}
}

pub trait SimulatorN<const N: usize>: Simulator {
    fn set_controller_command_queue(
        &mut self,
        control_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
    );
}
