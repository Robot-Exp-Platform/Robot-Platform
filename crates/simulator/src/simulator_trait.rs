use crossbeam::queue::SegQueue;
use serde_json::Value as JsonValue;
use std::sync::{Arc, Mutex};

use message::control_command::ControlCommand;
use task_manager::ros_thread::ROSThread;

pub trait Simulator: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_simulator(&self) -> &Vec<Arc<Mutex<dyn Simulator>>> {
        unimplemented!()
    }

    fn set_params(&mut self, params: JsonValue);
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    );

    fn add_simulator(&mut self, _: Arc<Mutex<dyn Simulator>>) {}
}
