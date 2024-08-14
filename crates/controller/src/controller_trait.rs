use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use message::control_command::ControlCommand;
use message::track::Track;
use sensor::sensor_trait::Sensor;
use task_manager::ros_thread::ROSThread;

pub trait Controller: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>);
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    );

    fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        unimplemented!()
    }
}
