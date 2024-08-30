use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use message::{ControlCommandN, TrackN};
use sensor::Sensor;
use task_manager::ROSThread;

pub trait Controller: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);

    fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        unimplemented!()
    }
}

pub trait ControllerN<const N: usize>: Controller {
    fn set_track_n_queue(&mut self, track_queue: Arc<SegQueue<TrackN<N>>>);
    fn set_controller_command_n_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
    );
}
