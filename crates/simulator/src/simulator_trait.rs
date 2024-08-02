use std::sync::{Arc, Mutex};
use task_manager::ros_thread::ROSThread;
pub trait Simulator: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn add_simulator(&mut self, _: Arc<Mutex<dyn Simulator>>) {}
}
