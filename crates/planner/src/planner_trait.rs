use std::sync::{Arc, Mutex};
use robot::ros_thread::ROSThread;

#[derive(Clone, Copy)]
pub enum PlannerState {
    Unknow,
    Uninit,
    Running,
}

pub trait Planner: ROSThread {
    fn get_planner_state(&self) -> PlannerState {
        PlannerState::Unknow
    }
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_params(&self) -> Vec<f64>;

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>);

    // TODO add plan function
}
