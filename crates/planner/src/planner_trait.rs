use crossbeam::queue::SegQueue;
use massage::target::Target;
use massage::track::Track;
use std::sync::{Arc, Mutex};
use task_manager::ros_thread::ROSThread;

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

    fn set_params(&mut self, params: String);
    fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>);
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>);

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>);
    fn get_planner(&self) -> &Vec<Arc<Mutex<dyn Planner>>> {
        unimplemented!()
    }

    // TODO add plan function
}
