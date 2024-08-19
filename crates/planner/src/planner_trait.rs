use crossbeam::queue::SegQueue;
use message::target::Target;
use message::track::Track;
use serde_json::Value;
// use serde_yaml::Value;
use sensor::sensor_trait::Sensor;
use std::sync::{Arc, Mutex, RwLock};
use task_manager::ros_thread::ROSThread;
use task_manager::state_collector::StateCollector;

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
    fn get_planner(&self) -> &Vec<Arc<Mutex<dyn Planner>>> {
        unimplemented!()
    }

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>);
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>);
    fn set_state_collector(&mut self, state_collector: StateCollector);

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>);
}
