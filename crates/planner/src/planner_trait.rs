use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use message::{Target, TrackN};
use sensor::Sensor;
use task_manager::{ROSThread, StateCollector};

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
    fn set_state_collector(&mut self, state_collector: StateCollector);

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>);
}

pub trait PlannerN<const N: usize>: Planner {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<TrackN<N>>>);
}
