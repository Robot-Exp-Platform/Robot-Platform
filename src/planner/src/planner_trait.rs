use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, RwLock};

use manager::Node;
use message::{DTrack, STrack, Target};
use sensor::Sensor;

pub trait Planner: Node {
    fn name(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>);
    fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>);
}

pub trait SPlanner<const N: usize>: Planner {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<STrack<N>>>);
}

pub trait DPlanner: Planner {
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<DTrack>>);
}
