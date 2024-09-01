use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use crate::{Planner, PlannerState};
use sensor::Sensor;
use task_manager::ROSThread;

pub struct PlannerList {
    name: String,
    path: String,

    planners: Vec<Arc<Mutex<dyn Planner>>>,
}

impl PlannerList {
    pub fn new(name: String, path: String) -> PlannerList {
        PlannerList::from_planners(name, path, vec![])
    }

    pub fn from_planners(
        name: String,
        path: String,
        planners: Vec<Arc<Mutex<dyn Planner>>>,
    ) -> PlannerList {
        PlannerList {
            name,
            path,
            planners,
        }
    }
}

impl Planner for PlannerList {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_planner(&self) -> &Vec<Arc<Mutex<dyn Planner>>> {
        &self.planners
    }
    fn get_planner_state(&self) -> PlannerState {
        PlannerState::Unknow
    }

    fn set_params(&mut self, _: Value) {}
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}
    fn set_state_collector(&mut self, _: task_manager::StateCollector) {}
    fn set_target_queue(&mut self, _: Arc<crossbeam::queue::SegQueue<message::Target>>) {}

    fn add_planner(&mut self, planner: Arc<Mutex<dyn Planner>>) {
        self.planners.push(planner);
    }
}

impl ROSThread for PlannerList {}
