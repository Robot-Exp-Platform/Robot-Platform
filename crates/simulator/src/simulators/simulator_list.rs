use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use crate::Simulator;
use sensor::Sensor;
use task_manager::ROSThread;

pub struct SimulatorList {
    name: String,
    path: String,

    simulators: Vec<Arc<Mutex<dyn Simulator>>>,
}

impl SimulatorList {
    pub fn new(name: String, path: String) -> SimulatorList {
        SimulatorList::from_simulators(name, path, vec![])
    }
    pub fn from_simulators(
        name: String,
        path: String,
        simulators: Vec<Arc<Mutex<dyn Simulator>>>,
    ) -> SimulatorList {
        SimulatorList {
            name,
            path,
            simulators,
        }
    }
}

impl Simulator for SimulatorList {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_simulator(&self) -> &Vec<Arc<Mutex<dyn Simulator>>> {
        &self.simulators
    }

    fn set_params(&mut self, _: Value) {}
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}

    fn add_simulator(&mut self, simulator: Arc<Mutex<dyn Simulator>>) {
        self.simulators.push(simulator)
    }
}

impl ROSThread for SimulatorList {}
