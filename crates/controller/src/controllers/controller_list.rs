use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex, RwLock};

use crate::Controller;
use sensor::Sensor;
use task_manager::ROSThread;

pub struct ControllerList {
    name: String,
    path: String,

    controllers: Vec<Arc<Mutex<dyn Controller>>>,
}

impl ControllerList {
    pub fn new(name: String, path: String) -> ControllerList {
        ControllerList::from_controllers(name, path, vec![])
    }

    pub fn from_controllers(
        name: String,
        path: String,
        controllers: Vec<Arc<Mutex<dyn Controller>>>,
    ) -> ControllerList {
        ControllerList {
            name,
            path,
            controllers,
        }
    }
}

impl Controller for ControllerList {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        &self.controllers
    }

    fn set_params(&mut self, _: Value) {}
    fn set_sensor(&mut self, _: Arc<RwLock<Sensor>>) {}

    fn add_controller(&mut self, controller: Arc<Mutex<dyn Controller>>) {
        self.controllers.push(controller)
    }
}

impl ROSThread for ControllerList {}
