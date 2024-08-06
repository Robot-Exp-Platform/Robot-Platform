// use serde_json::Value;
use serde_yaml::Value;
use std::sync::{Arc, Mutex};

use crate::controller_trait::Controller;
use task_manager::ros_thread::ROSThread;

pub struct ControllerList {
    name: String,
    path: String,

    controllers: Vec<Arc<Mutex<dyn Controller>>>,
}

macro_rules! apply_closure_to_iter {
    ($controllers:expr,$closure:expr) => {
        $controllers.iter().map($closure).collect::<Vec<_>>()
    };
}

impl ControllerList {
    pub fn new(name: String, path: String) -> ControllerList {
        ControllerList::from_controllers(name, path, Vec::new())
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
        let names = apply_closure_to_iter!(self.controllers, |controller| controller
            .lock()
            .unwrap()
            .get_name())
        .join(", ");
        format!("{}:{{{}}}", self.name, names)
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn set_params(&mut self, _: Value) {}
    fn set_track_queue(&mut self, _: Arc<crossbeam::queue::SegQueue<message::track::Track>>) {}
    fn set_controller_command_queue(
        &mut self,
        _: Arc<crossbeam::queue::SegQueue<message::control_command::ControlCommand>>,
    ) {
    }

    fn add_controller(&mut self, controller: Arc<Mutex<dyn Controller>>) {
        self.controllers.push(controller)
    }
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        &self.controllers
    }
}

impl ROSThread for ControllerList {
    fn init(&mut self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().init())
    }

    fn start(&mut self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().start())
    }

    fn update(&mut self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().update())
    }
}
