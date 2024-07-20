use std::sync::{Arc, Mutex};

use crate::controller_trait::Controller;
use robot::ros_thread::ROSThread;

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
        ControllerList::new_with_controllers(name, path, Vec::new())
    }

    pub fn new_with_controllers(
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

    pub fn add_controller(&mut self, controller: Arc<Mutex<dyn Controller>>) {
        self.controllers.push(controller)
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

    fn set_params(&mut self, _: String) {}

    fn add_controller(&mut self, controller: Arc<Mutex<dyn Controller>>) {
        self.controllers.push(controller)
    }
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        &self.controllers
    }
}

impl ROSThread for ControllerList {
    fn init(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().init())
    }

    fn start(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().start())
    }

    fn update(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.lock().unwrap().update())
    }
}
