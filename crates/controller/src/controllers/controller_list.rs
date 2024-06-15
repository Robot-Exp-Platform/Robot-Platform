use crate::controller_trait::Controller;

pub struct ControllerList {
    name: String,
    path: String,

    controllers: Vec<Box<dyn Controller>>,
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
        controllers: Vec<Box<dyn Controller>>,
    ) -> ControllerList {
        ControllerList {
            name,
            path,
            controllers,
        }
    }

    pub fn add_controller(&mut self, controller: Box<dyn Controller>) {
        self.controllers.push(controller)
    }
}

impl Controller for ControllerList {
    fn get_name(&self) -> String {
        let names =
            apply_closure_to_iter!(self.controllers, |controller| controller.get_name()).join(", ");
        format!("{}:{{{}}}", self.name, names)
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn init(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.init())
    }

    fn starting(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.starting())
    }

    fn update(&mut self, time: f64) {
        self.controllers
            .iter_mut()
            .for_each(|controller| controller.update(time))
    }

    fn stopping(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.stopping())
    }

    fn waiting(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.waiting())
    }

    fn aborting(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.aborting())
    }

    fn init_request(&self) {
        self.controllers
            .iter()
            .for_each(|controller| controller.init_request())
    }
}
