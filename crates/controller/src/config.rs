use std::sync::{Arc, RwLock};

use crate::controller_trait::Controller;
use crate::controllers::controller_list::ControllerList;
use crate::controllers::pid::Pid;
use robot::robot_trait::Robot;

pub fn create_controller<R: Robot + 'static, const N: usize>(
    controller_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Controller> {
    match controller_type.as_str() {
        "pid" => Box::new(Pid::<R, N>::new_without_params(
            robot_type + "_pid",
            path,
            robot,
        )),
        "controller_list" => Box::new(ControllerList::new(robot_type + "_controllers", path)),
        _ => panic!("Controller type not found"),
    }
}
