use std::sync::{Arc, RwLock};

use crate::controller_trait::Controller;
use crate::controllers::controller_list::ControllerList;
use crate::controllers::pid::Pid;
use robot::robot_trait::Robot;
use robot::robots::panda;

pub fn build_controller<R: Robot + 'static>(
    controller_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Controller> {
    match controller_type.as_str() {
        "pid" => match robot_type.as_str() {
            "panda" => Box::new(Pid::<R, { panda::PANDA_DOF + 1 }>::new_without_params(
                robot_type + "_pid",
                path,
                robot,
            )),
            _ => panic!("pid controller does not support this kind of robot"),
        },
        "controller_list" => Box::new(ControllerList::new(robot_type + "_controllers", path)),
        _ => panic!("Controller type not found"),
    }
}
