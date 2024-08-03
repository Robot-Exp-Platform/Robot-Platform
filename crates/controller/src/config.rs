use std::sync::{Arc, Mutex, RwLock};

use crate::controller_trait::Controller;
use crate::controllers::controller_list::ControllerList;
use crate::controllers::pid::Pid;
use robot::robot_trait::Robot;

pub fn create_controller<R: Robot + 'static, const N: usize>(
    controller_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Controller>> {
    // !跟着文家新建 Controller 啦啦啦
    let name = format!("{}:{}", controller_type, robot_name);
    match controller_type.as_str() {
        "pid" => Arc::new(Mutex::new(Pid::<R, N>::new_without_params(
            name, path, robot,
        ))),
        "controller_list" => Arc::new(Mutex::new(ControllerList::new(name, path))),
        _ => panic!("Controller type not found,{}", controller_type),
    }
}
