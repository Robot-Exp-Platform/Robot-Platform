use std::sync::{Arc, Mutex, RwLock};

use crate::Controller;
use crate::Impedance;
use crate::Pid;
use robot::SeriesRobot;

pub fn create_controller<R: SeriesRobot<N> + 'static, const N: usize>(
    controller_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Controller>> {
    // !跟着文家新建 Controller 啦啦啦
    let name = format!("{}:{}", controller_type, robot_name);
    match controller_type.as_str() {
        "pid" => Arc::new(Mutex::new(Pid::<R, N>::new(name, path, robot))),
        "impedance" => Arc::new(Mutex::new(Impedance::<R, N>::new(name, path, robot))),
        _ => panic!("Controller type not found,{}", controller_type),
    }
}
