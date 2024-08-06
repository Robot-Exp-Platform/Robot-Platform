use std::sync::{Arc, Mutex, RwLock};

use crate::planner_trait::Planner;
use crate::planners::linear::Linear;
use robot::robot_trait::Robot;

pub fn create_planner<R: Robot + 'static, const N: usize>(
    planner_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Planner>> {
    // !跟着文家新建 Planner 啦啦啦
    let name = format!("{}:{}", planner_type, robot_name);
    match planner_type.as_str() {
        "linear" => Arc::new(Mutex::new(Linear::<R, N>::new(name, path, robot))),
        _ => panic!("Planner type not found"),
    }
}
