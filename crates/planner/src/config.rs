use std::sync::{Arc, Mutex, RwLock};

use crate::planner_trait::Planner;
use crate::planners::linear::Linear;
use robot::robot_trait::Robot;

pub fn create_planner<R: Robot + 'static, const N: usize>(
    planner_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Planner>> {
    // !跟着文家新建 Planner 啦啦啦
    match planner_type.as_str() {
        "linear" => Arc::new(Mutex::new(Linear::<R, N>::new_without_params(
            robot_type + "_linear",
            path,
            robot,
        ))),
        _ => panic!("Planner type not found"),
    }
}
