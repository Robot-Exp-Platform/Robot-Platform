use std::sync::{Arc, RwLock};

use crate::planner_trait::Planner;
use crate::planners::linear::Linear;
use robot::robot_trait::Robot;

pub fn create_planner<R: Robot + 'static, const N: usize>(
    planner_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Planner> {
    match planner_type.as_str() {
        "linear" => Box::new(Linear::<R, N>::new_without_params(
            robot_type + "_linear",
            path,
            robot,
        )),
        // "planner_list" => Box::new(PlannerList::new(robot_type + "_planners", path)),
        _ => panic!("Planner type not found"),
    }
}
