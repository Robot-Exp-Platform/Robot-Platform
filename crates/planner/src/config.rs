use std::sync::{Arc, RwLock};

use crate::planner_trait::Planner;
use crate::planners::linear::Linear;
use robot::robot_trait::Robot;
use robot::robots::panda;

pub fn build_planner<R: Robot + 'static>(
    planner_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Planner> {
    match planner_type.as_str() {
        "linear" => match robot_type.as_str() {
            "panda" => Box::new(Linear::<R, { panda::PANDA_DOF + 1 }>::new_without_params(
                robot_type + "_linear",
                path,
                robot,
            )),
            _ => panic!("linear planner does not support this kind of robot"),
        },
        // "planner_list" => Box::new(PlannerList::new(robot_type + "_planners", path)),
        _ => panic!("Planner type not found"),
    }
}
