use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{Cfs, Interp, Planner};
use robot::RobotType;

pub fn create_planner(
    planner_type: &str,
    robot_name: String,
    robot: Arc<RwLock<RobotType>>,
    json: Value,
) -> Box<dyn Planner> {
    match planner_type {
        "interp" => Box::new(Interp::from_json(robot_name, robot, json)),
        "cfs" => Box::new(Cfs::from_json(robot_name, robot, json)),
        _ => panic!("Unknown planner type: {}", planner_type),
    }
}
