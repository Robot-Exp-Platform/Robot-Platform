use robot::DRobot;
use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{Cfs, Interp, Planner};

pub fn create_planner<R: DRobot + 'static>(
    planner_type: &str,
    robot_name: String,
    robot: Arc<RwLock<R>>,
    json: Value,
) -> Box<dyn Planner> {
    let name = format!("{}:{}", planner_type, robot_name);
    match planner_type {
        "interp" => Box::new(Interp::from_json(name, robot, json)),
        "cfs" => Box::new(Cfs::from_json(name, robot, json)),
        _ => panic!("Unknown planner type: {}", planner_type),
    }
}
