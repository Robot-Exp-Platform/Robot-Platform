use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{Controller, Pid};
use robot::RobotType;

pub fn create_controller(
    controller_type: &str,
    robot_name: String,
    robot: Arc<RwLock<RobotType>>,
    json: Value,
) -> Box<dyn Controller> {
    match controller_type {
        "pid" => Box::new(Pid::from_json(robot_name, robot, json)),
        _ => panic!("Unknown controller type: {}", controller_type),
    }
}
