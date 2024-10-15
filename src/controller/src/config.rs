use robot::DRobot;
use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{Controller, Pid};

pub fn create_controller<R: DRobot + 'static>(
    controller_type: &str,
    robot_name: String,
    robot: Arc<RwLock<R>>,
    json: Value,
) -> Box<dyn Controller> {
    match controller_type {
        "pid" => Box::new(Pid::from_json(robot_name, robot, json)),
        _ => panic!("Unknown controller type: {}", controller_type),
    }
}
