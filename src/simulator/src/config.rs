use std::sync::{Arc, RwLock};

use robot::DRobot;

use crate::{DBullet, DSimulator};

pub struct SimulatorConfig {
    pub simulator_type: String,
}

pub fn create_simulator(
    simulator_type: &str,
    robot_name: String,
    robot: Arc<RwLock<dyn DRobot>>,
) -> Box<dyn DSimulator> {
    match simulator_type {
        "bullet" => Box::new(DBullet::new(robot_name, robot)),
        _ => panic!("Unknown simulator type: {}", simulator_type),
    }
}
