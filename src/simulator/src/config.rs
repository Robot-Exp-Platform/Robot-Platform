use std::sync::{Arc, RwLock};

use robot::DRobot;

use crate::DSimulator;

pub struct SimulatorConfig {
    pub simulator_type: String,
}

pub fn create_simulator<R: DRobot + 'static>(
    simulator_type: &str,
    _robot_name: String,
    _robot: Arc<RwLock<dyn DRobot>>,
) -> Box<dyn DSimulator> {
    match simulator_type {
        _ => panic!("Unknown simulator type: {}", simulator_type),
    }
}
