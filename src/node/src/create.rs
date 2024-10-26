use nalgebra as na;
use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::{Cfs, Interp, Node};
use robot::DSeriseRobot;

pub fn create_node(
    node_type: &str,
    robot_name: String,
    robot: Arc<RwLock<DSeriseRobot>>,
    params: Value,
) -> Box<dyn Node<na::DVector<f64>>> {
    let name = format!("{}:{}", node_type, robot_name);
    match node_type {
        "interp" => Box::new(Interp::from_json(name, robot, params)),
        "cfs" => Box::new(Cfs::from_json(name, robot, params)),
        _ => panic!("Unknown node type: {}", node_type),
    }
}
