use nalgebra as na;
use serde_json::Value;

use crate::{Cfs, DBullet, Interp, Node, Pid, Position};

pub fn create_node(
    node_type: &str,
    robot_name: String,
    params: Value,
) -> Box<dyn Node<na::DVector<f64>>> {
    let name = format!("{}:{}", node_type, robot_name);
    match node_type {
        "interp" => Box::new(Interp::from_json(name, params)),
        "cfs" => Box::new(Cfs::from_json(name, params)),

        "pid" => Box::new(Pid::from_json(name, params)),
        "position" => Box::new(Position::from_json(name, params)),

        "bullet" => Box::new(DBullet::from_json(name, params)),
        _ => panic!("Unknown node type: {}", node_type),
    }
}
