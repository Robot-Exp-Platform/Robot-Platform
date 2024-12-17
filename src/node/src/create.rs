use nalgebra as na;
use serde_json::Value;

use crate::{
    example::{ExController, ExPlanner},
    Cfs, DBullet, DImpedence, DImpedenceDiag, DPandaPlant, DPid, GripperPlant, Interp, Node,
    ObstacleReleaser, Position,
};

pub fn create_node(
    node_type: &str,
    robot_name: String,
    params: Value,
) -> Box<dyn Node<na::DVector<f64>>> {
    let name = format!("{}:{}", node_type, robot_name);
    match node_type {
        "example_planner" => Box::new(ExPlanner::from_json(name, params)),
        "interp" => Box::new(Interp::from_json(name, params)),
        "cfs" => Box::new(Cfs::from_json(name, params)),

        "example_controller" => Box::new(ExController::from_json(name, params)),
        "impedence" => Box::new(DImpedence::from_json(name, params)),
        "impedence_diag" => Box::new(DImpedenceDiag::from_json(name, params)),
        "pid" => Box::new(DPid::from_json(name, params)),
        "position" => Box::new(Position::from_json(name, params)),

        "bullet" => Box::new(DBullet::from_json(name, params)),
        "obstacle_releaser" => Box::new(ObstacleReleaser::from_json(name, params)),

        "panda_plant" => Box::new(DPandaPlant::from_json(name, params)),
        "gripper_plant" => Box::new(GripperPlant::from_json(name, params)),
        _ => panic!("Unknown node type: {}", node_type),
    }
}
