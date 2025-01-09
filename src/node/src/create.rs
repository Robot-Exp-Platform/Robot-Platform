use nalgebra as na;
use serde_json::Value;

use crate::{
    example::{ExController, ExPlanner},
    Cfs, DBullet, DImpedence, DImpedenceDiag, DPandaPlant, DPid, DPosition, GripperPlant, Interp,
    NodeBehavior, NodeExt, ObstacleReleaser,
};

pub trait NodeExtBehavior: NodeExt<na::DVector<f64>> + NodeBehavior {}

impl<T> NodeExtBehavior for T where T: NodeExt<na::DVector<f64>> + NodeBehavior {}

pub fn create_node(node_type: &str, robot_name: String, params: Value) -> Box<dyn NodeExtBehavior> {
    let name = format!("{}:{}", node_type, robot_name);
    match node_type {
        "example_planner" => Box::new(ExPlanner::from_params(name, params)),
        "interp" => Box::new(Interp::from_params(name, params)),
        "cfs" => Box::new(Cfs::from_params(name, params)),

        "example_controller" => Box::new(ExController::from_params(name, params)),
        "impedence" => Box::new(DImpedence::from_params(name, params)),
        "impedence_diag" => Box::new(DImpedenceDiag::from_params(name, params)),
        "pid" => Box::new(DPid::from_params(name, params)),
        "position" => Box::new(DPosition::from_params(name, params)),

        "bullet" => Box::new(DBullet::from_params(name, params)),
        "obstacle_releaser" => Box::new(ObstacleReleaser::from_params(name, params)),

        "panda_plant" => Box::new(DPandaPlant::from_params(name, params)),
        "gripper_plant" => Box::new(GripperPlant::from_params(name, params)),
        _ => panic!("Unknown node type: {}", node_type),
    }
}
