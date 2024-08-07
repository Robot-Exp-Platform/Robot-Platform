use serde::{Deserialize, Serialize};

use robot::robot_trait::Pose;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Target {
    Pose(Pose),
    Joint(Vec<f64>),
}
