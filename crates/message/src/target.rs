use serde::{Deserialize, Serialize};

use crate::state::Pose;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Target {
    Pose(Pose),
    Joint(Vec<f64>),
}
