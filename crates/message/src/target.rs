use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};

use crate::state::Pose;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum Target {
    Pose(Pose),
    Joint(Vec<f64>),
    EndSpace(Pose, Vec<String>, Vec<Isometry3<f64>>),
}
