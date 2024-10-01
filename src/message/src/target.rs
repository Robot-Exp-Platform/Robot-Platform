use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::Pose;

#[derive(Debug, Deserialize, Serialize, Clone)]
pub enum Target {
    Pose(Pose),
    Joint(na::DVector<f64>),
    JointList(na::DMatrix<f64>),
}
