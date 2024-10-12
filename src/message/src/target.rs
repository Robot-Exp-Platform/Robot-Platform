use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::Pose;

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub enum Target {
    #[default]
    NoneTarget,
    Pose(Pose),
    Joint(na::DVector<f64>),
    JointList(na::DMatrix<f64>),
}
