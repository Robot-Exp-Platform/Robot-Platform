use nalgebra::SVector;
use serde::Deserialize;

use crate::state::Pose;

#[derive(Debug, Deserialize, Clone)]
pub enum Track {
    Pose(Vec<f64>),
    Joint(Vec<f64>),
    Velocity(Vec<f64>),
    Acceleration(Vec<f64>),
    JointVel(Vec<f64>, Vec<f64>),
    JointVelAcc(Vec<f64>, Vec<f64>, Vec<f64>),
}

#[derive(Debug, Deserialize, Clone)]
pub enum TrackN<const N: usize> {
    PoseN(Pose),
    Joint(SVector<f64, N>),
    Velocity(SVector<f64, N>),
    Acceleration(SVector<f64, N>),
    JointVec(SVector<f64, N>, SVector<f64, N>),
    JointVelAcc(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
}
