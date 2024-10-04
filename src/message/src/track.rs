use nalgebra::{DVector, SVector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize, Clone)]
pub enum Track {
    Pose(Vec<f64>),
    Joint(Vec<f64>),
    Velocity(Vec<f64>),
    Acceleration(Vec<f64>),
    JointVel(Vec<f64>, Vec<f64>),
    JointVelAcc(Vec<f64>, Vec<f64>, Vec<f64>),
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub enum STrack<const N: usize> {
    Pose(SVector<f64, N>),
    Joint(SVector<f64, N>),
    Velocity(SVector<f64, N>),
    Acceleration(SVector<f64, N>),
    JointVel(SVector<f64, N>, SVector<f64, N>),
    JointVelAcc(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub enum DTrack {
    NoneTrack,
    Pose(DVector<f64>),
    Joint(DVector<f64>),
    Velocity(DVector<f64>),
    Acceleration(DVector<f64>),
    JointVel(DVector<f64>, DVector<f64>),
    JointVelAcc(DVector<f64>, DVector<f64>, DVector<f64>),
}

impl Default for DTrack {
    fn default() -> Self {
        DTrack::NoneTrack
    }
}
