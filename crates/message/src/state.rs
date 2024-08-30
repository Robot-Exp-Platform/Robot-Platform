use nalgebra::{Isometry3, SVector};
use serde::Deserialize;

#[derive(Debug)]
pub enum NodeState {
    Starting,
    Updating,
    Finished,
}

pub type Pose = Isometry3<f64>;

#[derive(Debug, Deserialize)]
pub enum RobotState {
    Pose(Pose),
    Joint(Vec<f64>),
    Velocity(Vec<f64>),
    Acceleration(Vec<f64>),
    JointVel(Vec<f64>, Vec<f64>),
    JointVelAcc(Vec<f64>, Vec<f64>, Vec<f64>),
}

#[derive(Debug, Deserialize)]
pub enum RobotStateN<const N: usize> {
    PoseN(Pose),
    Joint(SVector<f64, N>),
    Velocity(SVector<f64, N>),
    Acceleration(SVector<f64, N>),
    JointVec(SVector<f64, N>, SVector<f64, N>),
    JointVelAcc(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
}
