use nalgebra as na;
use serde::Deserialize;

pub type Pose = na::Isometry3<f64>;

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
pub enum SRobotState<const N: usize> {
    Pose(Pose),
    Joint(na::SVector<f64, N>),
    Velocity(na::SVector<f64, N>),
    Acceleration(na::SVector<f64, N>),
    JointVel(na::SVector<f64, N>, na::SVector<f64, N>),
    JointVelAcc(
        na::SVector<f64, N>,
        na::SVector<f64, N>,
        na::SVector<f64, N>,
    ),
}

#[derive(Debug, Deserialize)]
pub enum DRobotState {
    Pose(Pose),
    Joint(na::DVector<f64>),
    Velocity(na::DVector<f64>),
    Acceleration(na::DVector<f64>),
    JointVel(na::DVector<f64>, na::DVector<f64>),
    JointVelAcc(na::DVector<f64>, na::DVector<f64>, na::DVector<f64>),
}

#[derive(Debug, Deserialize)]
pub enum TaskState {
    RelyRelease(String),
    PlanEnd(String),
    ControlEnd(String),
}
