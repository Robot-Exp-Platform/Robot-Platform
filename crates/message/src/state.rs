use nalgebra as na;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub enum RobotState {
    Pose(Pose),
    Joint(Vec<f64>),
    Velocity(Vec<f64>),
    Acceleration(Vec<f64>),
    JointVelocity(Vec<f64>, Vec<f64>),
    JointVelocityAcceleration(Vec<f64>, Vec<f64>, Vec<f64>),
}

pub type Pose = na::Isometry3<f64>;

pub struct JointVelocity {
    pub q: Vec<f64>,
    pub q_dot: Vec<f64>,
}

pub struct JointVelocityAcceleration {
    pub q: Vec<f64>,
    pub q_dot: Vec<f64>,
    pub q_ddot: Vec<f64>,
}

// type Joint<const N: usize> = na::SVector<f64, N>;
// type Velocity<const N: usize> = na::SVector<f64, N>;
// type Acceleration<const N: usize> = na::SVector<f64, N>;

// pub enum RobotState<const N: usize> {
//     Poes(Pose),
//     Joint(Joint<N>),
//     Velocity(Velocity<N>),
//     Acceleration(Acceleration<N>),
//     JointVelocity(JointVelocity<N>),
//     JointVelocityAcceleration(JointVelocityAcceleration<N>),
// }

// pub struct JointVelocity<const N: usize> {
//     pub q: Joint<N>,
//     pub q_dot: Velocity<N>,
// }

// pub struct JointVelocityAcceleration<const N: usize> {
//     pub q: Joint<N>,
//     pub q_dot: Velocity<N>,
//     pub q_ddot: Acceleration<N>,
// }
