use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub enum Track {
    Pose(Vec<f64>),
    Joint(Vec<f64>),
    Velocity(Vec<f64>),
    Acceleration(Vec<f64>),
    JointVelocity(Vec<f64>, Vec<f64>),
    JointVelocityAcceleration(Vec<f64>, Vec<f64>, Vec<f64>),
}
