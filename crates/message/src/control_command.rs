use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub enum ControlCommand {
    Joint(Vec<f64>),
    JointWithPeriod(f64, Vec<f64>),
    JointVelocity(Vec<f64>, Vec<f64>),
    JointVelocityWithPeriod(f64, Vec<f64>, Vec<f64>),
    JointVelocityAcceleration(Vec<f64>, Vec<f64>, Vec<f64>),
    JointVelocityAccelerationWithPeriod(f64, Vec<f64>, Vec<f64>, Vec<f64>),
    Tau(Vec<f64>),
    TauWithPeriod(f64, Vec<f64>),
}

#[derive(Debug, Deserialize)]
pub struct JointWithPeriod {
    pub period: f64,
    pub joint: Vec<f64>,
}
