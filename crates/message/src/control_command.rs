use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub enum ControlCommand {
    Joint(Vec<f64>),
    JointWithPeriod(f64, Vec<f64>),
    JointVel(Vec<f64>, Vec<f64>),
    JointVelWithPeriod(f64, Vec<f64>, Vec<f64>),
    JointVelAcc(Vec<f64>, Vec<f64>, Vec<f64>),
    JointVelAccWithPeriod(f64, Vec<f64>, Vec<f64>, Vec<f64>),
    Tau(Vec<f64>),
    TauWithPeriod(f64, Vec<f64>),
}

#[derive(Debug, Deserialize)]
pub struct JointWithPeriod {
    pub period: f64,
    pub joint: Vec<f64>,
}
