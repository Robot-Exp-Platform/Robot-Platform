use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub enum ControlCommand {
    Joint(Vec<f64>),
    JointWithPeriod(f64, Vec<f64>),
    Tau(Vec<f64>),
    TauWithPeriod(f64, Vec<f64>),
}

#[derive(Debug, Deserialize)]
pub struct JointWithPeriod {
    pub period: f64,
    pub joint: Vec<f64>,
}
