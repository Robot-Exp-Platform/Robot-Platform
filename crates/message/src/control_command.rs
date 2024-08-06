use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub enum ControlCommand {
    Joint(Vec<f64>),
    JointWithPeriod(JointWithPeriod),
}

#[derive(Debug, Deserialize)]
pub struct JointWithPeriod {
    pub period: f64,
    pub joint: Vec<f64>,
}
