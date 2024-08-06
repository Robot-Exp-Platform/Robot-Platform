use serde::Deserialize;

use robot::robot_trait::Pose;

#[derive(Debug, Deserialize, Clone)]
pub enum Target {
    Pose(Pose),
    Joint(Vec<f64>),
}
