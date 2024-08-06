use serde::Deserialize;

use robot::robot_trait::Pose;

#[derive(Debug, Deserialize, Clone, Copy)]
pub enum Target {
    Pose(Pose),
}
