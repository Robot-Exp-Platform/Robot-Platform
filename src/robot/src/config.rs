use message::Pose;
use serde::Deserialize;
use std::sync::{Arc, RwLock};

use crate::{DPanda, Gripper, RobotType};

#[derive(Debug, Deserialize)]
pub struct RobotConfig {
    pub name: String,
    pub robot_type: String,
    pub base_pose: Pose,
}

/// 通过配置文件生成机器人实例
/// TODO 该函数目前只有动态版本，需要考虑如何实现对于其他储存类型的支持
pub fn from_config(robot_config: &RobotConfig) -> RobotType {
    match robot_config.robot_type.as_str() {
        "panda" => RobotType::DSeriseRobot(Arc::new(RwLock::new(DPanda::new_panda(
            robot_config.name.clone(),
            robot_config.base_pose,
        )))),
        "franka_gripper" => {
            RobotType::FrankaGripper(Arc::new(RwLock::new(Gripper::new(&robot_config.name))))
        }
        _ => panic!("Unknown robot type: {}", robot_config.robot_type),
    }
}
