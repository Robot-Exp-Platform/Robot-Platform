use serde::Deserialize;
use std::sync::{Arc, RwLock};

use crate::{DPanda, DRobot};

#[derive(Debug, Deserialize)]
pub struct RobotConfig {
    pub name: String,
    pub robot_type: String,
    pub modes: Vec<String>,
}

/// 通过配置文件生成机器人实例
/// TODO 该函数目前只有动态版本，需要考虑如何实现对于其他储存类型的支持
pub fn from_config(robot_config: &RobotConfig) -> Arc<RwLock<dyn DRobot>> {
    match robot_config.robot_type.as_str() {
        "panda" => Arc::new(RwLock::new(DPanda::new_panda(robot_config.name.clone()))),
        _ => panic!("Unknown robot type: {}", robot_config.robot_type),
    }
}
