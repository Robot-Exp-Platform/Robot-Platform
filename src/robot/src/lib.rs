mod config;
mod robot_trait;
mod robots;
mod utilities;

use std::sync::{Arc, RwLock};

pub use config::*;
pub use robot_trait::*;
pub use robots::*;

#[derive(Clone)]
pub enum RobotType {
    // 所有的动态串联机器人。管理起来毫无压力
    DSeriseRobot(Arc<RwLock<DSeriseRobot>>),

    // 特殊的静态机器人，可以被静态表示
    Panda(Arc<RwLock<SPanda>>),
}

impl RobotType {
    pub fn name(&self) -> String {
        match self {
            RobotType::DSeriseRobot(robot) => robot.read().unwrap().name(),
            RobotType::Panda(robot) => robot.read().unwrap().name(),
        }
    }
}
