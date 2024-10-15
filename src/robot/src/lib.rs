mod config;
mod robot_trait;
mod robots;
mod utilities;

pub use config::*;
pub use robot_trait::*;
pub use robots::*;

pub enum RobotType {
    // 所有的动态串联机器人。管理起来毫无压力
    DSeriseRobot(DSeriseRobot),

    // 特殊的静态机器人，可以被静态表示
    Panda(SPanda),
}

impl RobotType {
    pub fn name(&self) -> String {
        match self {
            RobotType::DSeriseRobot(robot) => robot.name(),
            RobotType::Panda(robot) => robot.name(),
        }
    }
}
