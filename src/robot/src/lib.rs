mod config;
mod robot_trait;
mod robots;
mod utilities;

use std::sync::{Arc, RwLock};

pub use config::*;
pub use robot_trait::*;
pub use robots::*;

pub type RobotLock<R> = Option<Arc<RwLock<R>>>;

#[derive(Clone)]
pub enum RobotType {
    // 所有的动态串联机器人。管理起来毫无压力
    DSeriseRobot(Arc<RwLock<DSeriseRobot>>),

    // 特殊的静态机器人，可以被静态表示
    Panda(Arc<RwLock<SPanda>>),

    // 其他的辅助件
    FrankaGripper(Arc<RwLock<Gripper>>),
}

impl RobotType {
    pub fn name(&self) -> String {
        match self {
            RobotType::DSeriseRobot(robot) => robot.read().unwrap().name(),
            RobotType::Panda(robot) => robot.read().unwrap().name(),
            RobotType::FrankaGripper(gripper) => gripper.read().unwrap().name(),
        }
    }
}

pub trait DownCastRobot: Sized {
    fn downcast_robot(robot: RobotType, currect_robot: Self) -> Self;
}

impl DownCastRobot for () {
    fn downcast_robot(_: RobotType, _: Self) -> Self {}
}

impl DownCastRobot for RobotLock<DSeriseRobot> {
    fn downcast_robot(robot: RobotType, currect_robot: Self) -> Self {
        match robot {
            RobotType::DSeriseRobot(robot) => Some(robot),
            _ => currect_robot,
        }
    }
}

impl DownCastRobot for RobotLock<Gripper> {
    fn downcast_robot(robot: RobotType, currect_robot: Self) -> Self {
        match robot {
            RobotType::FrankaGripper(gripper) => Some(gripper),
            _ => currect_robot,
        }
    }
}

impl DownCastRobot for (RobotLock<DSeriseRobot>, RobotLock<Gripper>) {
    fn downcast_robot(robot: RobotType, currect_robot: Self) -> Self {
        match robot {
            RobotType::DSeriseRobot(robot1) => (Some(robot1.clone()), currect_robot.1.clone()),
            _ => currect_robot,
        }
    }
}
