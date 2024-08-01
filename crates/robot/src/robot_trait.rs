use nalgebra as na;

use crate::robots::panda::{PandaParams, PandaState};

// TODO 我没想好这里的type是该作为一个单纯的判别依据比较好还是直接作为Robot的枚举比较好，如果直接作为符合同一个特征的枚举类型有些许的浪费，但是会带来之后在编写过程中的便捷……
pub enum RobotType {
    RobotListType(Vec<RobotType>),
    PandaType,
}

pub enum RobotState {
    RobotListState(Vec<RobotState>),
    PandaState(Box<PandaState>),
}
pub enum RobotParams {
    RobotListParams(Vec<RobotParams>),
    PandaParams(Box<PandaParams>),
}
pub type Pose = na::SVector<f64, 6>;

pub trait Robot: Send + Sync {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_type(&self) -> RobotType;

    fn get_end_effector_pose(&self) -> Vec<Pose>;

    fn set_name(&mut self, name: String);
    fn set_path(&mut self, path: String);
    fn set_q(&mut self, q: Vec<f64>);
    fn set_q_dot(&mut self, q_dot: Vec<f64>);

    fn reset_state(&mut self);
}

// pub trait RobotState {}
