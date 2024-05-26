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

pub trait Robot {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_type(&self) -> RobotType;
    fn get_state(&self) -> RobotState;
    fn get_params(&self) -> RobotParams;

    fn get_joint_positions(&self) -> na::DVector<f64>;
    fn get_joint_velocities(&self) -> na::DVector<f64>;
    fn get_end_effector_pose(&self) -> Vec<Pose>;

    fn update_state(&mut self, new_state: RobotState);
    fn reset_state(&mut self);
}

// pub trait RobotState {}
