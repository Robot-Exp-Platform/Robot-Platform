pub mod config;
pub mod robot_trait;
pub mod robots;
pub mod utilities;

pub use robot_trait::{Robot, SeriesRobot};
pub use robots::{
    franka_emika::{FrankaEmika, EMIKA_DOF},
    franka_research3::{FrankaResearch3, RESEARCH3_DOF},
    panda::{Panda, PANDA_DOF},
    robot_list::RobotList,
    robot_n_dof::RobotNDof,
};
