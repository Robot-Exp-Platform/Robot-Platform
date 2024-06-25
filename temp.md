# temp

```rust
// Cargo.toml
[workspace]
members = [
    "crates/controller",
    "crates/iterator",
    "crates/planner",
    "crates/recoder",
    "crates/robot",
    "crates/simulator",
    "crates/solver",
    "crates/temp",
]

[package]
name = "robot_platform"
version = "0.1.0"
edition = "2021"
# edition = "2024"
build = "build.rs"

[dependencies]
robot = { path = "crates/robot" }
controller = { path = "crates/controller" }
planner = { path = "crates/planner" }
recoder = { path = "crates/recoder" }


rosrust = '*'
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
nalgebra = "*"
```

```rust
//crates/robot/src/robot_trait.rs
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

    fn set_name(&mut self, name: String);
    fn set_path(&mut self, path: String);

    fn update_state(&mut self, new_state: RobotState);
    fn reset_state(&mut self);
}

// pub trait RobotState {}
```

```rust
// crates/robot/src/robots/panda.rs
use crate::robot_trait::{Pose, Robot, RobotParams, RobotState, RobotType};
use nalgebra as na;
use std::f64::consts::PI;

pub const PANDA_DOF: usize = 7;
const PANDA_DH_ROW: usize = PANDA_DOF + 1;
const PANDA_DH_COL: usize = 4;

#[derive(Clone)]
pub struct Panda {
    name: String,
    path: String,

    state: PandaState,
    params: PandaParams,
}

#[derive(Clone, Copy)]
pub struct PandaState {
    q: na::SVector<f64, PANDA_DOF>,
    q_dot: na::SVector<f64, PANDA_DOF>,
    base_pose: Pose,
}

#[derive(Clone, Copy)]
pub struct PandaParams {
    _nlink: usize,
    _q_up_bound: na::SVector<f64, PANDA_DOF>,
    _q_done_bound: na::SVector<f64, PANDA_DOF>,
    _q_dot_bound: na::SVector<f64, PANDA_DOF>,
    _q_ddot_bound: na::SVector<f64, PANDA_DOF>,
    _q_jerk_bound: na::SVector<f64, PANDA_DOF>,
    _denavit_hartenberg: na::SMatrix<f64, PANDA_DH_ROW, PANDA_DH_COL>,
}
```

```rust
// crates/robot/src/robots/robot_list.rs
use crate::robot_trait::{Robot, RobotParams, RobotState, RobotType};
use nalgebra as na;

pub struct RobotList {
    name: String,
    path: String,

    pub robots: Vec<Box<dyn Robot>>,
}

macro_rules! apply_closure_to_iter {
    ($robots:expr, $closure:expr) => {
        $robots.iter().map($closure).collect::<Vec<_>>()
    };
}

impl RobotList {
    pub fn new(name: String, path: String) -> RobotList {
        RobotList::new_with_robots(name, path, Vec::new())
    }

    pub fn new_with_robots(name: String, path: String, robots: Vec<Box<dyn Robot>>) -> RobotList {
        RobotList { name, path, robots }
    }

    pub fn add_robot(&mut self, robot: Box<dyn Robot>) {
        self.robots.push(robot)
    }
}
```

```rust
// crates/robot/src/controller_trait.rs
use crate::controllers::pid::{PidParams, PidState};
use robot::robots::panda::PANDA_DOF;

pub type PidParamsForPanda = PidParams<PANDA_DOF>;
pub type PidStateForPanda = PidState<PANDA_DOF>;

#[derive(Clone)]
pub enum ControllerState {
    Unknow,
    ControllerList,
    // PidState(PidState<N>),
    PidStateForPanda(Box<PidState<PANDA_DOF>>),
}

pub enum ControllerParams {
    ControllerList(Vec<ControllerParams>),
    // PidParams(PidParams<N>),
    PidParamsForPanda(Box<PidParamsForPanda>),
}

pub trait Controller {
    // fn get_contoller_state(&self) -> ControllerState<N> {
    //     ControllerState::Unknow
    // }
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    // fn set_params(&mut self, params: ControllerParams<N>);

    fn init(&self) {
        // 在这里进行话题的声明，
        // 新建发布者和接收者，并将他们放入list中去
    }
    fn starting(&self) {}
    fn update(&mut self, _: f64) {}
    fn stopping(&self) {}
    fn waiting(&self) {}
    fn aborting(&self) {}
    fn init_request(&self) {}
}
```

```rust
// crates/robot/src/controllers/pid.rs
use crate::controller_trait::Controller;
use nalgebra as na;
use recoder::recoder_trait::Recoder;
use robot::robot_trait::Robot;

pub struct Pid<'a, R: Robot, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    _rosnode: PidNode,
    robot: &'a R,
}
#[derive(Clone, Copy)]
pub struct PidState<const N: usize> {
    target: na::SVector<f64, N>,
    error: na::SVector<f64, N>,
    integral: na::SVector<f64, N>,
    derivative: na::SVector<f64, N>,
}

pub struct PidParams<const N: usize> {
    kp: na::SMatrix<f64, N, N>,
    ki: na::SMatrix<f64, N, N>,
    kd: na::SMatrix<f64, N, N>,
}

pub struct PidNode {
    #[cfg(target_os = "unix")]
    sub_list: Vec<ros::Subscriber>,
    #[cfg(target_os = "unix")]
    pub_list: Vec<ros::Publisher>,
}
```
