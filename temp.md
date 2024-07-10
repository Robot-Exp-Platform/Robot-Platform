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
// src/exp.rs
use serde::Deserialize;
use serde_json::from_reader;
use std::fs::File;
use std::sync::{Arc, RwLock};

use crate::config::CONFIG_PATH;
use controller::config::build_controller;
use planner::config::build_planner;
use robot::robots::{panda, robot_list::RobotList};

pub struct Exp {
    pub robot_exp: Arc<RwLock<dyn robot::Robot>>,
    pub controller_exp: Box<dyn controller::Controller>,
    pub planner_exp: Box<dyn planner::Planner>,
}

#[derive(Debug, Deserialize)]
struct Config {
    name: String,
    robot_type: String,
    controller: String,
    planner: String,
    robots: Option<Vec<Config>>,
}

impl Exp {
    pub fn new(
        robot_exp: Arc<RwLock<dyn robot::Robot>>,
        controller_exp: Box<dyn controller::Controller>,
        planner_exp: Box<dyn planner::Planner>,
    ) -> Exp {
        Exp {
            robot_exp,
            controller_exp,
            planner_exp,
        }
    }

    fn build_exp_tree(
        config: Config,
        path: String,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Box<dyn controller::Controller>,
        Box<dyn planner::Planner>,
    ) {
        match config.robot_type.as_str() {
            "RobotList" => {
                // 建立该节点的各个模块
                let robot_list = Arc::new(RwLock::new(RobotList::new(
                    config.name.clone(),
                    path.clone(),
                )));
                let mut controller = build_controller(
                    config.controller,
                    config.robot_type.clone(),
                    path.clone(),
                    robot_list.clone(),
                );
                let mut planner = build_planner(
                    config.planner,
                    config.robot_type.clone(),
                    path.clone(),
                    robot_list.clone(),
                );

                // 递归建立子节点
                for robot_config in config.robots.unwrap() {
                    let (robot, child_controller, child_planner) = Exp::build_exp_tree(
                        robot_config,
                        format!("{}{}/", path.clone(), config.name.clone()),
                    );
                    robot_list.write().unwrap().add_robot(robot);
                    controller.add_controller(child_controller);
                    planner.add_planner(child_planner);
                }
                (robot_list, controller, planner)
            }
            "panda" => {
                let robot = panda::Panda::new_with_name(config.name.clone(), path.clone());
                let robot = Arc::new(RwLock::new(robot));
                let controller = build_controller(
                    config.controller,
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                let planner = build_planner(
                    config.planner,
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                (robot, controller, planner)
            }
            _ => panic!("Unknown robot type: {}", config.robot_type.clone()),
        }
    }

    pub fn init() -> Exp {
        // 加载配置文件
        let config_file = File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let (robot, controller, planner) = Exp::build_exp_tree(config, "".to_string());
        Exp::new(robot, controller, planner)
    }
}
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
// crates/controller/src/controller_trait.rs
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

    fn add_controller(&mut self, controller: Box<dyn Controller>);

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
// crates/controller/src/config.rs
use std::sync::{Arc, RwLock};

use crate::controller_trait::Controller;
use crate::controllers::controller_list::ControllerList;
use crate::controllers::pid::Pid;
use robot::robot_trait::Robot;
use robot::robots::panda;

pub fn build_controller<R: Robot + 'static>(
    controller_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Controller> {
    match controller_type.as_str() {
        "pid" => match robot_type.as_str() {
            "panda" => Box::new(Pid::<R, { panda::PANDA_DOF + 1 }>::new_without_params(
                robot_type + "_pid",
                path,
                robot,
            )),
            _ => panic!("pid controller does not support this kind of robot"),
        },
        "controller_list" => Box::new(ControllerList::new(robot_type + "_controllers", path)),
        _ => panic!("Controller type not found"),
    }
}
```

```rust
// crates/controller/src/controllers/pid.rs
use nalgebra as na;
use std::sync::{Arc, RwLock};

use crate::controller_trait::Controller;
use recoder::recoder_trait::Recoder;
use robot::robot_trait::Robot;

pub struct Pid<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    state: PidState<N>,
    params: PidParams<N>,

    _rosnode: PidNode,
    robot: Arc<RwLock<R>>,
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

impl<R: Robot + 'static, const N: usize> Pid<R, N> {
    pub fn new(
        name: String,
        path: String,
        params: PidParams<N>,
        robot: Arc<RwLock<R>>,
    ) -> Pid<R, N> {
        Pid {
            name,
            path,

            state: PidState {
                target: na::SVector::from_element(0.0),
                error: na::SVector::from_element(0.0),
                integral: na::SVector::from_element(0.0),
                derivative: na::SVector::from_element(0.0),
            },
            params,

            _rosnode: PidNode {
                #[cfg(target_os = "unix")]
                sub_list: Vec::new(),
                #[cfg(target_os = "unix")]
                pub_list: Vec::new(),
            },
            robot,
        }
    }

    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Pid<R, N> {
        Pid::new(
            name,
            path,
            PidParams {
                kp: na::SMatrix::from_element(0.0),
                ki: na::SMatrix::from_element(0.0),
                kd: na::SMatrix::from_element(0.0),
            },
            robot,
        )
    }

    // fn set_kp(&mut self, kp: na::SMatrix<f64, N, N>) {
    //     self.params.kp = kp;
    // }
    // fn set_ki(&mut self, ki: na::SMatrix<f64, N, N>) {
    //     self.params.ki = ki;
    // }
    // fn set_kd(&mut self, kd: na::SMatrix<f64, N, N>) {
    //     self.params.kd = kd;
    // }
}
```

```rust
// crates/planner/src/planner_trait.rs
#[derive(Clone, Copy)]
pub enum PlannerState {
    Unknow,
    Uninit,
    Running,
}

pub trait Planner {
    fn get_planner_state(&self) -> PlannerState {
        PlannerState::Unknow
    }
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn get_params(&self) -> Vec<f64>;

    fn add_planner(&mut self, planner: Box<dyn Planner>);

    // TODO add plan function
}
```

```rust
// crates/planner/src/config.rs
use std::sync::{Arc, RwLock};

use crate::planner_trait::Planner;
use crate::planners::linear::Linear;
use robot::robot_trait::Robot;
use robot::robots::panda;

pub fn build_planner<R: Robot + 'static>(
    planner_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Box<dyn Planner> {
    match planner_type.as_str() {
        "linear" => match robot_type.as_str() {
            "panda" => Box::new(Linear::<R, { panda::PANDA_DOF + 1 }>::new_without_params(
                robot_type + "_linear",
                path,
                robot,
            )),
            _ => panic!("linear planner does not support this kind of robot"),
        },
        // "planner_list" => Box::new(PlannerList::new(robot_type + "_planners", path)),
        _ => panic!("Planner type not found"),
    }
}
```

```rust
use serde::{Deserialize, Serialize};
use std::sync::{Arc, RwLock};

use crate::planner_trait::Planner;
use recoder::recoder_trait::Recoder;
use robot::robot_trait::Robot;

#[derive(Serialize, Deserialize)]
pub struct LinearParams {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
    interpolation: i32,
}

pub struct LinearNode {
    #[cfg(target_os = "unix")]
    sub_list: Vec<String>,
    #[cfg(target_os = "unix")]
    pub_list: Vec<String>,
}

pub struct Linear<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    params: LinearParams,

    _rosnode: LinearNode,

    #[allow(dead_code)]
    robot: Arc<RwLock<R>>,
}

impl<R: Robot + 'static, const N: usize> Linear<R, N> {
    pub fn new(
        name: String,
        path: String,
        params: LinearParams,
        robot: Arc<RwLock<R>>,
    ) -> Linear<R, N> {
        Linear {
            name,
            path,

            params,

            _rosnode: LinearNode {
                #[cfg(target_os = "unix")]
                sub_list: Vec::new(),
                #[cfg(target_os = "unix")]
                pub_list: Vec::new(),
            },
            robot,
        }
    }

    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Linear<R, N> {
        Linear::new(name, path, LinearParams { interpolation: 0 }, robot)
    }
}

impl<R: Robot + 'static, const N: usize> Planner for Linear<R, N> {
    // fn get_planner_state(&self) -> PlannerState {
    //     self.state
    // }

    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_params(&self) -> Vec<f64> {
        vec![self.params.interpolation as f64]
    }

    fn add_planner(&mut self, _planner: Box<dyn Planner>) {}
}

impl<R: Robot + 'static, const N: usize> Recoder for Linear<R, N> {
    fn recoder() {
        // TODO Recoder for Linear
    }
}
```
