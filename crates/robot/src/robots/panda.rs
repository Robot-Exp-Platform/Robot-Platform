use crate::robot_trait::{Pose, Robot, RobotType};
use nalgebra as na;
use std::f64::consts::PI;

pub const PANDA_DOF: usize = 7;
const PANDA_DH_COL: usize = 4;

#[derive(Clone)]
pub struct Panda {
    name: String,
    path: String,

    state: PandaState,
    #[allow(dead_code)]
    params: PandaParams,
}

#[derive(Clone, Copy)]
pub struct PandaState {
    // 机器人状态,在运行状态下将会实时改变
    #[allow(dead_code)]
    q: na::SVector<f64, PANDA_DOF>,
    #[allow(dead_code)]
    q_dot: na::SVector<f64, PANDA_DOF>,
    base_pose: Pose,
}

#[derive(Clone, Copy)]
pub struct PandaParams {
    // 机器人参数,在运行状态下一般不会改变
    _nlink: usize,
    _q_up_bound: na::SVector<f64, PANDA_DOF>,
    _q_done_bound: na::SVector<f64, PANDA_DOF>,
    _q_dot_bound: na::SVector<f64, PANDA_DOF>,
    _q_ddot_bound: na::SVector<f64, PANDA_DOF>,
    _q_jerk_bound: na::SVector<f64, PANDA_DOF>,
    _denavit_hartenberg: na::SMatrix<f64, { PANDA_DOF + 1 }, PANDA_DH_COL>,
}

impl PandaParams {
    fn new() -> PandaParams {
        PandaParams {
            _nlink: PANDA_DOF,
            _q_up_bound: na::SVector::from_vec(vec![
                -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
            ]),
            _q_done_bound: na::SVector::from_vec(vec![
                2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
            ]),
            _q_dot_bound: na::SVector::from_vec(vec![
                2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100,
            ]),
            _q_ddot_bound: na::SVector::from_vec(vec![15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]),
            _q_jerk_bound: na::SVector::from_vec(vec![
                7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0,
            ]),
            #[rustfmt::skip]
            _denavit_hartenberg: na::SMatrix::from_vec(vec![
                0.0,      0.333,  0.0,      0.0,
                0.0,      0.0,    0.0,      -PI * 0.5,
                0.0,      0.316,  0.0,      PI * 0.5,
                0.0,      0.0,    0.0825,   PI * 0.5,
                0.0,      0.384,  -0.0825,  -PI * 0.5,
                0.0,      0.0,    0.0,      PI * 0.5,
                0.0,      0.0,    0.088,    PI * 0.5,
                -PI*0.25,  0.107,  0.0,      0.0,
            ]),
        }
    }
}

impl Panda {
    pub fn new(path: String) -> Panda {
        Panda::new_with_name("panda".to_string(), path)
    }

    pub fn new_with_name(name: String, path: String) -> Panda {
        Panda {
            path: path + &name,
            name,

            state: PandaState {
                q: na::SVector::from_element(0.0),
                q_dot: na::SVector::from_element(0.0),
                base_pose: na::SVector::from_element(0.0),
            },
            params: PandaParams::new(),
        }
    }
}

impl Robot for Panda {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_type(&self) -> RobotType {
        RobotType::PandaType
    }
    fn get_end_effector_pose(&self) -> Vec<Pose> {
        vec![self.state.base_pose]
    }

    fn set_name(&mut self, name: String) {
        self.name = name
    }
    fn set_path(&mut self, path: String) {
        self.path = path
    }

    fn reset_state(&mut self) {
        // TODO 位置重置
    }
}
