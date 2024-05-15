use std::f64::consts::PI;

use crate::robot_trait::{Robot, RobotParams, RobotState};
use nalgebra as na;

#[derive(Clone)]
pub struct Panda {
    name: String,
    state: PandaState<7>,
    params: PandaParams<7>,
}

#[derive(Clone)]
struct PandaState<const N: usize> {
    _q: na::SVector<f64, N>,
    _q_dot: na::SVector<f64, N>,
}

impl<const N: usize> RobotState for PandaState<N> {}

#[derive(Clone)]
struct PandaParams<const N: usize> {
    _nlink: usize,
    _q_up_bound: na::SVector<f64, N>,
    _q_done_bound: na::SVector<f64, N>,
    _q_dot_bound: na::SVector<f64, N>,
    _q_ddot_bound: na::SVector<f64, N>,
    _q_jerk_bound: na::SVector<f64, N>,
    _denavit_hartenberg: na::SMatrix<f64, 8, 4>,
}

impl<const N: usize> PandaParams<N> {
    fn new() -> PandaParams<N> {
        PandaParams {
            _nlink: N,
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
                -0.7854,  0.107,  0.0,      0.0,
            ]),
        }
    }
}

impl<const N: usize> RobotParams for PandaParams<N> {}

impl Panda {
    pub fn new() -> Panda {
        Panda {
            name: "Panda".to_string(),
            state: PandaState {
                _q: na::SVector::from_element(0.0),
                _q_dot: na::SVector::from_element(0.0),
            },
            params: PandaParams::new(),
        }
    }
}

impl Robot for Panda {
    fn get_name(&self) -> String {
        self.name.clone()
    }
}
