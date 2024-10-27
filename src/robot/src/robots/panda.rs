use nalgebra as na;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use crate::{DSeriseRobot, SeriseRobotParams};
use message::{Capsule, NodeMessage};

use super::{SeriseRobot, SeriseRobotState};

pub const PANDA_DOF: usize = 7;

pub type Panda<V> = SeriseRobot<V>;
pub type DPanda = Panda<na::DVector<f64>>;
pub type SPanda = Panda<na::SVector<f64, PANDA_DOF>>;

impl DPanda {
    pub fn new_panda(name: String) -> DPanda {
        DSeriseRobot {
            name,
            state: SeriseRobotState::<na::DVector<f64>> {
                q: na::DVector::from_vec(vec![
                    0.0, -FRAC_PI_4, 0.0, -2.3562, 0.0, FRAC_PI_2, FRAC_PI_4,
                ]),
                q_dot: na::DVector::zeros(PANDA_DOF),
                q_ddot: na::DVector::zeros(PANDA_DOF),
                q_jerk: na::DVector::zeros(PANDA_DOF),
                base: na::Isometry3::identity(),
                control_message: NodeMessage::NoneNodeMessage,
            },
            params: SeriseRobotParams::<na::DVector<f64>> {
                nlink: PANDA_DOF,
                q_default: na::DVector::from_vec(vec![
                    0.0, -FRAC_PI_4, 0.0, -2.3562, 0.0, FRAC_PI_2, FRAC_PI_4,
                ]),
                q_min_bound: na::DVector::from_vec(vec![
                    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                ]),
                q_max_bound: na::DVector::from_vec(vec![
                    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
                ]),
                q_dot_bound: na::DVector::from_vec(vec![
                    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100,
                ]),
                q_ddot_bound: na::DVector::from_vec(vec![15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]),
                q_jerk_bound: na::DVector::from_vec(vec![
                    7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0,
                ]),
                tau_bound: na::DVector::from_vec(vec![87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]),
                tau_dot_bound: na::DVector::from_vec(vec![
                    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
                ]),
                #[rustfmt::skip]
                dh: na::DMatrix::from_vec(PANDA_DOF+1,4,
                    vec![
                    0.0,        0.333,  0.0,     0.0,
                    0.0,        0.0,    0.0,     -FRAC_PI_2,
                    0.0,        0.316,  0.0,     FRAC_PI_2,
                    0.0,        0.0,    0.0825,  FRAC_PI_2,
                    0.0,        0.384,  -0.0825, -FRAC_PI_2,
                    0.0,        0.0,    0.0,     FRAC_PI_2,
                    0.0,        0.0,    0.088,   FRAC_PI_2,
                    -FRAC_PI_4, 0.107,  0.0,     0.0,
                    ]),
                capsules: vec![
                    Capsule::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, -0.333, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, -0.316, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, -0.05, 0.0, 0.0, 0.055, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, -0.0384, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, -0.088, 0.0, 0.0, 0.0, 0.07]),
                    Capsule::from_vec(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.107, 0.07]),
                    Capsule::from_vec(vec![0.0, -0.05, 0.0, 0.0, 0.05, 0.0, 0.1]),
                ],
            },
        }
    }
}
