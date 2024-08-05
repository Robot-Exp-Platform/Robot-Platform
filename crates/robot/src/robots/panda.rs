use nalgebra as na;
use std::f64::consts::PI;

use crate::robots::robot_n_dof::RobotNDof;
use crate::robots::robot_n_dof::RobotNDofParams;

pub const PANDA_DOF: usize = 7;

pub type Panda = RobotNDof<PANDA_DOF, { PANDA_DOF + 1 }>;

impl Panda {
    pub fn new(name: String, path: String) -> Panda {
        RobotNDof::new_from_params(
            name,
            path,
            RobotNDofParams {
                nlink: PANDA_DOF,
                q_up_bound: na::SVector::from_vec(vec![
                    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                ]),
                q_down_bound: na::SVector::from_vec(vec![
                    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
                ]),
                q_dot_bound: na::SVector::from_vec(vec![
                    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100,
                ]),
                q_ddot_bound: na::SVector::from_vec(vec![15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]),
                q_jerk_bound: na::SVector::from_vec(vec![
                    7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0,
                ]),
                #[rustfmt::skip]
            denavit_hartenberg: na::SMatrix::from_vec(vec![
                0.0,      0.333,  0.0,      0.0,
                0.0,      0.0,    0.0,      -PI * 0.5,
                0.0,      0.316,  0.0,      PI * 0.5,
                0.0,      0.0,    0.0825,   PI * 0.5,
                0.0,      0.384,  -0.0825,  -PI * 0.5,
                0.0,      0.0,    0.0,      PI * 0.5,
                0.0,      0.0,    0.088,    PI * 0.5,
                -PI*0.25,  0.107,  0.0,     0.0,]),
            },
        )
    }
}
