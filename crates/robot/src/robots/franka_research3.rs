use nalgebra as na;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use crate::robots::robot_n_dof::RobotNDof;
use crate::robots::robot_n_dof::RobotNDofParams;
use message::collision_object::Capsule;

pub const RESEARCH3_DOF: usize = 7;

pub type FrankaResearch3 = RobotNDof<RESEARCH3_DOF, { RESEARCH3_DOF + 1 }>;

impl FrankaResearch3 {
    pub fn new_research3(name: String, path: String) -> FrankaResearch3 {
        RobotNDof::new_from_params(
            name,
            path,
            RobotNDofParams {
                nlink: RESEARCH3_DOF,
                q_default: na::SVector::from_vec(vec![
                    0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854,
                ]),
                q_min_bound: na::SVector::from_vec(vec![
                    -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159,
                ]),
                q_max_bound: na::SVector::from_vec(vec![
                    2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159,
                ]),
                q_dot_bound: na::SVector::from_vec(vec![2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]),
                q_ddot_bound: na::SVector::from_vec(vec![10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]),
                q_jerk_bound: na::SVector::from_vec(vec![
                    5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0,
                ]),
                tau_bound: na::SVector::from_vec(vec![87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]),
                tau_dot_bound: na::SVector::from_vec(vec![
                    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0,
                ]),
                #[rustfmt::skip]
                denavit_hartenberg: na::SMatrix::from_vec(vec![
                    0.0,        0.333,  0.0,     0.0,
                    0.0,        0.0,    0.0,     -FRAC_PI_2,
                    0.0,        0.316,  0.0,     FRAC_PI_2,
                    0.0,        0.0,    0.0825,  FRAC_PI_2,
                    0.0,        0.384,  -0.0825, -FRAC_PI_2,
                    0.0,        0.0,    0.0,     FRAC_PI_2,
                    0.0,        0.0,    0.088,   FRAC_PI_2,
                    -FRAC_PI_4, 0.107,  0.0,     0.0,]),
                capsules: [
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
        )
    }
}
