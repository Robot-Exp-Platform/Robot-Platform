use nalgebra as na;
use std::f64::consts::PI;

use crate::robots::robot_n_dof::RobotNDof;
use crate::robots::robot_n_dof::RobotNDofParams;

pub const RESEARCH3_DOF: usize = 7;

pub type FrankaResearch3 = RobotNDof<RESEARCH3_DOF, { RESEARCH3_DOF + 1 }>;

impl FrankaResearch3 {
    pub fn new_research3(name: String, path: String) -> FrankaResearch3 {
        RobotNDof::new_from_params(
            name,
            path,
            RobotNDofParams {
                nlink: RESEARCH3_DOF,
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
