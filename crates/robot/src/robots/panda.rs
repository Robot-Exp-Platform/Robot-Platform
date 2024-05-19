use crate::robot_trait::{Pose, Robot, RobotParams, RobotState, RobotType};
use nalgebra as na;
use std::f64::consts::PI;

#[derive(Clone)]
pub struct Panda {
    name: String,
    state: PandaState,
    params: PandaParams,
}

#[derive(Clone, Copy)]
pub struct PandaState {
    q: na::SVector<f64, 7>,
    q_dot: na::SVector<f64, 7>,
    base_pose: Pose,
}

#[derive(Clone, Copy)]
pub struct PandaParams {
    _nlink: usize,
    _q_up_bound: na::SVector<f64, 7>,
    _q_done_bound: na::SVector<f64, 7>,
    _q_dot_bound: na::SVector<f64, 7>,
    _q_ddot_bound: na::SVector<f64, 7>,
    _q_jerk_bound: na::SVector<f64, 7>,
    _denavit_hartenberg: na::SMatrix<f64, 8, 4>,
}

impl PandaParams {
    fn new() -> PandaParams {
        PandaParams {
            _nlink: 7,
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

impl Panda {
    pub fn new() -> Panda {
        Panda {
            name: "Panda".to_string(),
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
    fn get_type(&self) -> RobotType {
        RobotType::PandaType
    }
    fn get_state(&self) -> RobotState {
        RobotState::PandaState(self.state)
    }
    fn get_params(&self) -> RobotParams {
        RobotParams::PandaParams(self.params)
    }
    fn get_joint_positions(&self) -> na::DVector<f64> {
        na::DVector::from_column_slice(self.state.q.as_slice())
    }
    fn get_joint_velocities(&self) -> na::DVector<f64> {
        na::DVector::from_column_slice(self.state.q_dot.as_slice())
    }
    fn get_end_effector_pose(&self) -> Vec<Pose> {
        vec![self.state.base_pose]
    }

    // fn update_state(&mut self, new_state: RS) {
    //     self.state = new_state.into()
    // }
    // fn update_state(&mut self, new_state: Box<dyn Any>) -> Result<(), Box<dyn Any>> {
    //     self.state = *new_state.downcast::<PandaState>()?;
    //     Ok(())
    // }
    fn update_state(&mut self, new_state: RobotState) {
        if let RobotState::PandaState(panda_state) = new_state {
            self.state = panda_state;
        }
    }

    fn reset_state(&mut self) {
        // TODO 位置重置
    }
}
