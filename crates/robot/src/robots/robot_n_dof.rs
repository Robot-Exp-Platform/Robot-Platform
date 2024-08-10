use nalgebra as na;

use crate::robot_trait::Robot;
use crate::robot_trait::SeriesRobot;
use message::state::Pose;

#[allow(dead_code)]
pub struct RobotNDof<const N: usize, const N_ADD_ONE: usize> {
    name: String,
    path: String,

    state: RobotNDofState<N>,
    params: RobotNDofParams<N, N_ADD_ONE>,
}

#[derive(Clone, Copy)]
pub struct RobotNDofState<const N: usize> {
    // 机器人状态,在运行状态下将会实时改变
    q: na::SVector<f64, N>,
    q_dot: na::SVector<f64, N>,
    base_pose: Pose,
}

#[derive(Clone, Copy)]
pub struct RobotNDofParams<const N: usize, const N_ADD_ONE: usize> {
    // 机器人参数,在运行状态下一般不会改变
    pub nlink: usize,
    pub q_min_bound: na::SVector<f64, N>,
    pub q_max_bound: na::SVector<f64, N>,
    pub q_dot_bound: na::SVector<f64, N>,
    pub q_ddot_bound: na::SVector<f64, N>,
    pub q_jerk_bound: na::SVector<f64, N>,
    pub tau_bound: na::SVector<f64, N>,
    pub tau_dot_bound: na::SVector<f64, N>,
    pub denavit_hartenberg: na::SMatrix<f64, N_ADD_ONE, 4>,
}

impl<const N: usize, const N_ADD_ONE: usize> RobotNDof<N, N_ADD_ONE> {
    pub fn new_from_params(
        name: String,
        path: String,
        params: RobotNDofParams<N, N_ADD_ONE>,
    ) -> RobotNDof<N, N_ADD_ONE> {
        RobotNDof {
            name,
            path,

            state: RobotNDofState::new(),
            params,
        }
    }

    pub fn new_without_params(name: String, path: String) -> RobotNDof<N, N_ADD_ONE> {
        RobotNDof::new_from_params(name, path, RobotNDofParams::new())
    }
}

impl<const N: usize, const N_ADD_ONE: usize> RobotNDofParams<N, N_ADD_ONE> {
    fn new() -> RobotNDofParams<N, N_ADD_ONE> {
        RobotNDofParams {
            nlink: N,
            q_min_bound: na::SVector::from_element(0.0),
            q_max_bound: na::SVector::from_element(0.0),
            q_dot_bound: na::SVector::from_element(0.0),
            q_ddot_bound: na::SVector::from_element(0.0),
            q_jerk_bound: na::SVector::from_element(0.0),
            tau_bound: na::SVector::from_element(0.0),
            tau_dot_bound: na::SVector::from_element(0.0),
            denavit_hartenberg: na::SMatrix::from_element(0.0),
        }
    }
}

impl<const N: usize> RobotNDofState<N> {
    fn new() -> RobotNDofState<N> {
        RobotNDofState {
            q: na::SVector::from_element(0.0),
            q_dot: na::SVector::from_element(0.0),
            base_pose: Pose::identity(),
        }
    }

    pub fn get_q(&self) -> &na::SVector<f64, N> {
        &self.q
    }

    pub fn get_q_dot(&self) -> &na::SVector<f64, N> {
        &self.q_dot
    }
}

impl<const N: usize, const N_ADD_ONE: usize> SeriesRobot<N> for RobotNDof<N, N_ADD_ONE> {
    fn get_q_na(&self) -> na::SVector<f64, N> {
        self.state.q
    }

    fn get_q_dot_na(&self) -> na::SVector<f64, N> {
        self.state.q_dot
    }
}

impl<const N: usize, const N_ADD_ONE: usize> Robot for RobotNDof<N, N_ADD_ONE> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_q(&self) -> Vec<f64> {
        self.state.q.iter().copied().collect()
    }
    fn get_q_dot(&self) -> Vec<f64> {
        self.state.q_dot.iter().copied().collect()
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
    fn set_q(&mut self, q: Vec<f64>) {
        self.state.q = na::SVector::from_vec(q)
    }
    fn set_q_dot(&mut self, q_dot: Vec<f64>) {
        self.state.q_dot = na::SVector::from_vec(q_dot)
    }

    fn reset_state(&mut self) {
        // TODO 位置重置
    }
}
