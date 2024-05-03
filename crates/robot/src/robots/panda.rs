use crate::robot_trait::{Robot, RobotParams, RobotState};
use nalgebra as na;

struct PandaState<const N: usize> {
    q: na::SVector<f64, N>,
    q_dot: na::SVector<f64, N>,
}

impl<const N: usize> RobotState for PandaState<N> {}

struct PandaParams<const N: usize>
where
    [(); N + 1]:
{
    nlink: usize,
    q_up_bound: na::SVector<f64, N>,
    q_done_bound: na::SVector<f64, N>,
    q_dot_bound: na::SVector<f64, N>,
    denavit_hartenberg: na::SMatrix<f64, N + 1, 4>,
    // denavit_hartenberg: vec<na::SVector<f64, 4>>,
}

impl<const N: usize> PandaParams<N> {
    fn new() -> PandaParams<N> {
        PandaParams {
            nlink: N,
            q_up_bound: na::SVector::from_vec(vec![
                -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
            ]),
            q_done_bound: na::SVector::from_vec(vec![
                2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
            ]),
            q_dot_bound: na::SVector::from_vec(vec![
                2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100,
            ]),
        }
    }
}

impl<const N: usize> RobotParams for PandaParams<N> {}

struct Panda {
    name: String,
    state: PandaState<7>,
    params: PandaParams<7>,
}

impl Panda {
    fn new() -> Panda {
        Panda {
            name: "Panda".to_string(),
            state: PandaState {
                q: na::SVector::from_element(0.0),
                q_dot: na::SVector::from_element(0.0),
            },
        }
    }
}

impl Robot for Panda {
    fn get_name(&self) -> String {
        self.name.clone()
    }
}
