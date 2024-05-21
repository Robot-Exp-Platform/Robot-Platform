use crate::controller_trait::{Controller, ControllerState};
use nalgebra as na;
use recoder::recoder_trait::Recoder;

pub struct Pid<const N: usize> {
    state: PidState<N>,
    _params: PidParams<N>,
}

#[derive(Clone, Copy)]
pub struct PidState<const N: usize> {
    error: na::SVector<f64, N>,
    integral: na::SVector<f64, N>,
    derivative: na::SVector<f64, N>,
}

pub struct PidParams<const N: usize> {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
    kp: na::SMatrix<f64, N, N>,
    ki: na::SMatrix<f64, N, N>,
    kd: na::SMatrix<f64, N, N>,
}

impl<const N: usize> Pid<N> {
    pub fn new(_params: PidParams<N>) -> Pid<N> {
        Pid {
            state: PidState {
                error: na::SVector::from_element(0.0),
                integral: na::SVector::from_element(0.0),
                derivative: na::SVector::from_element(0.0),
            },
            _params,
        }
    }
    pub fn init(params: PidParams<N>) -> Box<dyn Controller<N>> {
        Box::new(Pid::<N>::new(params))
    }
}

impl<const N: usize> Controller<N> for Pid<N> {
    fn get_contoller_state(&self) -> ControllerState<N> {
        ControllerState::PidState(self.state)
    }
}

impl<const N: usize> Recoder for Pid<N> {
    fn recoder() {
        // TODO Recoder for Pid
    }
}
