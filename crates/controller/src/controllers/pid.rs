use crate::controller_trait::{Controller, ControllerState};
use recoder::recoder_trait::Recoder;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct PidParams {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
    kp: f64,
    ki: f64,
    kd: f64,
}

pub struct Pid {
    state: ControllerState,
    params: PidParams,
}

impl Pid {
    pub fn new(params: PidParams) -> Pid {
        Pid {
            state: ControllerState::Uninit,
            params,
        }
    }
    pub fn init(params: PidParams) -> Box<dyn Controller> {
        Box::new(Pid::new(params))
    }
}

impl Controller for Pid {
    fn get_contoller_state(&self) -> ControllerState {
        self.state
    }

    fn get_params(&self) -> Vec<f64> {
        vec![self.params.kp, self.params.ki, self.params.kd]
    }
}

impl Recoder for Pid {
    fn recoder() {
        // TODO Recoder for Pid
    }
}
