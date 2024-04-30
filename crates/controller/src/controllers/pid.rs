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
    _params: PidParams,
}

impl Pid {
    pub fn new(_params: PidParams) -> Pid {
        Pid {
            state: ControllerState::Uninit,
            _params,
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
}

impl Recoder for Pid {
    fn recoder() {
        // TODO Recoder for Pid
    }
}
