use crate::planner_trait::{Planner, PlannerState};
use recoder::recoder_trait::Recoder;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct LinearParams {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
    interpolation: i32,
}

pub struct Linear {
    state: PlannerState,
    params: LinearParams,
}

impl Linear {
    pub fn new(params: LinearParams) -> Linear {
        Linear {
            state: PlannerState::Uninit,
            params,
        }
    }
    pub fn init(params: LinearParams) -> Box<dyn Planner> {
        Box::new(Linear::new(params))
    }
}

impl Planner for Linear {
    fn get_planner_state(&self) -> PlannerState {
        self.state
    }

    fn get_params(&self) -> Vec<f64> {
        vec![self.params.interpolation as f64]
    }
}

impl Recoder for Linear {
    fn recoder() {
        // TODO Recoder for Linear
    }
}
