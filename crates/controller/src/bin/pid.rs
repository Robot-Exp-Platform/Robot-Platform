use controller::bin::control_trait::{Controller, ControllerState};

struct PID {
    state: ControllerState,
}

impl Controller for PID {
    fn get_contoller_state(&self) -> ControllerState {
        self.state
    }
}
