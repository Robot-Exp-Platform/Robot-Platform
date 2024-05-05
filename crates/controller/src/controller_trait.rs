#[derive(Clone, Copy)]
pub enum ControllerState {
    Unknow,
    Uninit,
    Running,
}

pub trait Controller {
    fn get_contoller_state(&self) -> ControllerState {
        ControllerState::Unknow
    }

    // TODO add control function
}
