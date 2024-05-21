use crate::controllers::pid::PidState;

#[derive(Clone, Copy)]
pub enum ControllerState<const N: usize> {
    Unknow,
    ControllerList,
    PidState(PidState<N>),
}

pub trait Controller<const N: usize> {
    fn get_contoller_state(&self) -> ControllerState<N> {
        ControllerState::Unknow
    }

    // TODO add control function
}
