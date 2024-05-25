use crate::controllers::pid::{PidParams, PidState};

#[derive(Clone, Copy)]
pub enum ControllerState<const N: usize> {
    Unknow,
    ControllerList,
    PidState(PidState<N>),
}

pub enum ControllerParams<const N: usize> {
    ControllerList(Vec<ControllerParams<N>>),
    PidParams(PidParams<N>),
}

pub trait Controller {
    // fn get_contoller_state(&self) -> ControllerState<N> {
    //     ControllerState::Unknow
    // }
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    // fn set_params(&mut self, params: ControllerParams<N>);

    fn init(&self) {
        // 在这里进行话题的声明，
        // 新建发布者和接收者，并将他们放入list中去
    }
    fn starting(&self) {}
    fn update(&mut self, _: f64) {}
    fn stopping(&self) {}
    fn waiting(&self) {}
    fn aborting(&self) {}
    fn init_request(&self) {}
}
