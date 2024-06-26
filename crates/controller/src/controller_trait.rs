use crate::controllers::pid::{PidParams, PidState};
use robot::robots::panda::PANDA_DOF;

pub type PidParamsForPanda = PidParams<PANDA_DOF>;
pub type PidStateForPanda = PidState<PANDA_DOF>;

#[derive(Clone)]
pub enum ControllerState {
    Unknow,
    ControllerList,
    // PidState(PidState<N>),
    PidStateForPanda(Box<PidState<PANDA_DOF>>),
}

pub enum ControllerParams {
    ControllerList(Vec<ControllerParams>),
    // PidParams(PidParams<N>),
    PidParamsForPanda(Box<PidParamsForPanda>),
}

pub trait Controller {
    // fn get_contoller_state(&self) -> ControllerState<N> {
    //     ControllerState::Unknow
    // }
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    // fn set_params(&mut self, params: ControllerParams<N>);

    fn add_controller(&mut self, controller: Box<dyn Controller>);

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
