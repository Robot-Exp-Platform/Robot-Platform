use crossbeam::queue::SegQueue;
use serde_json::Value as JsonValue;
use std::sync::{Arc, Mutex};

use crate::controllers::pid::{PidParams, PidState};
use message::control_command::ControlCommand;
use message::track::Track;
use robot::robots::panda::PANDA_DOF;
use task_manager::ros_thread::ROSThread;

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

pub trait Controller: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn set_params(&mut self, params: JsonValue);
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>);
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    );

    fn add_controller(&mut self, controller: Arc<Mutex<dyn Controller>>);
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        unimplemented!()
    }
}
