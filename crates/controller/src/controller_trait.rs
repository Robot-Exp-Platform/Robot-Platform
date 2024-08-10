use crossbeam::queue::SegQueue;
use serde_json::Value;
// use serde_yaml::Value;
use std::sync::{Arc, Mutex};

use message::control_command::ControlCommand;
use message::track::Track;
use task_manager::ros_thread::ROSThread;

pub trait Controller: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;

    fn set_params(&mut self, params: Value);
    fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>);
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    );

    fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
    fn get_controller(&self) -> &Vec<Arc<Mutex<dyn Controller>>> {
        unimplemented!()
    }
}

#[macro_export]
macro_rules! generate_controller_methods {
    () => {
        fn get_name(&self) -> String {
            self.name.clone()
        }
        fn get_path(&self) -> String {
            self.path.clone()
        }

        fn set_params(&mut self, params: Value) {
            self.params = from_value(params).unwrap();
        }
        fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>) {
            self.msgnode.track_queue = track_queue;
        }
        fn set_controller_command_queue(
            &mut self,
            controller_command_queue: Arc<SegQueue<ControlCommand>>,
        ) {
            self.msgnode.control_command_queue = controller_command_queue;
        }

        fn add_controller(&mut self, _: Arc<Mutex<dyn Controller>>) {}
    };
}
