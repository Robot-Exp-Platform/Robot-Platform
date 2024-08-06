use crossbeam::queue::SegQueue;
use serde::{Deserialize, Serialize};
// use serde_json::{Value, from_value};
use serde_yaml::{from_value, Value};
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::planner_trait::Planner;
use message::target::Target;
use message::track::Track;
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

#[derive(Serialize, Deserialize)]
pub struct LinearParams {
    period: f64,
    interpolation: i32,
}

pub struct LinearNode {
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<Track>>,
}

pub struct Linear<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    params: LinearParams,

    magnode: LinearNode,

    #[allow(dead_code)]
    robot: Arc<RwLock<R>>,
}

impl<R: Robot + 'static, const N: usize> Linear<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Linear<R, N> {
        Linear::from_params(
            name,
            path,
            LinearParams {
                period: 0.0,
                interpolation: 0,
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        path: String,
        params: LinearParams,
        robot: Arc<RwLock<R>>,
    ) -> Linear<R, N> {
        Linear {
            name,
            path,

            params,

            magnode: LinearNode {
                target_queue: Arc::new(SegQueue::new()),
                track_queue: Arc::new(SegQueue::new()),
            },
            robot,
        }
    }
}

impl<R: Robot + 'static, const N: usize> Planner for Linear<R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_params(&self) -> Vec<f64> {
        vec![self.params.interpolation as f64]
    }

    fn set_params(&mut self, params: Value) {
        let params: LinearParams = from_value(params).unwrap();
        self.params = params;
    }
    fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>) {
        self.magnode.target_queue = target_queue;
    }
    fn set_track_queue(&mut self, _track_queue: Arc<SegQueue<Track>>) {
        self.magnode.track_queue = _track_queue;
    }

    fn add_planner(&mut self, _planner: Arc<Mutex<dyn Planner>>) {}
}

impl<R: Robot + 'static, const N: usize> ROSThread for Linear<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}
