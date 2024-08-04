use crossbeam::queue::SegQueue;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex, RwLock};

use crate::planner_trait::Planner;
use massage::target::Target;
use massage::track::Track;
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

#[derive(Serialize, Deserialize)]
pub struct LinearParams {
    // TODO params should be vec of 64,which has deferent length for deferent Robot
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
    pub fn new(
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

    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Linear<R, N> {
        Linear::new(name, path, LinearParams { interpolation: 0 }, robot)
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

    fn set_params(&mut self, params: String) {
        let params: LinearParams = serde_json::from_str(&params).unwrap();
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
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
}
