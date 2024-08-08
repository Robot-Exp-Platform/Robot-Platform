use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::sync::{Arc, Mutex, RwLock};
use std::time::Duration;

use crate::planner_trait::Planner;
use message::target::Target;
use message::track::Track;
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

pub struct Linear<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    _state: LinearState<N>,
    params: LinearParams,

    magnode: LinearNode,

    #[allow(dead_code)]
    robot: Arc<RwLock<R>>,
}

pub struct LinearState<const N: usize> {}

#[derive(Deserialize)]
pub struct LinearParams {
    period: f64,
    interpolation: i32,
}

pub struct LinearNode {
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<Track>>,
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

            _state: LinearState {},
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

    fn update(&mut self) {
        println!("{} updating!", self.name);

        // 更新 target
        let target = match self.magnode.target_queue.pop() {
            // 根据不同的 target 类型，执行不同的任务，也可以将不同的 Target 类型处理为相同的类型
            Some(Target::Joint(joint)) => joint,
            Some(Target::Pose(_pose)) => {
                unimplemented!("Linear planner does not support Pose target.");
            }
            None => {
                eprintln!("Failed to pop control command from queue.");
                return;
            }
        };
        let target = na::SVector::from_vec(target);
        println!("{} get target: {:?}", self.name, target);

        // 获取 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = na::SVector::from_vec(robot_read.get_q());

        // 执行插值逻辑，将当前位置到目标位置的插值点和目标位置塞入 track 队列
        let track_list = interpolation::<N>(&q, &target, self.params.interpolation);

        // 发送 track
        for track in track_list {
            let track = Track::Joint(track.as_slice().to_vec());
            self.magnode.track_queue.push(track);
        }
    }

    fn get_period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}

fn interpolation<const N: usize>(
    start: &na::SVector<f64, N>,
    end: &na::SVector<f64, N>,
    interpolation: i32,
) -> Vec<na::SVector<f64, N>> {
    let mut track_list = Vec::new();
    for i in 0..interpolation {
        let mut track = na::SVector::from_vec(vec![0.0; N]);
        for j in 0..N {
            track[j] = start[j] + (end[j] - start[j]) * (i as f64 / interpolation as f64);
        }
        track_list.push(track);
    }
    track_list
}
