use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::time::Duration;

use crate::planner_trait::Planner;
use message::target::Target;
use message::track::Track;
#[cfg(feature = "recode")]
use recoder::*;
use robot::robot_trait::SeriesRobot;
use task_manager::generate_node_method;
use task_manager::ros_thread::ROSThread;
use task_manager::state_collector::{NodeState, StateCollector};

pub struct Linear<R: SeriesRobot<N> + 'static, const N: usize> {
    name: String,
    path: String,

    params: LinearParams,

    msgnode: LinearNode,

    robot: Arc<RwLock<R>>,
}

#[derive(Deserialize)]
pub struct LinearParams {
    period: f64,
    interpolation: i32,
}

pub struct LinearNode {
    recoder: Option<BufWriter<fs::File>>,
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<Track>>,
    state_collector: StateCollector,
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Linear<R, N> {
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

            msgnode: LinearNode {
                recoder: Option::None,
                target_queue: Arc::new(SegQueue::new()),
                track_queue: Arc::new(SegQueue::new()),
                state_collector: Arc::new((Mutex::new(NodeState::new()), Condvar::new())),
            },
            robot,
        }
    }
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Planner for Linear<R, N> {
    generate_node_method!();

    fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>) {
        self.msgnode.target_queue = target_queue;
    }
    fn set_track_queue(&mut self, _track_queue: Arc<SegQueue<Track>>) {
        self.msgnode.track_queue = _track_queue;
    }
    fn set_state_collector(&mut self, _state_collector: StateCollector) {
        self.msgnode.state_collector = _state_collector;
    }

    fn add_planner(&mut self, _planner: Arc<Mutex<dyn Planner>>) {}
}

impl<R: SeriesRobot<N> + 'static, const N: usize> ROSThread for Linear<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn start(&mut self) {
        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name()
            ))
            .unwrap();
            let file = OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "./data/{}/{}/{}/linear.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.msgnode.recoder = Some(BufWriter::new(file));
        }
    }

    fn update(&mut self) {
        // 更新 target
        let target = match self.msgnode.target_queue.pop() {
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
        let q = robot_read.get_q_na();

        // 执行插值逻辑，将当前位置到目标位置的插值点和目标位置塞入 track 队列
        let track_list = interpolation::<N>(&q, &target, self.params.interpolation);

        // 记录 track
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.msgnode.recoder {
            for track in track_list.iter() {
                recode!(recoder, track);
            }
        }

        // 发送 track
        for track in track_list {
            let track = Track::Joint(track.as_slice().to_vec());
            self.msgnode.track_queue.push(track);
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.msgnode.recoder {
            recoder.flush().unwrap();
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
