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
use robot_macros_derive::*;
use task_manager::ros_thread::ROSThread;
use task_manager::state_collector::{NodeState, StateCollector};

#[allow(dead_code)]
pub struct Cfs<R: SeriesRobot<N> + 'static, const N: usize> {
    name: String,
    path: String,

    params: CfsParams,

    msgnode: CfsNode,

    robot: Arc<RwLock<R>>,
}

#[allow(dead_code)]
#[derive(Deserialize)]
pub struct CfsParams {
    period: f64,
    interpolation: usize,
    q1: na::DMatrix<f64>,
    q2: na::DMatrix<f64>,
    q3: na::DMatrix<f64>,
}

pub struct CfsNode {
    recoder: Option<BufWriter<fs::File>>,
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<Track>>,
    state_collector: StateCollector,
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Cfs<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Cfs<R, N> {
        Cfs::from_params(
            name,
            path,
            CfsParams {
                period: 0.0,
                interpolation: 0,
                q1: na::DMatrix::zeros(0, 0),
                q2: na::DMatrix::zeros(0, 0),
                q3: na::DMatrix::zeros(0, 0),
            },
            robot,
        )
    }
    pub fn from_params(
        name: String,
        path: String,
        params: CfsParams,
        robot: Arc<RwLock<R>>,
    ) -> Cfs<R, N> {
        Cfs {
            name,
            path,
            params,
            msgnode: CfsNode {
                recoder: None,
                target_queue: Arc::new(SegQueue::new()),
                track_queue: Arc::new(SegQueue::new()),
                state_collector: Arc::new((Mutex::new(NodeState::new()), Condvar::new())),
            },
            robot,
        }
    }
}

impl<R: SeriesRobot<N> + 'static, const N: usize> Planner for Cfs<R, N> {
    generate_planner_method!();
}

impl<R: SeriesRobot<N> + 'static, const N: usize> ROSThread for Cfs<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn start(&mut self) {
        // 进入启动状态，并通知所有线程
        let (state, cvar) = &*self.msgnode.state_collector;
        state.lock().unwrap().start();
        cvar.notify_all();

        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name()
            ))
            .unwrap();
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "./data/{}/{}/{}/cfs.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.msgnode.recoder = Some(BufWriter::new(file));
        }

        // 进入循环状态，并通知所有线程
        state.lock().unwrap().update();
        cvar.notify_all();
    }

    fn update(&mut self) {
        // 更新 target

        // 获取 robot 状态

        // 执行CFS逻辑

        // 记录 track

        // 发送 track
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
