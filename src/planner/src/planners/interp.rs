use crossbeam::queue::SegQueue;
use manager::Node;
use robot::DRobot;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{utilities::lerp, DPlanner, Planner};
use generate_tools::{get_fn, set_fn};
use message::{DTrack, Target};
use sensor::Sensor;

/// 通过插值实现的路径规划器，在不考虑逆运动学的情况下实现轨迹插值。
pub struct DInterp<R> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    _state: DinterpState,
    /// The parameters of the planner.
    params: DInterpParams,
    /// The node of the planner.
    node: DInterpNode,
    /// The robot that the planner is controlling.
    robot: Arc<RwLock<R>>,
}

#[derive(Default)]
pub struct DinterpState {
    /// The current target of the planner.
    _target: Option<Target>,
}

#[derive(Deserialize, Default)]
pub struct DInterpParams {
    period: f64,
    interp_fn: String,
    ninter: usize,
}

#[derive(Default)]
pub struct DInterpNode {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    target_queue: Arc<SegQueue<Target>>,
    track_queue: Arc<SegQueue<DTrack>>,
}

impl<R: DRobot> DInterp<R> {
    pub fn new(name: String, robot: Arc<RwLock<R>>) -> DInterp<R> {
        DInterp::from_params(name, DInterpParams::default(), robot)
    }
    pub fn from_params(name: String, params: DInterpParams, robot: Arc<RwLock<R>>) -> DInterp<R> {
        DInterp {
            name,
            _state: DinterpState::default(),
            params,
            node: DInterpNode::default(),
            robot,
        }
    }
}

impl<R: DRobot> DPlanner for DInterp<R> {
    set_fn!((set_track_queue, track_queue: Arc<SegQueue<DTrack>>, node));
}

impl<R: DRobot> Planner for DInterp<R> {
    get_fn!((name: String));
    set_fn!((set_target_queue, target_queue: Arc<SegQueue<Target>>, node));

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl<R: DRobot> Node for DInterp<R> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
    }

    fn update(&mut self) {
        // 获取当前 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.q();
        // TODO 需要在此处检查任务是否完成，如果未完成则无需从队列中取出新的目标，而是应当继续执行当前目标

        // 根据不同的Target生成插值轨迹，每两点之间的插值数量为 ninter
        let track_list = match self.node.target_queue.pop() {
            Some(Target::Joint(joint)) => match self.params.interp_fn.as_str() {
                "lerp" => lerp(&q, &vec![joint], self.params.ninter),
                _ => panic!("Unknown interp function."),
            },
            _ => return,
        };

        // 记录 track
        #[cfg(feature = "recode")]
        if let Some(ref mut recoder) = self.node.recoder {
            for track in track_list.iter() {
                recode!(recoder, track);
            }
        }

        // 将 track_list 中的轨迹放入 track_queue 中
        for track in track_list {
            self.node.track_queue.push(DTrack::Joint(track));
        }
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
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "./data/{}/{}/{}/linear.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }
}