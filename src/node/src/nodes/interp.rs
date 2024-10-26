use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::{from_value, Value};
use std::fs;
use std::io::{BufWriter, Write};
use std::sync::{Arc, RwLock};
use std::time::Duration;

use crate::{utilities::lerp, Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue, DTrack, NodeMessageQueue};
use robot::{DSeriseRobot, Robot, RobotType};
use sensor::Sensor;

/// 通过插值实现的路径规划器，在不考虑逆运动学的情况下实现轨迹插值。
pub struct Interp<R, V> {
    /// The name of the planner.
    name: String,
    /// The state of the planner.
    _state: InterpState,
    /// The parameters of the planner.
    params: InterpParams,
    /// The node of the planner.
    node: InterpNode<V>,
    /// The robot that the planner is controlling.
    robot: Arc<RwLock<R>>,
}

pub type DInterp = Interp<DSeriseRobot, na::DVector<f64>>;
pub type SInterp<R, const N: usize> = Interp<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct InterpState {
    /// The current target of the planner.
    _target: Option<DNodeMessage>,
}

#[derive(Deserialize, Default)]
pub struct InterpParams {
    period: f64,
    interp_fn: String,
    ninter: usize,
}

#[derive(Default)]
pub struct InterpNode<V> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    input_queue: DNodeMessageQueue,
    output_queue: NodeMessageQueue<V>,
}

impl DInterp {
    pub fn new(name: String, robot: Arc<RwLock<DSeriseRobot>>) -> DInterp {
        DInterp::from_params(name, InterpParams::default(), robot)
    }
    pub fn from_json(name: String, robot: Arc<RwLock<DSeriseRobot>>, json: Value) -> DInterp {
        DInterp::from_params(name, from_value(json).unwrap(), robot)
    }
    pub fn from_params(
        name: String,
        params: InterpParams,
        robot: Arc<RwLock<DSeriseRobot>>,
    ) -> DInterp {
        DInterp {
            name,
            _state: InterpState::default(),
            params,
            node: InterpNode::default(),
            robot,
        }
    }
}

impl Node<na::DVector<f64>> for DInterp {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));

    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = robot;
        }
    }
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.node.sensor = Some(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DInterp {
    fn update(&mut self) {
        // 获取当前 robot 状态
        let robot_read = self.robot.read().unwrap();
        let q = robot_read.q();
        // TODO 需要在此处检查任务是否完成，如果未完成则无需从队列中取出新的目标，而是应当继续执行当前目标

        // 根据不同的Target生成插值轨迹，每两点之间的插值数量为 ninter
        let track_list = match self.node.input_queue.pop() {
            Some(DNodeMessage::Joint(joint)) => match self.params.interp_fn.as_str() {
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
            self.node.output_queue.push(DTrack::Joint(track));
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
