use kernel_macro::node_registration;
use nalgebra as na;
use serde::Deserialize;
use std::time::Duration;
use tracing::info;

use crate::{utilities::lerp, Node, NodeBehavior, NodeExtBehavior, NodeRegister};
use message::{DNodeMessage, NodeMessage};
use robot::{DSeriseRobot, Robot, RobotLock};

pub type Interp<R, V> = Node<InterpState<V>, InterpParams, RobotLock<R>, V>;

#[node_registration("interp")]
pub type DInterp = Interp<DSeriseRobot, na::DVector<f64>>;
pub type SInterp<R, const N: usize> = Interp<R, na::SVector<f64, N>>;

#[derive(Default)]
pub struct InterpState<V> {
    /// The current target of the planner.
    target: Option<NodeMessage<V>>,
}

#[derive(Deserialize)]
pub struct InterpParams {
    period: f64,
    interp_fn: String,
    ninter: usize,
}

impl NodeBehavior for DInterp {
    fn update(&mut self) {
        // 获取当前 robot 状态
        let robot_read = self.robot.as_ref().unwrap().read().unwrap();
        let q = robot_read.q();
        // TODO 需要在此处检查任务是否完成，如果未完成则无需从队列中取出新的目标，而是应当继续执行当前目标

        let interp_fn = match self.params.interp_fn.as_str() {
            "lerp" => lerp,
            _ => return,
        };

        // 根据不同的Target生成插值轨迹，每两点之间的插值数量为 ninter
        if let Some(target) = self.input_queue.pop() {
            self.state.target = Some(target);
        } else {
            return;
        }

        let target = self.state.target.clone().unwrap();
        info!(node = self.node_name().as_str(), input = ?target.as_slice());

        let track_list = match target {
            DNodeMessage::Joint(joint) => {
                interp_fn(&(0.3 * &q + 0.7 * &joint), &vec![joint], self.params.ninter)
            }
            DNodeMessage::JointList(joint_list) => interp_fn(&q, &joint_list, self.params.ninter),
            _ => return,
        };
        // 将 track_list 中的轨迹放入 track_queue 中
        while self.output_queue.pop().is_some() {}
        for track in track_list {
            let control_message = DNodeMessage::Joint(track);

            self.output_queue.push(control_message);
        }
    }

    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
