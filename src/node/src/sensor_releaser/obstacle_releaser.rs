use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde_json::{from_value, Value};
use std::sync::{Arc, RwLock};
use tracing::info;

use crate::{Node, NodeBehavior, NodeState};
use generate_tools::*;
use message::{DNodeMessageQueue, NodeMessageQueue, Pose, Target};
use robot::RobotType;
use sensor::Sensor;
use serde::Deserialize;

pub struct ObstacleReleaser<V> {
    /// The name of the releaser.
    name: String,
    /// The state of the releaser.
    state: ObstacleReleaserState,
    /// The parameters of the releaser.
    params: ObstacleReleaserParams,
    /// The node of the releaser.
    node: ObstacleReleaserNode<V>,
    /// The sensor that the releaser is using.
    sensor: Option<Arc<RwLock<Sensor>>>,
}

pub type DObstacleReleaser = ObstacleReleaser<na::DVector<f64>>;

struct ObstacleReleaserState {
    is_end: bool,
    node_state: NodeState,
    pose_queue: Vec<(usize, SegQueue<Pose>)>,
}

#[derive(Deserialize)]
pub struct ObstacleReleaserParams {
    period: f64,
    interp: usize,
}

#[derive(Default)]
struct ObstacleReleaserNode<V> {
    input_queue: NodeMessageQueue<V>,
}

impl DObstacleReleaser {
    pub fn new(name: String) -> DObstacleReleaser {
        ObstacleReleaser::from_params(
            name,
            ObstacleReleaserParams {
                period: 0.0,
                interp: 0,
            },
        )
    }

    pub fn from_json(name: String, params: serde_json::Value) -> DObstacleReleaser {
        ObstacleReleaser::from_params(name, serde_json::from_value(params).unwrap())
    }

    pub fn from_params(name: String, params: ObstacleReleaserParams) -> DObstacleReleaser {
        DObstacleReleaser {
            name,
            state: ObstacleReleaserState {
                is_end: false,
                node_state: NodeState::default(),
                pose_queue: Vec::new(),
            },
            params,
            node: ObstacleReleaserNode::default(),
            sensor: None,
        }
    }
}

impl Node<na::DVector<f64>> for DObstacleReleaser {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node));

    fn set_output_queue(&mut self, _: Arc<SegQueue<message::NodeMessage<na::DVector<f64>>>>) {}

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, _: RobotType) {}

    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor = Some(sensor);
    }

    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DObstacleReleaser {
    fn update(&mut self) {
        let interp = self.params.interp;

        // Process target
        if let Some(Target::Transform(id_t, pose_s, pose_e)) = self.node.input_queue.pop() {
            let queue = {
                let pose_queue = &mut self.state.pose_queue;
                if let Some((_, queue)) = pose_queue.iter_mut().find(|(id, _)| *id == id_t) {
                    queue
                } else {
                    println!("Create new queue for id: {}", id_t);
                    pose_queue.push((id_t, SegQueue::new()));
                    &mut pose_queue.last_mut().unwrap().1
                }
            };

            for i in 0..interp {
                queue.push(pose_s.lerp_slerp(&pose_e, i as f64 / interp as f64));
            }
        }

        // Update all obstacles
        if let Some(sensor) = &self.sensor {
            let Sensor::ObstacleList(obstacle_list) = &mut *sensor.write().unwrap();
            for (id, queue) in &self.state.pose_queue {
                if let Some(pose) = queue.pop() {
                    obstacle_list.update_pose(*id, pose);
                }
            }
        }

        // 输出当前障碍物列表
        if let Some(sensor) = &self.sensor {
            let Sensor::ObstacleList(obstacle_list) = &*sensor.read().unwrap();
            for obstacle in &obstacle_list.obstacle {
                info!(
                    node = format!("{}:{}", self.name, obstacle.id()),
                    obstacle = ?obstacle.as_slice()
                );
            }
        }
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }

    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }

    fn state(&mut self) -> crate::NodeState {
        self.state.node_state
    }
}
