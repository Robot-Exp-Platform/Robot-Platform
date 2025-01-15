use crossbeam::queue::SegQueue;
use kernel_macro::node_registration;
use nalgebra as na;
use tracing::info;

use crate::{Node, NodeBehavior, NodeExtBehavior, NodeRegister};
use message::{Pose, Target};
use sensor::Sensor;
use serde::Deserialize;

pub type ObstacleReleaser<V> = Node<ObstacleReleaserState, ObstacleReleaserParams, (), V>;
#[node_registration("obstacle_releaser")]
pub type DObstacleReleaser = ObstacleReleaser<na::DVector<f64>>;

#[derive(Default)]
pub struct ObstacleReleaserState {
    pose_queue: Vec<(usize, SegQueue<Pose>)>,
}

#[derive(Deserialize)]
pub struct ObstacleReleaserParams {
    period: f64,
    interp: usize,
}

impl NodeBehavior for DObstacleReleaser {
    fn update(&mut self) {
        let interp = self.params.interp;

        // Process target
        if let Some(Target::Transform(id_t, pose_s, pose_e)) = self.input_queue.pop() {
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
        self.node_state
    }
}
