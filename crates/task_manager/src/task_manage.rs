use crossbeam::queue::SegQueue;
use std::sync::Arc;

use crate::task::Task;
use message::target::Target;
pub struct TaskManager {
    task: Task,
    pub target_node_list: Vec<TargetNode>,
}

pub struct TargetNode {
    planner_name: String,
    target_queue: Arc<SegQueue<Target>>,
}

impl TaskManager {
    pub fn new() -> TaskManager {
        TaskManager {
            task: Task {
                nodes: Vec::new(),
                robot_tasks: Vec::new(),
            },
            target_node_list: Vec::new(),
        }
    }

    pub fn get_queue_with_name(&self, name: String) -> Option<Arc<SegQueue<Target>>> {
        for target_node in &self.target_node_list {
            if target_node.planner_name == name {
                return Some(target_node.target_queue.clone());
            }
        }
        None
    }

    pub fn set_task(&mut self, task: Task) {
        self.task = task;
    }

    pub fn add_target_node(&mut self, planner_name: String, target_queue: Arc<SegQueue<Target>>) {
        self.target_node_list.push(TargetNode {
            planner_name,
            target_queue,
        });
    }
}

impl Default for TaskManager {
    fn default() -> Self {
        Self::new()
    }
}
