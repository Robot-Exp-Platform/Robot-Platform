use serde::Deserialize;
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};

use message::{Target, TaskState};

type TaskId = usize;

/// 任务管理器，负责任务的调度和依赖关系
/// TODO 任务执行完成后会通过
#[derive(Default)]
#[allow(dead_code)]
pub struct TaskManager {
    /// 存储所有任务节点
    tasks: HashMap<TaskId, Task>,
    /// 邻接表：记录任务之间的依赖关系
    adj_list: HashMap<TaskId, HashSet<TaskId>>,
    /// 入度表：记录每个任务的入度
    in_degree: HashMap<TaskId, usize>,
    /// 开放任务列表，正在执行或者可执行的任务
    open_tasks: HashSet<TaskId>,

    /// 与线程管理器通信的接收器
    receiver: Option<Arc<Mutex<Receiver<TaskState>>>>,
}

#[derive(Deserialize, Default, Clone)]
pub struct Task {
    pub id: TaskId,
    pub rely: Vec<TaskId>,
    pub target: Vec<Target>,

    pub nodes: Vec<(String, Vec<String>, Vec<String>, Value)>,
    pub edges: Vec<(usize, usize)>,
}

impl TaskManager {
    /// 从 JSON 文件读取任务并构建 DAG
    pub fn from_json(receiver: Receiver<TaskState>, file_path: &str) -> Self {
        let file_content = std::fs::read_to_string(file_path).expect("Failed to read JSON file");
        let task_list: Vec<Task> =
            serde_json::from_str(&file_content).expect("Invalid JSON format");

        let mut task_manager = TaskManager {
            receiver: Some(Arc::new(Mutex::new(receiver))),
            ..Default::default()
        };
        for task in task_list {
            task_manager.add_task(task);
        }
        task_manager
    }

    /// 添加任务并处理其依赖关系
    pub fn add_task(&mut self, task: Task) {
        let task_id = task.id;

        // 将任务插入 DAG
        self.tasks.insert(task_id, task.clone());
        self.adj_list.entry(task_id).or_default();
        self.in_degree.entry(task_id).or_insert(0);

        // 添加依赖关系
        for &rely_id in &task.rely {
            self.adj_list.entry(rely_id).or_default().insert(task_id);
            *self.in_degree.entry(task_id).or_insert(0) += 1;
        }

        // 如果入度为 0，则加入 open_tasks 集合
        if self.in_degree[&task_id] == 0 {
            self.open_tasks.insert(task_id);
        }
    }

    /// 获取所有入度为 0 的任务
    pub fn get_open_tasks(&self) -> Vec<Task> {
        self.open_tasks
            .iter()
            .filter_map(|id| self.tasks.get(id).cloned())
            .collect()
    }

    /// 删除任务，并更新依赖的入度
    pub fn remove_task(&mut self, task_id: usize) {
        if let Some(neighbors) = self.adj_list.remove(&task_id) {
            for &neighbor in &neighbors {
                let entry = self.in_degree.get_mut(&neighbor).unwrap();
                *entry -= 1;
                if *entry == 0 {
                    self.open_tasks.insert(neighbor);
                }
            }
        }

        self.tasks.remove(&task_id);
        self.in_degree.remove(&task_id);
        self.open_tasks.remove(&task_id);
    }
}
