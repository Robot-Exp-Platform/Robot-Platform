# temp

本 rust 项目主要用于多机器人的仿真、规划与控制。

每个实验被声明为一个结构体，

```rust
pub struct Exp {
    pub thread_manager: ThreadManager,
    pub task_manager: TaskManager,
    pub post_office: PostOffice,

    pub robot_pool: Vec<Arc<RwLock<dyn DRobot>>>,
    pub sensor_pool: Vec<Arc<RwLock<Sensor>>>,
}
```

其中 `DRobot` 保存了一个机器人当前的所有状态以及参数信息，`Sensor` 是一个枚举类型，包裹了不同的传感器。机器人和传感器会被多线程同时调用，故采用了 `Arc<RwLock<T>>` 的方式进行共享。

`ThreadManager` 负责管理所有线程，其结构如下

```rust
    #[derive(Default)]
pub struct ThreadManager {
    threads: Vec<thread::JoinHandle<()>>,

    /// 与 taskmanager 通信的通道,用于报告任务完成情况
    /// 考虑之后将传递的消息改为枚举类型，或许更加有利于管理
    sender: Option<Sender<TaskState>>,
}
impl ThreadManager {
    pub fn new(sender: Sender<TaskState>) -> Self ;

    pub fn add_closure<F>(&mut self, closure: F)
    where
        F: FnOnce() + Send + 'static,

    pub fn add_node(&mut self, node: Box<dyn Node>) ;

    pub fn add_mutex_node(&mut self, node: Arc<Mutex<dyn Node>>) ;

    pub fn add_rwlock_node(&mut self, node: Arc<RwLock<dyn Node>>) ;
}
```

其中 `Node` 是一个 trait，定义了一个节点的基本行为，需要注意，在 ThreadManager 中的 Node 与 TaskManager 中的 Node 是不同的，前者是一个 trait，后者是一个结构体。

`TaskManager` 负责管理所有任务，所有的任务被以一个有向无环图管理，每个节点是一个任务，每个边是一个依赖关系。`TaskManager` 会根据任务的依赖关系，将任务分配给不同的线程执行。

```rust
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

    pub robots: Vec<String>,
    pub nodes: Vec<Node>,
    pub target: Target,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Node {
    pub node_type: String,
    pub name: String,

    pub sensor: Option<String>,
    pub param: Value,
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
    pub fn get_open_tasks(&self) -> Vec<&Task> {
        self.open_tasks
            .iter()
            .filter_map(|id| self.tasks.get(id))
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
```
