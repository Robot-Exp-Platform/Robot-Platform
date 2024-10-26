use chrono::Local;
use node::create_node;
use serde_json::from_reader;
use std::{
    fs,
    sync::{mpsc, Arc, RwLock},
};

use manager::{Config, PostOffice, Task, TaskManager, ThreadManager};
use node::NodeBehavior;
use robot::{self, RobotType};
use sensor::Sensor;

#[derive(Default)]
pub struct Exp {
    pub state: ExpState,

    pub thread_manager: ThreadManager,
    pub task_manager: TaskManager,
    pub post_office: PostOffice,

    pub robot_pool: Vec<RobotType>,
    pub sensor_pool: Vec<Arc<RwLock<Sensor>>>,
}

#[derive(Default, PartialEq)]
pub enum ExpState {
    #[default]
    Init,
    Running,
    TaskFinishCallback,
}

impl Exp {
    pub fn from_json(config: &str, task: &str) -> Self {
        // 加载配置文件
        let config_file = fs::File::open(config).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");
        #[cfg(feature = "recode")]
        {
            fs::create_dir(format!("./data/{}", *EXP_NAME)).unwrap();
            fs::copy(CONFIG_PATH, format!("./data/{}/config.json", *EXP_NAME)).unwrap();
        }
        // 根据配置开始初始化，关键在于搭建通讯
        let (sender, receiver) = mpsc::channel();

        // 创建线程管理器，线程管理器向任务管理器汇报任务完成情况,汇报内容为一个枚举类型
        let thread_manager = ThreadManager::new(sender);
        // 创建任务管理器，任务管理器接受线程管理器的汇报内容
        let task_manager = TaskManager::from_json(receiver, task);
        // 创建消息邮局，消息邮局负责保管传输信道以及负责统合或分发控制指令
        let post_office = PostOffice::default();
        // 创建实验状态机，实验状态机负责管理实验的整个过程
        let state = ExpState::Init;

        // 根据配置文件创建机器人池和传感器池，这些机器人和传感器将伴随整个实验过程，当有节点需要时从池中取出
        // TODO 设置使用状态，避免多个任务同时发布至同一个机器人
        let mut robot_pool = Vec::new();
        let mut sensor_pool = Vec::new();
        for robot_config in config.robots {
            let robot = robot::from_config(&robot_config);
            // 为 robot 配置仿真模式
            for _mode in robot_config.modes {
                // 创建仿真器节点

                // 创建与邮局的通讯
            }
            robot_pool.push(robot);
        }
        for sensor in config.sensors {
            sensor_pool.push(sensor::from_config(sensor));
        }

        Exp {
            state,
            thread_manager,
            task_manager,
            post_office,
            robot_pool,
            sensor_pool,
        }
    }

    /// 从机器人池中抓取机器人
    /// TODO 需要做机器人状态管理，避免多个任务
    pub fn get_robot_from_name(&self, name: &str) -> Option<RobotType> {
        for robot in &self.robot_pool {
            if robot.name() == name {
                return Some(robot.clone());
            }
        }
        None
    }

    pub fn get_sensor_from_name(&self, name: &str) -> Option<Arc<RwLock<Sensor>>> {
        for sensor in &self.sensor_pool {
            if sensor.read().unwrap().name() == name {
                return Some(sensor.clone());
            }
        }
        None
    }

    // /// 根据机器人类型创建对应的节点
    // pub fn create_nodes<R: DRobot + 'static>(&mut self, robot: Arc<RwLock<R>>, task: &Task) {
    //     let planner = create_planner(
    //         task.planner.0.as_str(),
    //         format!("task_{}", task.id),
    //         robot.clone(),
    //         task.planner.1.clone(),
    //     );

    //     let controller = create_controller(
    //         task.controller.0.as_str(),
    //         format!("task_{}", task.id),
    //         robot.clone(),
    //         task.controller.1.clone(),
    //     );

    //     self.thread_manager.add_node(planner);
    //     self.thread_manager.add_node(controller);
    // }
}

impl NodeBehavior for Exp {
    // 实验初始化过程
    fn init(&mut self) {
        println!(
            "现在是 {}，先生，祝您早上、中午、晚上好",
            Local::now().format("%Y-%m-%d %H:%M:%S")
        );
        self.state = ExpState::Running;
    }

    fn is_running(&mut self) -> bool {
        self.state == ExpState::Running
    }

    /// 实验进行过程，需要从任务管理器中取出位于开放列表中的任务并筹备对应的节点，然后交给线程管理器
    /// 这里实际上在执行任务管理器的逻辑分为以下几步
    /// 1. 在任务管理器中检查 `open_list` 是否为空，如果为空则结束任务。否则就对每个任务新建节点
    /// 2. 新建节点的过程中首先需要从机器人池子里面找到对应的机器人，然后针对任务描述新建规划器节点以及控制器节点
    ///    一般来说规划器节点直接对应任务，可以以规划器生命的结束作为任务的结束，而控制器更多的是针对机器人的控制
    fn update(&mut self) {
        let tasks: Vec<Task> = self
            .task_manager
            .get_open_tasks()
            .into_iter()
            .cloned()
            .collect();

        // 直接在循环中进行操作
        for task in tasks {
            if let Some(RobotType::DSeriseRobot(robot)) = self.get_robot_from_name(&task.robots[0])
            {
                // self.create_nodes(robot, &task); // task 使用引用即可
            }
        }
    }
}
