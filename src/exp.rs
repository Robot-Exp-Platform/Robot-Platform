use chrono::Local;
use serde_json::from_reader;
use std::{
    fs,
    sync::{mpsc, Arc, RwLock},
};

use manager::{Config, TaskManager, ThreadManager};
use robot::{self, DRobot};
use sensor::Sensor;

#[derive(Default)]
pub struct Exp {
    pub thread_manager: ThreadManager,
    pub task_manager: TaskManager,

    pub robot_pool: Vec<Arc<RwLock<dyn DRobot>>>,
    pub sensor_pool: Vec<Arc<RwLock<Sensor>>>,
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
        let thread_manager = ThreadManager::new(sender);
        let task_manager = TaskManager::from_json(receiver, task);
        let mut robot_pool = Vec::new();
        let mut sensor_pool = Vec::new();
        for robot in config.robots {
            robot_pool.push(robot::from_config(robot));
        }
        for sensor in config.sensors {
            sensor_pool.push(sensor::from_config(sensor));
        }

        Exp {
            thread_manager,
            task_manager,
            robot_pool,
            sensor_pool,
        }
    }

    // 实验初始化过程
    pub fn init(&mut self) {
        println!(
            "现在是 {}，先生，祝您早上、中午、晚上好",
            Local::now().format("%Y-%m-%d %H:%M:%S")
        );
    }

    // 实验进行过程，需要从任务管理器中取出位于开放列表中的任务并筹备对应的节点，然后交给线程管理器
    pub fn update(&mut self) {}
    pub fn is_running(&mut self) -> bool {
        true
    }
}
