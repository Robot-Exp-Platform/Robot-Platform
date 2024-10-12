use core::task;
use crossbeam::thread;
use nalgebra as na;
use serde_json::from_reader;
use std::{
    fs,
    sync::{mpsc, Arc, RwLock},
};

use manager::{Config, TaskManager, ThreadManager};
use robot::Robot;
use sensor::Sensor;

#[derive(Default)]
pub struct Exp {
    pub thread_manager: ThreadManager,
    pub task_manager: TaskManager,

    pub robot_pool: Vec<Arc<RwLock<dyn Robot<na::DVector<f64>>>>>,
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
        let mut thread_manager = ThreadManager::new(sender);
        let mut task_manager = TaskManager::from_json(receiver, task);
    }

    pub fn init(&mut self) {}
    pub fn start(&mut self) {}
    pub fn update(&mut self) {}
    pub fn is_running(&mut self) -> bool {
        true
    }
}
