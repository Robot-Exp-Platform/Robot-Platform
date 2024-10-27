use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Instant;

use message::TaskState;

use node::NodeBehavior;

#[derive(Default)]
pub struct ThreadManager {
    threads: Vec<thread::JoinHandle<()>>,

    /// 与 taskmanager 通信的通道,用于报告任务完成情况
    /// 考虑之后将传递的消息改为枚举类型，或许更加有利于管理
    sender: Option<Sender<TaskState>>,
}

impl ThreadManager {
    /// 创建一个线程管理器
    pub fn new(sender: Sender<TaskState>) -> Self {
        ThreadManager {
            threads: Vec::new(),
            sender: Some(sender),
        }
    }

    // 最自由的线程管理方式，不受线程管理器的内部管理，只是存在一个无穷长声明的线程罢了
    pub fn add_closure<F>(&mut self, closure: F)
    where
        F: FnOnce() + Send + 'static,
    {
        let thread = thread::spawn(move || {
            closure();
        });
        self.threads.push(thread);
    }

    /// 为节点开辟线程，节点符合 node 规范，是可以被线程管理器管理的线程
    /// 此时的 node 是裸漏的，不具备任何多线程能力，扔进线程之后不可被外部访问
    /// 我们要组一辈子的线程啊
    pub fn add_node(&mut self, node: Box<dyn NodeBehavior>) {
        let name = node.node_name();
        let sender = self.sender.clone().unwrap();
        let thread = thread::Builder::new()
            .name(name.clone())
            .spawn(move || {
                let mut node = node; // 将 node 声明为可变的
                println!("{} 向您问好. {} says hello.", name, name);
                node.init();
                node.start();

                let period = node.period();

                while node.is_running() {
                    let start_time = Instant::now();

                    node.update();

                    let elapsed_time = start_time.elapsed();
                    if period > elapsed_time {
                        thread::sleep(period - elapsed_time);
                    }
                }
                node.finalize();
                if "planner" == node.node_type().as_str() {
                    sender.send(TaskState::PlanEnd(name)).unwrap();
                }
            })
            .unwrap();
        self.threads.push(thread);
    }

    pub fn add_mutex_node(&mut self, node: Arc<Mutex<dyn NodeBehavior>>) {
        let node_lock = node.lock().unwrap();
        let name = node_lock.node_name();
        drop(node_lock);

        let node = node.clone();
        let sender = self.sender.clone().unwrap();
        let thread = thread::Builder::new()
            .name(name.clone())
            .spawn(move || {
                let mut node = node.lock().unwrap();
                println!("{} 向您问好. {} says hello.", name, name);
                node.init();
                node.start();

                let period = node.period();

                while node.is_running() {
                    let start_time = Instant::now();

                    node.update();

                    let elapsed_time = start_time.elapsed();
                    if period > elapsed_time {
                        thread::sleep(period - elapsed_time);
                    }
                }
                node.finalize();
                if "planner" == node.node_type().as_str() {
                    sender.send(TaskState::PlanEnd(name)).unwrap();
                }
            })
            .unwrap();
        self.threads.push(thread);
    }

    pub fn add_rwlock_node(&mut self, node: Arc<RwLock<dyn NodeBehavior>>) {
        let node_lock = node.read().unwrap();
        let name = node_lock.node_name();
        drop(node_lock);

        let node = node.clone();
        let sender = self.sender.clone().unwrap();
        let thread = thread::Builder::new()
            .name(name.clone())
            .spawn(move || {
                let mut node = node.write().unwrap();
                println!("{} 向您问好. {} says hello.", name, name);
                node.init();
                node.start();

                let period = node.period();

                while node.is_running() {
                    let start_time = Instant::now();

                    node.update();

                    let elapsed_time = start_time.elapsed();
                    if period > elapsed_time {
                        thread::sleep(period - elapsed_time);
                    }
                }
                node.finalize();
                if "planner" == node.node_type().as_str() {
                    sender.send(TaskState::PlanEnd(name)).unwrap();
                }
            })
            .unwrap();
        self.threads.push(thread);
    }
}
