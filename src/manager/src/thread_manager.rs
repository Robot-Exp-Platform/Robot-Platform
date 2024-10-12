use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Instant;

use crate::Node;

#[derive(Default)]
#[allow(dead_code)]
pub struct ThreadManager {
    threads: Vec<thread::JoinHandle<()>>,
}

impl ThreadManager {
    pub fn add_node(&mut self, node: Arc<dyn Node>) {}
    pub fn add_mutex_node(&mut self, node: Arc<Mutex<dyn Node>>) {
        let node_lock = node.lock().unwrap();
        let name = node_lock.thread_name() + "_thread";
        drop(node_lock);

        let node = node.clone();
        let thread = thread::Builder::new()
            .name(name)
            .spawn(move || {
                let mut node = node.lock().unwrap();
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
            })
            .unwrap();
        self.threads.push(thread);
    }
    pub fn add_rwlock_node(&mut self, node: Arc<RwLock<dyn Node>>) {
        let node_lock = node.read().unwrap();
        let name = node_lock.thread_name() + "_thread";
        drop(node_lock);

        let node = node.clone();
        let thread = thread::Builder::new()
            .name(name)
            .spawn(move || {
                let mut node = node.write().unwrap();
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
            })
            .unwrap();
        self.threads.push(thread);
    }
}
