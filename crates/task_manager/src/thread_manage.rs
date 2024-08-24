use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread;
use std::time::Instant;

use crate::ros_thread::ROSThread;
pub struct ThreadManage {
    threads: Vec<thread::JoinHandle<()>>,
    pub condvar: Arc<(AtomicBool, Condvar, Mutex<()>)>,
}

impl ThreadManage {
    pub fn new() -> Self {
        let condvar = Arc::new((AtomicBool::new(false), Condvar::new(), Mutex::new(())));
        ThreadManage {
            threads: Vec::new(),
            condvar,
        }
    }

    pub fn add_thread(&mut self, node: Arc<Mutex<dyn ROSThread>>) {
        // 获取节点名称作为线程名称
        let node_lock = node.lock().unwrap();
        let thread_name = node_lock.get_thread_name() + "_thread";
        drop(node_lock);

        let condvar = self.condvar.clone();
        let node = node.clone();
        let thread = thread::Builder::new()
            .name(thread_name)
            .spawn(move || {
                let mut node_lock = node.lock().unwrap();
                node_lock.init();
                drop(node_lock);
                let (flag, cvar, lock) = &*condvar;
                loop {
                    let mut locked = lock.lock().unwrap();
                    while !flag.load(Ordering::SeqCst) {
                        locked = cvar.wait(locked).unwrap();
                    }
                    drop(locked);

                    let mut node_lock = node.lock().unwrap();
                    node_lock.start();
                    let period = node_lock.get_period();
                    while flag.load(Ordering::SeqCst) {
                        let start_time = Instant::now();

                        node_lock.update();

                        let elapsed_time = start_time.elapsed();
                        if period > elapsed_time {
                            thread::sleep(period - elapsed_time);
                        }
                    }
                    node_lock.finalize();
                    drop(node_lock);
                }
            })
            .unwrap();
        self.threads.push(thread);
    }

    pub fn add_thread_rwlock(&mut self, node: Arc<RwLock<dyn ROSThread>>) {
        let condvar = self.condvar.clone();
        let node = node.clone();
        let thread = thread::spawn(move || {
            let mut node_lock = node.write().unwrap();
            node_lock.init();
            drop(node_lock);
            let (flag, cvar, lock) = &*condvar;
            loop {
                let mut locked = lock.lock().unwrap();
                while !flag.load(Ordering::SeqCst) {
                    locked = cvar.wait(locked).unwrap();
                }
                drop(locked);

                let mut node_lock = node.write().unwrap();
                node_lock.start();
                let period = node_lock.get_period();
                while flag.load(Ordering::SeqCst) {
                    let start_time = Instant::now();

                    node_lock.update();

                    let elapsed_time = start_time.elapsed();
                    if period > elapsed_time {
                        thread::sleep(period - elapsed_time);
                    }
                }
                node_lock.finalize();
                drop(node_lock);
            }
        });
        self.threads.push(thread);
    }

    pub fn start_all(&self) {
        let (flag, cvar, _) = &*self.condvar;
        flag.store(true, Ordering::SeqCst);
        cvar.notify_all();
    }

    pub fn stop_all(&self) {
        let (flag, _, _) = &*self.condvar;
        flag.store(false, Ordering::SeqCst);
    }
}

impl Default for ThreadManage {
    fn default() -> Self {
        Self::new()
    }
}
