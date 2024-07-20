use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

use robot::ros_thread::ROSThread;
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
        let condvar = self.condvar.clone();
        let node = node.clone();
        let thread = thread::spawn(move || {
            let node = node.lock().unwrap();
            node.init();
            let (flag, cvar, lock) = &*condvar;
            loop {
                let mut locked = lock.lock().unwrap();
                while !flag.load(Ordering::SeqCst) {
                    locked = cvar.wait(locked).unwrap();
                }
                drop(locked);

                node.start();

                while flag.load(Ordering::SeqCst) {
                    node.update();
                }
            }
        });
        self.threads.push(thread);
    }

    pub fn _start_all(&self) {
        let (flag, cvar, _) = &*self.condvar;
        flag.store(true, Ordering::SeqCst);
        cvar.notify_all();
    }

    pub fn _stop_all(&self) {
        let (flag, _, _) = &*self.condvar;
        flag.store(false, Ordering::SeqCst);
    }
}
