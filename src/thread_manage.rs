use std::sync::{Arc, Condvar, Mutex};
use std::thread;
pub struct ThreadManage {
    threads: Vec<thread::JoinHandle<()>>,
    pub condvar: Arc<(Mutex<bool>, Condvar)>,
}

impl ThreadManage {
    pub fn new() -> Self {
        let condvar = Arc::new((Mutex::new(false), Condvar::new()));
        ThreadManage {
            threads: Vec::new(),
            condvar,
        }
    }

    pub fn add_thread<FI, FS, FU>(&mut self, init: FI, start: FS, update: FU)
    where
        FI: FnOnce() + Send + 'static,
        FS: Fn() + Send + 'static,
        FU: Fn() + Send + 'static,
    {
        let condvar = self.condvar.clone();
        let thread = thread::spawn(move || {
            init();
            let &(ref lock, ref cvar) = &*condvar;
            loop {
                let mut started = lock.lock().unwrap();
                while !*started {
                    started = cvar.wait(started).unwrap();
                }

                start();

                while *started {
                    drop(started); // Release the lock before running update
                    update();
                    started = lock.lock().unwrap();
                }
            }
        });
        self.threads.push(thread);
    }
}
