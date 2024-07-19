use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
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

    pub fn add_thread<FI, FS, FU>(&mut self, init: FI, start: FS, update: FU)
    where
        FI: FnOnce() + Send + 'static,
        FS: Fn() + Send + 'static,
        FU: Fn() + Send + 'static,
    {
        let condvar = self.condvar.clone();
        let thread = thread::spawn(move || {
            init();
            let &(ref flag, ref cvar, ref lock) = &*condvar;
            loop {
                let mut locked = lock.lock().unwrap();
                while !flag.load(Ordering::SeqCst) {
                    locked = cvar.wait(locked).unwrap();
                }
                drop(locked);

                start();

                while flag.load(Ordering::SeqCst) {
                    update();
                }
            }
        });
        self.threads.push(thread);
    }

    pub fn start_all(&self) {
        let &(ref flag, ref cvar, _) = &*self.condvar;
        flag.store(true, Ordering::SeqCst);
        cvar.notify_all();
    }

    pub fn stop_all(&self) {
        let &(ref flag, _, _) = &*self.condvar;
        flag.store(false, Ordering::SeqCst);
    }
}
