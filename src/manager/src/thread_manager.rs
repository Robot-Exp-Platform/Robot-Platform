use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

#[derive(Default)]
#[allow(dead_code)]
pub struct ThreadManager {
    threads: Vec<thread::JoinHandle<()>>,
    running: Arc<AtomicBool>,
    condvar: Arc<(Mutex<bool>, Condvar)>,
}
