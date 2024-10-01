use std::time::Duration;

pub trait Node: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn period(&self) -> Duration {
        Duration::from_secs(0)
    }
    fn thread_name(&self) -> String {
        String::from("unnamed_thread")
    }
    fn finalize(&mut self) {}
}
