use std::time::Duration;

pub trait Node: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn finalize(&mut self) {}
    fn is_running(&mut self) -> bool {
        true
    }
    fn period(&self) -> Duration {
        Duration::from_secs(0)
    }
    fn node_name(&self) -> String {
        String::from("unnamed_thread")
    }
}
