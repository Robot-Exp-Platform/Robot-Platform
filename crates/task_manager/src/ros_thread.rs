use std::time::Duration;

pub trait ROSThread: Send + Sync {
    fn init(&mut self) {}
    fn start(&mut self) {}
    fn update(&mut self) {}
    fn get_period(&self) -> Duration {
        Duration::from_secs(0)
    }
}
