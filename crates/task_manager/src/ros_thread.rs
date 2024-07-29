pub trait ROSThread: Send + Sync {
    fn init(&self) {}
    fn start(&mut self) {}
    fn update(&self) {}
}
