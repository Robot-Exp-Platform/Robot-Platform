pub trait ROSThread: Send + Sync {
    fn init(&self) {}
    fn start(&self) {}
    fn update(&self) {}
}
