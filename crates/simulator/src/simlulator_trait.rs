use task_manager::ros_thread::ROSThread;
pub trait Simlulator: ROSThread {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
}
