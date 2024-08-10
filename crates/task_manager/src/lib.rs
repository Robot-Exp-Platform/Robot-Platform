pub mod ros_thread;
pub mod state_collector;
pub mod task;
pub mod task_manage;
pub mod thread_manage;

#[macro_export]
macro_rules! generate_node_method {
    () => {
        fn get_name(&self) -> String {
            self.name.clone()
        }
        fn get_path(&self) -> String {
            self.path.clone()
        }

        fn set_params(&mut self, params: Value) {
            self.params = from_value(params).unwrap();
        }
    };
}
