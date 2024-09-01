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
        fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
            self.node.sensor = Some(sensor);
        }
    };
}

#[macro_export]
macro_rules! generate_controller_method {
    () => {
        generate_node_method!();
    };
}

#[macro_export]
macro_rules! generate_planner_method {
    () => {
        generate_node_method!();
        fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>) {
            self.node.target_queue = target_queue;
        }
        fn set_state_collector(&mut self, state_collector: StateCollector) {
            self.node.state_collector = state_collector;
        }

        fn add_planner(&mut self, _planner: Arc<Mutex<dyn Planner>>) {}
    };
}

#[macro_export]
macro_rules! generate_simulator_method {
    () => {
        generate_node_method!();
        fn subscribe_post_office(
            &mut self,
            sender: Sender<(String, String)>,
            receiver: Receiver<String>,
        ) {
            self.node.sender = Some(sender);
            self.node.receiver = Some(receiver);
        }
    };
}
