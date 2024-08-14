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
        fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>) {
            self.node.track_queue = track_queue;
        }
        fn set_controller_command_queue(
            &mut self,
            controller_command_queue: Arc<SegQueue<ControlCommand>>,
        ) {
            self.node.control_command_queue = controller_command_queue;
        }
    };
}

#[macro_export]
macro_rules! generate_planner_method {
    () => {
        generate_node_method!();
        fn set_target_queue(&mut self, target_queue: Arc<SegQueue<Target>>) {
            self.node.target_queue = target_queue;
        }
        fn set_track_queue(&mut self, track_queue: Arc<SegQueue<Track>>) {
            self.node.track_queue = track_queue;
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
        fn set_controller_command_queue(
            &mut self,
            controller_command_queue: Arc<SegQueue<ControlCommand>>,
        ) {
            self.node.control_command_queue = controller_command_queue;
        }
    };
}
