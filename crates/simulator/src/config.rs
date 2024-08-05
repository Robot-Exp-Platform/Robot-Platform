use std::sync::{Arc, Mutex, RwLock};

use crate::simulator_trait::Simulator;
use crate::simulators::bullet::Bullet;
use robot::robot_trait::Robot;

pub fn create_simulator<R: Robot + 'static, const N: usize>(
    simulator_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Simulator>> {
    let name = format!("{}:{}", simulator_type, robot_name);
    match simulator_type.as_str() {
        // 在这里按照仿真器类型创建仿真器
        "bullet" => Arc::new(Mutex::new(Bullet::<R, N>::new_without_params(
            name, path, robot,
        ))),
        _ => panic!("Controller type not found"),
    }
}
