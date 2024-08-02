use std::sync::{Arc, Mutex, RwLock};

use crate::simulator_trait::Simulator;
use crate::simulators::bullet::Bullet;
use robot::robot_trait::Robot;

pub fn create_simulator<R: Robot + 'static, const N: usize>(
    simulator_type: String,
    robot_type: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn Simulator>> {
    match simulator_type.as_str() {
        // 在这里按照仿真器类型创建仿真器
        "pybullet" => Arc::new(Mutex::new(Bullet::<R, N>::new(
            robot_type + "_bullet",
            path,
            robot,
        ))),
        _ => panic!("Controller type not found"),
    }
}
