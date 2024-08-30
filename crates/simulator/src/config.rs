use std::sync::{Arc, Mutex, RwLock};

use crate::{Bullet, SimulatorN};
use robot::SeriesRobot;

pub fn create_simulator<R: SeriesRobot<N> + 'static, const N: usize>(
    simulator_type: String,
    robot_name: String,
    path: String,
    robot: Arc<RwLock<R>>,
) -> Arc<Mutex<dyn SimulatorN<N>>> {
    let name = format!("{}:{}", simulator_type, robot_name);
    match simulator_type.as_str() {
        // 在这里按照仿真器类型创建仿真器
        "bullet" => Arc::new(Mutex::new(Bullet::<R, N>::new(name, path, robot))),
        _ => panic!("Controller type not found"),
    }
}
