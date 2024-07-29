// use std::{
//     path,
//     sync::{Arc, Mutex, RwLock},
// };

// use robot::robot_trait::Robot;

// pub fn create_simulator<R: Robot + 'static, const N: usize>(
//     simulator_type: String,
//     robot_type: String,
//     path: String,
//     robot: Arc<RwLock<R>>,
// ) -> Arc<Mutex<dyn Simulator>> {
//     match simulator_type.as_str() {
//         // 在这里按照仿真器类型创建仿真器
//         _ => panic!("Controller type not found"),
//     }
// }
