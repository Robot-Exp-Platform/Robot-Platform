use crossbeam::queue::SegQueue;
use std::sync::{Arc, RwLock};

use message::control_command::ControlCommand::Joint;
use robot::robots::panda::Panda;
use simulator::{simulators::bullet::Bullet, Simulator};
use task_manager::ros_thread::ROSThread;

fn main() {
    let robot = Panda::new("panda_1".to_string(), "/robot".to_string());
    let mut simulator = Bullet::<Panda, 7>::new(
        "bullet:panda_1".to_string(),
        "/simulator".to_string(),
        Arc::new(RwLock::new(robot)),
    );

    let controller_command_queue = Arc::new(SegQueue::new());

    // 随便往队列里面塞点数据
    controller_command_queue.push(Joint(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    controller_command_queue.push(Joint(vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]));

    simulator.set_controller_command_queue(controller_command_queue);

    simulator.init();

    loop{
        simulator.update();
    }
}
