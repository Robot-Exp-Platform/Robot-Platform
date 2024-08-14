use crossbeam::queue::SegQueue;
use std::sync::{Arc, RwLock};
use std::thread::sleep;
use std::time::Duration;

use message::control_command::ControlCommand;
use robot::robots::panda::Panda;
use simulator::{simulators::bullet::Bullet, Simulator};
use task_manager::ros_thread::ROSThread;


fn main() {
    let robot = Panda::new_panda("panda_1".to_string(), "/robot".to_string());
    let mut simulator = Bullet::<Panda, 7>::new(
        "bullet:panda_1".to_string(),
        "/simulator".to_string(),
        Arc::new(RwLock::new(robot)),
    );

    let controller_command_queue = Arc::new(SegQueue::new());

    // 随便往队列里面塞点数据
    controller_command_queue.push(ControlCommand::Joint(vec![
        2.02244, -4.08431, 6.07565, -2.43899, 5.4941, -4.27498, -1.38176,
    ]));
    controller_command_queue.push(ControlCommand::TauWithPeriod(
        1.0 / 240.0,
        vec![
            2.02244, -4.08431, 6.07565, -2.43899, 5.4941, -4.27498, -1.38176,
        ],
    ));
    
    simulator.set_controller_command_queue(controller_command_queue);

    simulator.init();
    loop {
        simulator.update();
        sleep(Duration::from_secs_f64(0.5));
    }
}
