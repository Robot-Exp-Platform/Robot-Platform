use crossbeam::queue::SegQueue;
use nalgebra as na;
use std::sync::{Arc, RwLock};
use std::thread::sleep;
use std::time::Duration;

use message::ControlCommandN;
use robot::Panda;
use simulator::{Bullet, SimulatorN};
use task_manager::ROSThread;

fn main() {
    let robot = Panda::new_panda("panda_1".to_string(), "/robot".to_string());
    let mut simulator = Bullet::<Panda, 7>::new(
        "bullet:panda_1".to_string(),
        "/simulator".to_string(),
        Arc::new(RwLock::new(robot)),
    );

    let controller_command_queue = Arc::new(SegQueue::new());

    // 随便往队列里面塞点数据
    controller_command_queue.push(ControlCommandN::Joint(na::SVector::from_column_slice(&[
        2.02244, -4.08431, 6.07565, -2.43899, 5.4941, -4.27498, -1.38176,
    ])));
    controller_command_queue.push(ControlCommandN::TauWithPeriod(
        1.0 / 240.0,
        na::SVector::from_column_slice(&[
            2.02244, -4.08431, 6.07565, -2.43899, 5.4941, -4.27498, -1.38176,
        ]),
    ));

    simulator.set_controller_command_queue(controller_command_queue);

    simulator.init();
    loop {
        simulator.update();
        sleep(Duration::from_secs_f64(0.5));
    }
}
