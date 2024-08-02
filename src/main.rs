#![feature(trait_upcasting)]

use task_manager::ros_thread::ROSThread;

mod collision_object;
mod config;
mod exp;
mod msg;

fn main() {
    // ! 初始化实验，将从 ${CONFIG_PATH}/config.json 中读取配置文件,并生成对应的实验森林，包括机器人树、控制器树、规划器树等
    let mut exp = exp::Exp::new();
    exp.init();
    // ! 主循环家人们，赞美主循环
    loop {
        // ! 更新任务，将从 ${TASK_PATH}/task.json 中读取任务文件和参数信息文件，并生成对应的任务树
        // exp.start();

        while exp.is_running() {
            exp.update();
        }
    }
}
