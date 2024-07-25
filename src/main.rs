#![feature(trait_upcasting)]

mod collision_object;
mod config;
mod exp;
mod thread_manage;

fn main() {
    // ! 初始化实验，将从 ${CONFIG_PATH}/config.json 中读取配置文件,并生成对应的实验森林，包括机器人树、控制器树、规划器树等
    let exp = exp::Exp::init();

    // ! 主循环家人们，赞美主循环
    loop {
        // ! 更新任务，将从 ${TASK_PATH}/task.json 中读取任务文件和参数信息文件，并生成对应的任务树
        exp.update_tesk();
        // ! 所有线程启动启动启动
        exp.thread_manage.start_all();

        // TODO: 获取任务结束或者异常的信号

        // ! 所有线程停一会儿，等待下个任务到来
        exp.thread_manage.stop_all();
    }
}
