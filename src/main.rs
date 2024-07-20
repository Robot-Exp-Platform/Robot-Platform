#![feature(trait_upcasting)]

mod collision_object;
mod config;
mod exp;
mod thread_manage;

fn main() {
    let exp = exp::Exp::init();

    loop {
        exp.update_tesk();
        exp.thread_manage._start_all();

        // TODO: 获取任务结束或者异常的信号

        exp.thread_manage._stop_all();
    }
}
