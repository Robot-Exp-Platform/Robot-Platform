#![feature(trait_alias)]
#![feature(trait_upcasting)]
#![feature(more_float_constants)]
#![feature(box_patterns)]

mod config;
mod exp;

use tracing_appender::{non_blocking, rolling};
use tracing_subscriber::{Registry, fmt, layer::SubscriberExt, util::SubscriberInitExt};

use exp::Exp;
use node::NodeBehavior;

// 正式运行时使用
// const CONFIG_PATH: &str = "./config/config.json";
// const TASK_PATH: &str = "./config/task.json";

// 非实时指令样例
// const CONFIG_PATH: &str = "./example/explanner_plant_config.json";
// const TASK_PATH: &str = "./example/explanner_plant_task.json";
// 实时指令样例
const CONFIG_PATH: &str = "./example/explanner_interp_excontroller_plant_config.json";
const TASK_PATH: &str = "./example/explanner_interp_excontroller_plant_task.json";

fn main() {
    // 删除已有的日志文件
    std::fs::remove_file("./logs/info.log").unwrap_or_default();
    std::fs::remove_file("./logs/info.json").unwrap_or_default();

    // 初始化日志
    let file_appender_log = rolling::never("logs", "info.log");
    let (non_blocking_appender_log, _guard_log) = non_blocking(file_appender_log);
    let file_layer_log = fmt::layer()
        .with_ansi(false)
        .with_writer(non_blocking_appender_log);

    let file_appender_json = rolling::never("logs", "info.json");
    let (non_blocking_appender_json, _guard_json) = non_blocking(file_appender_json);
    let file_layer_json = fmt::layer()
        .json()
        .with_ansi(false)
        .with_writer(non_blocking_appender_json);

    // 注册
    Registry::default()
        .with(file_layer_log)
        .with(file_layer_json)
        .init();

    let mut exp = Exp::from_json(CONFIG_PATH, TASK_PATH);

    exp.init();

    while exp.state() == node::NodeState::Running {
        exp.update();
    }
}
