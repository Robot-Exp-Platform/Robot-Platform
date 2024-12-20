#![feature(trait_alias)]
#![feature(trait_upcasting)]
#![feature(more_float_constants)]

mod config;
mod exp;

use tracing_appender::{non_blocking, rolling};
use tracing_subscriber::{fmt, layer::SubscriberExt, util::SubscriberInitExt, Registry};

use exp::Exp;
use node::NodeBehavior;

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

    let mut exp = Exp::from_json("./config.json", "./task.json");

    exp.init();

    while exp.state() == node::NodeState::Running {
        exp.update();
    }
}
