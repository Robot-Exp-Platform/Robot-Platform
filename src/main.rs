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
    std::fs::remove_file("logs/test.log").unwrap_or_default();

    // 初始化日志
    let file_appender = rolling::never("logs", "test.log");
    let (non_blocking_appender, _guard) = non_blocking(file_appender);
    let file_layer = fmt::layer()
        .with_ansi(false)
        .with_writer(non_blocking_appender);

    // 注册
    Registry::default().with(file_layer).init();

    let mut exp = Exp::from_json("./config.json", "./task.json");

    exp.init();

    while exp.state() == node::NodeState::Running {
        exp.update();
    }
}
