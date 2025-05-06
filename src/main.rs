#![feature(trait_alias)]
#![feature(more_float_constants)]
#![feature(box_patterns)]

mod config;
mod exp;

use clap::{Arg, Command};
use std::net::TcpStream;

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
// const CONFIG_PATH: &str = "./example/explanner_interp_excontroller_plant_config.json";
// const TASK_PATH: &str = "./example/explanner_interp_excontroller_plant_task.json";

fn main() {
    // 使用 clap 解析命令行参数
    let matches = Command::new("Robot Platform")
        .version("0.1.0")
        .author("Your Name <your.email@example.com>")
        .about("Robot Experiment Platform")
        .arg(
            Arg::new("config")
                .short('c')
                .long("config")
                .value_name("CONFIG")
                .help("Path to the config.json file")
                .required(true),
        )
        .arg(
            Arg::new("task")
                .short('t')
                .long("task")
                .value_name("TASK")
                .help("Path to the task.json file")
                .required(true),
        )
        .arg(
            Arg::new("port")
                .short('p')
                .long("port")
                .value_name("PORT")
                .help("Port for log transmission")
                .required(true),
        )
        .get_matches();

    let config_path = matches.get_one::<String>("config").unwrap();
    let task_path = matches.get_one::<String>("task").unwrap();
    let port = matches.get_one::<String>("port").unwrap();

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

    let tcp = TcpStream::connect(format!("localhost:{}", port)).unwrap();
    let (non_blocking_appender_tcp, _guard_tcp) = non_blocking(tcp);
    let tcp_layer = fmt::layer()
        .json()
        .with_ansi(false)
        .with_writer(non_blocking_appender_tcp);

    // 注册
    Registry::default()
        .with(file_layer_log)
        .with(file_layer_json)
        .with(tcp_layer)
        .init();

    let mut exp = Exp::from_json(config_path, task_path);

    exp.init();

    while exp.state() == node::NodeState::Running {
        exp.update();
    }
}
