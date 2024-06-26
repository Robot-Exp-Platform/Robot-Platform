use serde::Deserialize;
use serde_json::from_reader;
use std::fs::File;
use std::sync::{Arc, RwLock};

use crate::config::CONFIG_PATH;
use controller::config::build_controller;
use planner::config::build_planner;
use robot::robots::{panda, robot_list::RobotList};

pub struct Exp {
    pub robot_exp: Arc<RwLock<dyn robot::Robot>>,
    pub controller_exp: Box<dyn controller::Controller>,
    pub planner_exp: Box<dyn planner::Planner>,
}

#[derive(Debug, Deserialize)]
struct Config {
    name: String,
    robot_type: String,
    controller: String,
    planner: String,
    robots: Option<Vec<Config>>,
}

impl Exp {
    pub fn new(
        robot_exp: Arc<RwLock<dyn robot::Robot>>,
        controller_exp: Box<dyn controller::Controller>,
        planner_exp: Box<dyn planner::Planner>,
    ) -> Exp {
        Exp {
            robot_exp,
            controller_exp,
            planner_exp,
        }
    }

    fn build_exp_tree(
        config: Config,
        path: String,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Box<dyn controller::Controller>,
        Box<dyn planner::Planner>,
    ) {
        match config.robot_type.as_str() {
            "RobotList" => {
                // 建立该节点的各个模块
                let robot_list = Arc::new(RwLock::new(RobotList::new(
                    config.name.clone(),
                    path.clone(),
                )));
                let mut controller = build_controller(
                    config.controller,
                    config.robot_type.clone(),
                    path.clone(),
                    robot_list.clone(),
                );
                let mut planner = build_planner(
                    config.planner,
                    config.robot_type.clone(),
                    path.clone(),
                    robot_list.clone(),
                );

                // 递归建立子节点
                for robot_config in config.robots.unwrap() {
                    let (robot, child_controller, child_planner) = Exp::build_exp_tree(
                        robot_config,
                        format!("{}{}/", path.clone(), config.name.clone()),
                    );
                    robot_list.write().unwrap().add_robot(robot);
                    controller.add_controller(child_controller);
                    planner.add_planner(child_planner);
                }
                (robot_list, controller, planner)
            }
            "panda" => {
                let robot = panda::Panda::new_with_name(config.name.clone(), path.clone());
                let robot = Arc::new(RwLock::new(robot));
                let controller = build_controller(
                    config.controller,
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                let planner = build_planner(
                    config.planner,
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                (robot, controller, planner)
            }
            _ => panic!("Unknown robot type: {}", config.robot_type.clone()),
        }
    }

    pub fn init() -> Exp {
        // 加载配置文件
        let config_file = File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let (robot, controller, planner) = Exp::build_exp_tree(config, "".to_string());
        Exp::new(robot, controller, planner)
    }
}
