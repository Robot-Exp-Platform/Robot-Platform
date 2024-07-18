use serde::Deserialize;
use serde_json::from_reader;
use std::fs::File;
use std::sync::{Arc, RwLock};

use crate::config::CONFIG_PATH;
use crate::thread_manage::{self, ThreadManage};
use controller::config::create_controller;
use planner::config::create_planner;
use robot::robots::{panda, robot_list::RobotList};

pub struct Exp {
    pub thread_manage: ThreadManage,

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
        thread_manage: ThreadManage,
        robot_exp: Arc<RwLock<dyn robot::Robot>>,
        controller_exp: Arc<dyn controller::Controller>,
        planner_exp: Arc<dyn planner::Planner>,
    ) -> Exp {
        Exp {
            thread_manage,
            robot_exp,
            controller_exp,
            planner_exp,
        }
    }

    fn build_exp_tree(
        config: Config,
        path: String,
        thread_manage: ThreadManage,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Box<dyn controller::Controller>,
        Box<dyn planner::Planner>,
    ) {
        match config.robot_type.as_str() {
            "robot_list" => {
                let robot = RobotList::new(config.name.clone(), path.clone());
                let robot = Arc::new(RwLock::new(robot));
                let mut controller = create_controller::<RobotList, 0>(
                    config.controller.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                let mut planner = create_planner::<RobotList, 0>(
                    config.planner.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                for robot_config in config.robots.unwrap() {
                    let (child_robot, child_controller, child_planner) =
                        Exp::build_exp_tree(robot_config, path.clone(), thread_manage);
                    robot.write().unwrap().add_robot(child_robot);
                    controller.add_controller(child_controller);
                    planner.add_planner(child_planner);
                }
                (robot, controller, planner)
            }
            "panda" => {
                let robot = panda::Panda::new(path.clone());
                let robot = Arc::new(RwLock::new(robot));
                let controller = create_controller::<panda::Panda, { panda::PANDA_DOF }>(
                    config.controller.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                let planner = create_planner::<panda::Panda, { panda::PANDA_DOF }>(
                    config.planner.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                thread_manage.add_thread(
                    || controller.init(),
                    || controller.start(),
                    || controller.update(),
                );
                thread_manage.add_thread(
                    || planner.init(),
                    || planner.start(),
                    || planner.update(),
                );
                (robot, controller, planner)
            }
            _ => panic!("Unknown robot type"),
        }
    }

    pub fn init() -> Exp {
        // 加载配置文件
        let config_file = File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let thread_manage = ThreadManage::new();
        let (robot, controller, planner) =
            Exp::build_exp_tree(config, "".to_string(), thread_manage);
        Exp::new(thread_manage, robot, controller, planner)
    }
}
