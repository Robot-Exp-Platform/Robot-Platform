use controller::Controller;
use serde::Deserialize;
use serde_json::from_reader;
use std::fs::File;
use std::path;
use std::sync::{Arc, Mutex, RwLock};

use crate::config::CONFIG_PATH;
use crate::thread_manage::ThreadManage;
use controller::config::create_controller;
use planner::config::create_planner;
use robot::robots::{panda, robot_list::RobotList};
use task_manager::task::{Task, TaskParam};

#[allow(dead_code)]
pub struct Exp {
    pub thread_manage: ThreadManage,

    pub robot_exp: Arc<RwLock<dyn robot::Robot>>,
    pub controller_exp: Arc<Mutex<dyn controller::Controller>>,
    pub planner_exp: Arc<Mutex<dyn planner::Planner>>,
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
        controller_exp: Arc<Mutex<dyn controller::Controller>>,
        planner_exp: Arc<Mutex<dyn planner::Planner>>,
    ) -> Exp {
        Exp {
            thread_manage,
            robot_exp,
            controller_exp,
            planner_exp,
        }
    }

    pub fn init() -> Exp {
        // 加载配置文件
        let config_file = File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let mut thread_manage = ThreadManage::new();
        let (robot, controller, planner) =
            Exp::build_exp_tree(config, "".to_string(), &mut thread_manage);
        Exp::new(thread_manage, robot, controller, planner)
    }

    #[allow(clippy::type_complexity)]
    fn build_exp_tree(
        config: Config,
        path: String,
        thread_manage: &mut ThreadManage,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Arc<Mutex<dyn controller::Controller>>,
        Arc<Mutex<dyn planner::Planner>>,
    ) {
        match config.robot_type.as_str() {
            "robot_list" => {
                let robot = RobotList::new(config.name.clone(), path.clone());
                let robot = Arc::new(RwLock::new(robot));
                let controller = create_controller::<RobotList, 0>(
                    config.controller.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                let planner = create_planner::<RobotList, 0>(
                    config.planner.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );
                for robot_config in config.robots.unwrap() {
                    let (child_robot, child_controller, child_planner) =
                        Exp::build_exp_tree(robot_config, path.clone(), thread_manage);
                    robot.write().unwrap().add_robot(child_robot);
                    controller.lock().unwrap().add_controller(child_controller);
                    planner.lock().unwrap().add_planner(child_planner);
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

                thread_manage.add_thread(controller.clone());
                thread_manage.add_thread(planner.clone());

                (robot, controller, planner)
            }
            _ => panic!("Unknown robot type"),
        }
    }

    pub fn get_controller_by_path(
        controller: &Arc<Mutex<dyn Controller>>,
        path: String,
    ) -> Option<Arc<Mutex<dyn controller::Controller>>> {
        let parts: Vec<&str> = path.trim_start_matches("/").split('/').collect();

        if parts.len() == 1 {
            return Some(controller.clone());
        }

        for child in controller.lock().unwrap().get_controller().iter() {
            if child.lock().unwrap().get_name() == parts[0] {
                return Exp::get_controller_by_path(controller, parts[1..].join("/"));
            }
        }

        None
    }

    pub fn get_planner_by_path(
        planner: &Arc<Mutex<dyn planner::Planner>>,
        path: String,
    ) -> Option<Arc<Mutex<dyn planner::Planner>>> {
        let parts: Vec<&str> = path.trim_start_matches("/").split('/').collect();

        if parts.len() == 1 {
            return Some(planner.clone());
        }

        for child in planner.lock().unwrap().get_planner().iter() {
            if child.lock().unwrap().get_name() == parts[0] {
                return Exp::get_planner_by_path(planner, parts[1..].join("/"));
            }
        }

        None
    }

    pub fn update_tesk(&self) {
        let task_file = File::open(path::Path::new("task.json")).expect("Failed to open task file");
        let task: Task = from_reader(task_file).expect("Failed to parse task file");

        let controller = self.controller_exp.clone();
        let planner = self.planner_exp.clone();

        for param in task.params {
            match param.node_type.as_str() {
                "controller" => {
                    let controller = Exp::get_controller_by_path(&controller, param.path).unwrap();
                    controller.lock().unwrap().set_params(param.param);
                }
                "planner" => {
                    let planner = Exp::get_planner_by_path(&planner, param.path).unwrap();
                    planner.lock().unwrap().set_params(param.param);
                }
                _ => panic!("Unknown node type"),
            }
        }
    }
}
