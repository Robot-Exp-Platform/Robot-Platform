use serde::Deserialize;
use serde_json::from_reader;
use std::fs::File;
use std::path;
use std::sync::{Arc, Mutex, RwLock};

use crate::config::CONFIG_PATH;
use controller::config::create_controller;
use planner::config::create_planner;
use robot::robots::{panda, robot_list::RobotList};
use simulator::config::create_simulator;
use task_manager::ros_thread::ROSThread;
use task_manager::task::Task;
use task_manager::thread_manage::ThreadManage;

#[allow(dead_code)]
pub struct Exp {
    // Exp 是一个森林状的结构，其中的包含 robot tree, controller tree, planner tree 等等树状结构的根节点，通过管理 exp 实现管理整个结构的目的
    pub thread_manage: ThreadManage,
    pub task_manage: Option<Task>,

    pub robot_exp: Arc<RwLock<dyn robot::Robot>>,
    pub controller_exp: Arc<Mutex<dyn controller::Controller>>,
    pub planner_exp: Arc<Mutex<dyn planner::Planner>>,
    pub simulator_exp: Arc<Mutex<dyn simulator::Simulator>>,
}

#[derive(Debug, Deserialize)]
struct Config {
    // Config 从 CONFIG_PATH/config.json 中读取的配置以如下结构保存
    name: String,
    robot_type: String,
    controller: String,
    planner: String,
    simulator: String,
    robots: Option<Vec<Config>>,
}

impl Exp {
    pub fn new() -> Exp {
        // 加载配置文件
        let config_file = File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let mut thread_manage = ThreadManage::new();
        let (robot, controller, planner, simulator) =
            Exp::build_exp_tree(config, "".to_string(), &mut thread_manage);
        Exp {
            thread_manage,
            task_manage: None,
            robot_exp: robot,
            controller_exp: controller,
            planner_exp: planner,
            simulator_exp: simulator,
        }
    }

    pub fn init() {}

    #[allow(clippy::type_complexity)]
    fn build_exp_tree(
        config: Config,
        path: String,
        thread_manage: &mut ThreadManage,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Arc<Mutex<dyn controller::Controller>>,
        Arc<Mutex<dyn planner::Planner>>,
        Arc<Mutex<dyn simulator::Simulator>>,
    ) {
        // 由于 Exo 的树状结构，所以这里需要递归的生成树状结构，每款机器人的 常数参量并不相同，所以需要在这里枚举以创建不同大小的 控制器、规划器 等
        match config.robot_type.as_str() {
            "robot_list" => {
                // ! 这里的 robot_list 是一个虚拟机器人，它的作用是将多个机器人组合成一个机器人，这样可以在一个控制器中控制多个机器人，所以 robot_list 一般指叶节点
                let robot = RobotList::new(config.name.clone(), path.clone());
                let robot = Arc::new(RwLock::new(robot));

                // 分别判断 controller 和 planner 的类型，然后创建对应的 controller 和 planner
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
                let simulator = create_simulator::<RobotList, 0>(
                    config.simulator.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );

                // ! 递归!树就是从这里长起来的
                for robot_config in config.robots.unwrap() {
                    let (child_robot, child_controller, child_planner, child_simulator) =
                        Exp::build_exp_tree(robot_config, path.clone(), thread_manage);
                    robot.write().unwrap().add_robot(child_robot);
                    controller.lock().unwrap().add_controller(child_controller);
                    planner.lock().unwrap().add_planner(child_planner);
                    simulator.lock().unwrap().add_simulator(child_simulator);
                }
                (robot, controller, planner, simulator)
            }
            "panda" => {
                // ! 经典的 Franka Emika Panda 机器人
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
                let simulator = create_simulator::<panda::Panda, { panda::PANDA_DOF }>(
                    config.simulator.clone(),
                    config.robot_type.clone(),
                    path.clone(),
                    robot.clone(),
                );

                // 需要给控制器和规划器开辟独立的线程
                thread_manage.add_thread(controller.clone());
                thread_manage.add_thread(planner.clone());
                thread_manage.add_thread(simulator.clone());

                (robot, controller, planner, simulator)
            }
            _ => panic!("Unknown robot type"),
        }
    }

    pub fn get_controller_by_path(
        controller: &Arc<Mutex<dyn controller::Controller>>,
        path: String,
    ) -> Option<Arc<Mutex<dyn controller::Controller>>> {
        // !从 controller 树中根据 path 获取 controller,辅助函数,之后可能作为私有函数.这个函数本身不依赖 exp 所以之后可以考虑从 exp 中那出去
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
        // !从 planner 树中根据 path 获取 planner,辅助函数,之后可能作为私有函数.这个函数本身不依赖 exp 所以之后可以考虑从 exp 中那出去
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

    fn update_tesk(&mut self) {
        // ! 从 task.json 中读取任务,然后,将对应的参数设置到对应的节点中去,这将有助于在后期反复迭代任务,但是就目前来说,只有更新参数的功能了
        let task_file = File::open(path::Path::new("task.json")).expect("Failed to open task file");
        let task: Task = from_reader(task_file).expect("Failed to parse task file");

        let controller = self.controller_exp.clone();
        let planner = self.planner_exp.clone();

        for param in &task.params {
            match param.node_type.as_str() {
                "controller" => {
                    let controller =
                        Exp::get_controller_by_path(&controller, param.path.clone()).unwrap();
                    controller.lock().unwrap().set_params(param.param.clone());
                }
                "planner" => {
                    let planner = Exp::get_planner_by_path(&planner, param.path.clone()).unwrap();
                    planner.lock().unwrap().set_params(param.param.clone());
                }
                _ => panic!("Unknown node type"),
            }
        }
        self.task_manage = Some(task);
    }

    pub fn is_running(&self) -> bool {
        // ! 一个状态判断函数,如果运行终端或者当前 task 已经运行完成,则返回 false,开始下一轮的任务安排
        // unimplemented!();
        true
    }
}

impl ROSThread for Exp {
    // ! 为 Exp 实现 ROSThread trait,这将使得 Exp 可以被 ThreadManage 管理,同时也具备基本的运行函数
    fn init(&self) {}

    fn start(&mut self) {
        // ! 所有线程停一会儿，等待下个任务到来
        self.thread_manage.stop_all();

        self.update_tesk();

        // ! 所有线程启动启动启动
        self.thread_manage.start_all();
    }

    fn update(&self) {}
}
