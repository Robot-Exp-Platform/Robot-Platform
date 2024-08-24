use chrono::Local;
use crossbeam::queue::SegQueue;
use serde::Deserialize;
use serde_json::from_reader;
// use serde_yaml::from_reader;
use std::fs;
use std::path;
use std::sync::{Arc, Condvar, Mutex, RwLock};

use crate::config::CONFIG_PATH;
use crate::config::TASK_PATH;
use controller::config::create_controller;
use planner::config::create_planner;
#[cfg(feature = "recode")]
use recoder::EXP_NAME;
#[cfg(feature = "recode")]
use recoder::TASK_NAME;
use robot::robot_trait::SeriesRobot;
use robot::robots::franka_emika;
use robot::robots::franka_research3;
use robot::robots::panda;
use robot::robots::robot_list::RobotList;
use sensor::config::create_sensor;
use sensor::sensor_trait::Sensor;
use simulator::config::create_simulator;
use task_manager::ros_thread::ROSThread;
use task_manager::state_collector::{NodeState, StateCollector};
use task_manager::task::Task;
use task_manager::task_manage::TaskManager;
use task_manager::thread_manage::ThreadManage;

pub struct Exp {
    // Exp 是一个森林状的结构，其中的包含 robot tree, controller tree, planner tree 等等树状结构的根节点，通过管理 exp 实现管理整个结构的目的
    pub thread_manage: ThreadManage,
    pub task_manage: TaskManager,
    pub state_collector: StateCollector,

    pub _robot_tree: Arc<RwLock<dyn robot::Robot>>,
    pub sensor_list: Vec<Arc<RwLock<Sensor>>>,
    pub planner_tree: Arc<Mutex<dyn planner::Planner>>,
    pub controller_tree: Arc<Mutex<dyn controller::Controller>>,
    pub simulator_tree: Arc<Mutex<dyn simulator::Simulator>>,
}

#[derive(Debug, Deserialize)]
struct RobotConfig {
    // Config 从 CONFIG_PATH/config.json 中读取的配置以如下结构保存
    name: String,
    robot_type: String,
    planner: String,
    controller: String,
    simulator: String,
    robots: Option<Vec<RobotConfig>>,
}

#[derive(Debug, Deserialize)]
struct SensorConfig {
    // Config 从 CONFIG_PATH/config.json 中读取的配置以如下结构保存
    name: String,
    sensor_type: String,
}

#[derive(Debug, Deserialize)]
struct Config {
    robot_config: RobotConfig,
    sensor_config: Vec<SensorConfig>,
}

impl Exp {
    pub fn new() -> Exp {
        // 加载配置文件
        let config_file = fs::File::open(CONFIG_PATH).expect("Failed to open config file");
        let config: Config = from_reader(config_file).expect("Failed to parse config file");

        // 根据配置文件生成机器人树
        let mut thread_manage = ThreadManage::new();
        let mut task_manage = TaskManager::new();
        let state_collector = Arc::new((Mutex::new(NodeState::new()), Condvar::new()));
        let (robot_tree, planner_tree, controller_tree, simulator_tree) = Exp::build_exp_tree(
            config.robot_config,
            "".to_string(),
            &mut thread_manage,
            &mut task_manage,
            &state_collector,
        );

        // 生成传感器目录
        let sensor_list = config
            .sensor_config
            .iter()
            .map(|sensor_config| create_sensor(&sensor_config.sensor_type, &sensor_config.name))
            .collect();

        Exp {
            thread_manage,
            task_manage,
            state_collector,
            _robot_tree: robot_tree,
            sensor_list,
            planner_tree,
            controller_tree,
            simulator_tree,
        }
    }

    #[allow(clippy::type_complexity)]
    fn build_exp_tree(
        config: RobotConfig,
        path: String,
        thread_manage: &mut ThreadManage,
        task_manager: &mut TaskManager,
        state_collector: &StateCollector,
    ) -> (
        Arc<RwLock<dyn robot::Robot>>,
        Arc<Mutex<dyn planner::Planner>>,
        Arc<Mutex<dyn controller::Controller>>,
        Arc<Mutex<dyn simulator::Simulator>>,
    ) {
        // 由于 Exp 的树状结构，所以这里需要递归的生成树状结构，每款机器人的 常数参量并不相同，所以需要在这里枚举以创建不同大小的 控制器、规划器 等
        match config.robot_type.as_str() {
            "robot_list" => {
                // ! 这里的 robot_list 是一个虚拟机器人，它的作用是将多个机器人组合成一个机器人，这样可以在一个控制器中控制多个机器人，所以 robot_list 一般指叶节点
                let robot = RobotList::new(config.name.clone(), format!("/robot/{}", path));
                let robot = Arc::new(RwLock::new(robot));

                // 分别判断 controller 和 planner 的类型，然后创建对应的 controller 和 planner
                let (planner, controller, simulator) = create_branch(&config, path.as_str());

                // ! 递归!树就是从这里长起来的
                for robot_config in config.robots.unwrap() {
                    let (child_robot, child_planner, child_controller, child_simulator) =
                        Exp::build_exp_tree(
                            robot_config,
                            format!("{}/robot_list", path),
                            thread_manage,
                            task_manager,
                            state_collector,
                        );
                    robot.write().unwrap().add_robot(child_robot);
                    planner.lock().unwrap().add_planner(child_planner);
                    controller.lock().unwrap().add_controller(child_controller);
                    simulator.lock().unwrap().add_simulator(child_simulator);
                }
                (robot, planner, controller, simulator)
            }
            // 特判各种机器人
            _ => {
                let (robot, (planner, controller, simulator)) = create_robot(&config, &path);

                let target_queue = Arc::new(SegQueue::new());
                let track_queue = Arc::new(SegQueue::new());
                let control_command_queue = Arc::new(SegQueue::new());

                // 为 planner 配置信道
                planner
                    .lock()
                    .unwrap()
                    .set_target_queue(target_queue.clone());
                planner.lock().unwrap().set_track_queue(track_queue.clone());
                planner
                    .lock()
                    .unwrap()
                    .set_state_collector(state_collector.clone());
                state_collector.0.lock().unwrap().add_node();

                // 为 controller 配置信道
                controller
                    .lock()
                    .unwrap()
                    .set_track_queue(track_queue.clone());
                controller
                    .lock()
                    .unwrap()
                    .set_controller_command_queue(control_command_queue.clone());

                // 为 simulator 配置信道
                simulator
                    .lock()
                    .unwrap()
                    .set_controller_command_queue(control_command_queue.clone());

                task_manager
                    .add_target_node(planner.lock().unwrap().get_name(), target_queue.clone());

                // 需要给控制器和规划器开辟独立的线程
                thread_manage.add_thread(planner.clone());
                thread_manage.add_thread(controller.clone());
                thread_manage.add_thread(simulator.clone());

                (robot, planner, controller, simulator)
            }
        }
    }

    fn update_tesk(&mut self) {
        // ! 从 task.json 中读取任务,然后,将对应的参数设置到对应的节点中去,这将有助于在后期反复迭代任务,但是就目前来说,只有更新参数的功能了
        let task_file =
            fs::File::open(path::Path::new(TASK_PATH)).expect("Failed to open task file");
        let task: Task = from_reader(task_file).expect("Failed to parse task file");

        let controller = self.controller_tree.clone();
        let planner = self.planner_tree.clone();

        #[cfg(feature = "recode")]
        {
            let mut task_name = TASK_NAME.lock().unwrap();
            *task_name = task.task_name.clone();
            fs::create_dir_all(format!("./data/{}/{}", *EXP_NAME, *task_name)).unwrap();
        }

        for node in &task.nodes {
            match node.node_type.as_str() {
                "planner" => {
                    let planner = get_planner_with_name(&planner, node.name.as_str()).unwrap();
                    let mut planner = planner.lock().unwrap();
                    planner.set_params(node.param.clone());
                    planner.set_sensor(
                        get_sensor_with_name(
                            &self.sensor_list,
                            node.sensor.clone().unwrap().as_str(),
                        )
                        .unwrap(),
                    )
                }
                "controller" => {
                    let controller =
                        get_controller_with_name(&controller, node.name.as_str()).unwrap();
                    let mut controller = controller.lock().unwrap();
                    controller.set_params(node.param.clone());
                    controller.set_sensor(
                        get_sensor_with_name(
                            &self.sensor_list,
                            node.sensor.clone().unwrap().as_str(),
                        )
                        .unwrap(),
                    )
                }
                "simulator" => {
                    let simulator =
                        get_simulator_with_name(&self.simulator_tree, node.name.as_str()).unwrap();
                    let mut simulator = simulator.lock().unwrap();
                    simulator.set_params(node.param.clone());
                    simulator.set_sensor(
                        get_sensor_with_name(
                            &self.sensor_list,
                            node.sensor.clone().unwrap().as_str(),
                        )
                        .unwrap(),
                    )
                }
                _ => panic!("Unknown node type"),
            }
        }

        for robot_task in &task.robot_tasks {
            let target_queue = self
                .task_manage
                .get_queue_with_name(robot_task.name.clone())
                .unwrap();
            for target in &robot_task.targets {
                target_queue.push(target.clone());
            }
        }

        self.task_manage.set_task(task);
    }

    pub fn is_running(&self) -> bool {
        // ! 一个状态判断函数,如果运行终端或者当前 task 已经运行完成,则返回 false,开始下一轮的任务安排
        true
    }
}

impl ROSThread for Exp {
    // ! 为 Exp 实现 ROSThread trait,这将使得 Exp 可以被 ThreadManage 管理,同时也具备基本的运行函数
    fn init(&mut self) {
        println!(
            "现在是 {}，先生，祝您早上、中午、晚上好",
            Local::now().format("%Y-%m-%d %H:%M:%S")
        );
    }

    fn update(&mut self) {
        println!("开始任务");
        // ! 所有线程停一会儿，等待下个任务到来
        self.thread_manage.stop_all();
        self.update_tesk();
        // ! 所有线程启动启动启动
        self.thread_manage.start_all();
        println!("任务目标完成");

        // ! 获取状态收集器，等待任务完成
        let (state, cvar) = &*self.state_collector;
        let mut state = state.lock().unwrap();

        while state.get_node_size() != state.get_finished() {
            state = cvar.wait(state).unwrap();
            println!("被通知提醒，此时任务完成数量为：{}", state.get_finished());
        }

        println!("任务完成，状态整理器重置");
        state.refresh();
        self.thread_manage.stop_all();
    }
}

#[allow(clippy::type_complexity)]
fn create_branch(
    _config: &RobotConfig,
    _path: &str,
) -> (
    Arc<Mutex<dyn planner::Planner>>,
    Arc<Mutex<dyn controller::Controller>>,
    Arc<Mutex<dyn simulator::Simulator>>,
) {
    unimplemented!();
}

#[allow(clippy::type_complexity)]
fn create_nodes<R: SeriesRobot<N> + 'static, const N: usize>(
    config: &RobotConfig,
    path: &str,
    robot: Arc<RwLock<R>>,
) -> (
    Arc<Mutex<dyn planner::Planner>>,
    Arc<Mutex<dyn controller::Controller>>,
    Arc<Mutex<dyn simulator::Simulator>>,
) {
    let planner = create_planner::<R, N>(
        config.planner.clone(),
        config.name.clone(),
        format!("/planner/{}", path),
        robot.clone(),
    );
    let controller = create_controller::<R, N>(
        config.controller.clone(),
        config.name.clone(),
        format!("/controller/{}", path),
        robot.clone(),
    );
    let simulator = create_simulator::<R, N>(
        config.simulator.clone(),
        config.name.clone(),
        format!("/simulator/{}", path),
        robot.clone(),
    );

    (planner, controller, simulator)
}

#[allow(clippy::type_complexity)]
fn create_robot(
    config: &RobotConfig,
    path: &str,
) -> (
    Arc<RwLock<dyn robot::Robot>>,
    (
        Arc<Mutex<dyn planner::Planner>>,
        Arc<Mutex<dyn controller::Controller>>,
        Arc<Mutex<dyn simulator::Simulator>>,
    ),
) {
    match config.robot_type.as_str() {
        "panda" => {
            let robot = panda::Panda::new_panda(config.name.clone(), format!("/robot/{}", path));
            let robot = Arc::new(RwLock::new(robot));
            (
                robot.clone(),
                create_nodes::<panda::Panda, { panda::PANDA_DOF }>(config, path, robot),
            )
        }
        "franka_emika" => {
            let robot = franka_emika::FrankaEmika::new_emika(
                config.name.clone(),
                format!("/robot/{}", path),
            );
            let robot = Arc::new(RwLock::new(robot));
            (
                robot.clone(),
                create_nodes::<franka_emika::FrankaEmika, { franka_emika::EMIKA_DOF }>(
                    config, path, robot,
                ),
            )
        }
        "franka_search3" => {
            let robot = franka_research3::FrankaResearch3::new_research3(
                config.name.clone(),
                format!("/robot/{}", path),
            );
            let robot = Arc::new(RwLock::new(robot));
            (
                robot.clone(),
                create_nodes::<
                    franka_research3::FrankaResearch3,
                    { franka_research3::RESEARCH3_DOF },
                >(config, path, robot),
            )
        }
        _ => panic!("Unknown robot type"),
    }
}

pub fn get_planner_with_name(
    planner: &Arc<Mutex<dyn planner::Planner>>,
    name: &str,
) -> Option<Arc<Mutex<dyn planner::Planner>>> {
    // !从 planner 树中根据 name 获取 planner,辅助函数
    if planner.lock().unwrap().get_name() == name {
        return Some(planner.clone());
    }

    for child in planner.lock().unwrap().get_planner().iter() {
        if child.lock().unwrap().get_name() == name {
            return Some(child.clone());
        }
    }

    None
}

pub fn get_controller_with_name(
    controller: &Arc<Mutex<dyn controller::Controller>>,
    name: &str,
) -> Option<Arc<Mutex<dyn controller::Controller>>> {
    // !从 controller 树中根据 name 获取 controller,辅助函数
    if controller.lock().unwrap().get_name() == name {
        return Some(controller.clone());
    }

    for child in controller.lock().unwrap().get_controller().iter() {
        if child.lock().unwrap().get_name() == name {
            return Some(child.clone());
        }
    }

    None
}

pub fn get_simulator_with_name(
    simulator: &Arc<Mutex<dyn simulator::Simulator>>,
    name: &str,
) -> Option<Arc<Mutex<dyn simulator::Simulator>>> {
    // !从 simulator 树中根据 name 获取 simulator,辅助函数
    if simulator.lock().unwrap().get_name() == name {
        return Some(simulator.clone());
    }

    for child in simulator.lock().unwrap().get_simulator().iter() {
        if child.lock().unwrap().get_name() == name {
            return Some(child.clone());
        }
    }

    None
}

pub fn get_sensor_with_name(
    sensor_list: &Vec<Arc<RwLock<Sensor>>>,
    name: &str,
) -> Option<Arc<RwLock<Sensor>>> {
    // !从 sensor_list 中根据 name 获取 sensor,辅助函数
    for sensor in sensor_list {
        if sensor.read().unwrap().get_name() == name {
            return Some(sensor.clone());
        }
    }
    None
}
