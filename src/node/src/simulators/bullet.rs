use nalgebra as na;
use robot::{DSeriseRobot, Robot, RobotType};
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::Value;
use crossbeam::queue::SegQueue;
use std::process::Command;
#[cfg(feature = "rszmq")]
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
use std::thread;
use tracing::info;
#[cfg(feature = "rszmq")]
use zmq;

use crate::{Node, NodeBehavior, NodeState};
use generate_tools::*;
use message::RobotState;
use sensor::Sensor;

// bullet 结构体声明，包含其名称，路径，消息节点，以及机器人
#[allow(dead_code)]
pub struct Bullet<R> {
    /// The name of the simulator.
    name: String,
    /// The state of the simulator.
    state: BulletState,
    /// The parameters of the simulator.
    params: BulletParams,
    /// The path of the simulator.
    node: BulletNode,
    /// The robot that the simulator is controlling.
    robot: Vec<Arc<RwLock<R>>>,
    /// The sensor that the simulator is using.
    sensor: Vec<Arc<RwLock<Sensor>>>,
}

pub type DBullet = Bullet<DSeriseRobot>;
pub type SBullet<const N: usize> = Bullet<RobotType>;

struct BulletState {
    is_end: bool,
    node_state: NodeState,
    pybullet_thread: Option<thread::JoinHandle<()>>,
}

#[derive(Deserialize)]
pub struct BulletParams {
    period: f64,
    config_path: String,
}

// 消息节点结构体声明，随条件编译的不同而不同，条件编译将决定其使用什么通讯方式
struct BulletNode {
    #[cfg(feature = "rszmq")]
    responder: Arc<Mutex<zmq::Socket>>,
}

// 为结构体 Bullet 实现方法，这里主要是初始化方法
impl DBullet {
    pub fn new(name: String) -> DBullet {
        DBullet::from_params(
            name,
            BulletParams {
                period: 0.0,
                config_path: String::new(),
            },
        )
    }

    pub fn from_json(name: String, params: Value) -> DBullet {
        DBullet::from_params(name, serde_json::from_value(params).unwrap())
    }

    pub fn from_params(name: String, params: BulletParams) -> DBullet {
        #[cfg(feature = "rszmq")]
        let context = Arc::new(zmq::Context::new());
        #[cfg(feature = "rszmq")]
        let responder = context.socket(zmq::REP).unwrap();

        DBullet {
            name,
            state: BulletState {
                is_end: false,
                node_state: NodeState::RelyRelease,
                pybullet_thread: None,
            },
            params,
            node: BulletNode {
                #[cfg(feature = "rszmq")]
                responder: Arc::new(Mutex::new(responder)),
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot: Vec::new(),
            sensor: Vec::new(),
        }
    }
}

impl Node<na::DVector<f64>> for DBullet {
    get_fn!((name: String));

    fn set_input_queue(&mut self, _: Arc<SegQueue<message::NodeMessage<na::DVector<f64>>>>) {}
    fn set_output_queue(&mut self, _: Arc<SegQueue<message::NodeMessage<na::DVector<f64>>>>) {}

    fn is_end(&mut self) {
        self.state.is_end = true;
    }
    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot.push(robot);
        }
    }
    fn set_sensor(&mut self, sensor: Arc<RwLock<Sensor>>) {
        self.sensor.push(sensor);
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
}

impl NodeBehavior for DBullet {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
        // 使用命令行启动 pybullet， 告知 pybullet 所有的机器人信息
        // 暂时使用命令行加载的形式，后续也可以使用文件加载

        let config_path = self.params.config_path.clone();

        self.state.pybullet_thread = Some(std::thread::spawn(move || {
            let output = Command::new("python")
                .arg("./scripts/simulators/sim_pybullet.py")
                .arg("-f")
                .arg(config_path)
                .spawn()
                .expect("Failed to execute command")
                .wait_with_output()
                .expect("Failed to wait on child process");

            if !output.status.success() {
                eprintln!(
                    "Command failed with error: {}",
                    String::from_utf8_lossy(&output.stderr)
                );
            } else {
                println!(
                    "Command executed successfully: {}",
                    String::from_utf8_lossy(&output.stdout)
                );
            }
        }));

        // 建立通讯
        #[cfg(feature = "rszmq")]
        {
            // 使用zmq实现程序通信，通信协议暂定为TCP
            // 以下为responder端
            // 使用锁来访问 responder
            let responder = self.node.responder.clone(); // 克隆 Arc 引用
            let responder_lock = responder.lock().unwrap(); // 锁定 Mutex 并解锁获得可变引用
                                                            // 绑定到TCP地址
            match responder_lock.bind("tcp://*:5555") {
                Ok(_) => println!("Socket successfully bound to tcp://*:5555"),
                Err(e) => eprintln!("Failed to bind socket: {}", e),
            }
        }

        // 初始化节点状态
        self.state.node_state = NodeState::Init;
    }

    fn update(&mut self) {
        // 读取所有机器人中保存的控制信息
        let mut commands = Vec::new();
        for robot in self.robot.iter() {
            let robot_read = robot.read().unwrap();
            let command = robot_read.control_message();
            commands.push(command);
        }

        let mut collections_info = Vec::new();
        for sensor in self.sensor.iter() {
            let sensor_read = sensor.read().unwrap();
            let mut collection = sensor_read.collision();
            collections_info.append(&mut collection);
        }

        // 整理控制指令为字符串
        let reply = format!(
            "{{\"command\": {}, \"obstacles\": {}}}",
            serde_json::to_string(&commands).unwrap(),
            serde_json::to_string(&collections_info).unwrap()
        );
        #[cfg(feature = "rszmq")]
        {
            // 获取 responder 并接受 RobotState 消息
            let responder = self.node.responder.lock().unwrap();
            let message = responder
                .recv_string(0)
                .expect("Received a message that is not a valid UTF-8 string.")
                .unwrap();
            let robot_state: Vec<RobotState> = serde_json::from_str(message.as_str()).unwrap();

            // 及时返回控制指令

            responder.send(&reply, 0).expect("Failed to send reply");

            // 处理消息，将消息中的状态信息写入到机器人状态中
            for (robot, state) in self.robot.iter().zip(robot_state.iter()) {
                let mut robot_write = robot.write().unwrap();
                match state {
                    RobotState::Joint(joint) => {
                        info!(node = self.name.as_str(), input = ?joint.as_slice());
                        robot_write.set_q(na::DVector::from_vec(joint.clone()))
                    }
                    RobotState::Velocity(velocity) => {
                        robot_write.set_q_dot(na::DVector::from_vec(velocity.clone()))
                    }
                    RobotState::Acceleration(acceleration) => {
                        robot_write.set_q_ddot(na::DVector::from_vec(acceleration.clone()))
                    }
                    RobotState::JointVel(joint, velocity) => {
                        info!(node = self.name.as_str(), input = ?joint.as_slice());
                        robot_write.set_q(na::DVector::from_vec(joint.clone()));
                        robot_write.set_q_dot(na::DVector::from_vec(velocity.clone()));
                    }
                    RobotState::JointVelAcc(joint, velocity, acceleration) => {
                        robot_write.set_q(na::DVector::from_vec(joint.clone()));
                        robot_write.set_q_dot(na::DVector::from_vec(velocity.clone()));
                        robot_write.set_q_ddot(na::DVector::from_vec(acceleration.clone()));
                    }
                    _ => (),
                }
                drop(robot_write);
            }
        }

        // 修改节点状态
        if self.state.node_state == NodeState::Init {
            self.state.node_state = NodeState::RelyRelease;
        } else {
            self.state.node_state = NodeState::Running;
        }
    }

    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }

    fn state(&mut self) -> crate::NodeState {
        self.state.node_state
    }
}
