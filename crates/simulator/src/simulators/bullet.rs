#[cfg(feature = "ros")]
use rosrust as ros;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::Value;
use crossbeam::queue::SegQueue;
use std::fs;
use std::io::{BufWriter, Write};
#[cfg(feature = "rszmq")]
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
#[cfg(feature = "rszmq")]
use zmq;

use crate::{Simulator, SimulatorN};
#[cfg(feature = "rszmq")]
use message::{ControlCommandN, RobotStateN};
#[cfg(feature = "recode")]
use recoder::*;
use robot::SeriesRobot;
use robot_macros_derive::*;
use sensor::Sensor;
use task_manager::ROSThread;

// bullet 结构体声明，包含其名称，路径，消息节点，以及机器人
#[allow(dead_code)]
pub struct Bullet<R: SeriesRobot<N>, const N: usize> {
    name: String,
    path: String,

    params: BulletParams,

    node: BulletNode<N>,
    robot: Arc<RwLock<R>>,
}

#[derive(Deserialize)]
pub struct BulletParams {
    period: f64,
}

// 消息节点结构体声明，随条件编译的不同而不同，条件编译将决定其使用什么通讯方式
struct BulletNode<const N: usize> {
    sensor: Option<Arc<RwLock<Sensor>>>,
    recoder: Option<BufWriter<fs::File>>,
    control_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
    #[cfg(feature = "rszmq")]
    responder: Arc<Mutex<zmq::Socket>>,
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

// 为结构体 Bullet 实现方法，这里主要是初始化方法
impl<R: SeriesRobot<N>, const N: usize> Bullet<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet::from_params(name, path, BulletParams { period: 0.0 }, robot)
    }
    pub fn from_params(
        name: String,
        path: String,
        params: BulletParams,
        robot: Arc<RwLock<R>>,
    ) -> Bullet<R, N> {
        #[cfg(feature = "rszmq")]
        let context = Arc::new(zmq::Context::new());
        #[cfg(feature = "rszmq")]
        let responder = context.socket(zmq::REP).unwrap();

        Bullet {
            name,
            path,
            params,
            node: BulletNode {
                sensor: None,
                recoder: None,
                control_command_queue: Arc::new(SegQueue::new()),
                #[cfg(feature = "rszmq")]
                responder: Arc::new(Mutex::new(responder)),
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot,
        }
    }
}

impl<R: SeriesRobot<N>, const N: usize> SimulatorN<N> for Bullet<R, N> {
    fn set_controller_command_queue(
        &mut self,
        control_command_queue: Arc<SegQueue<ControlCommandN<N>>>,
    ) {
        self.node.control_command_queue = control_command_queue;
    }
}

// 为 Bullet 实现 Simulator 特征，使得其是一个仿真器
impl<R: SeriesRobot<N>, const N: usize> Simulator for Bullet<R, N> {
    generate_simulator_method!();
}

// 为 Bullet 实现 ROSThread 特征，使得其可以被类似 ros 的线程管理器调用，而实际上并不一定用到了ros，只是结构相似罢了
impl<R: SeriesRobot<N>, const N: usize> ROSThread for Bullet<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
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
        #[cfg(feature = "ros")]
        {
            // ! 使用 ros 写的订阅者通讯
            // 在这里进行节点和话题的声明
            let robot = self.robot.clone();
            self.node.sub_list.push(
                ros::subscribe(
                    (self.path.clone() + self.name.clone().as_str()).as_str(),
                    1,
                    move |msg: rosrust_msg::msg_package::RobotState| {
                        robot.write().unwrap().set_q(msg.q.clone());
                        robot.write().unwrap().set_q_dot(msg.q_dot.clone());
                    },
                )
                .unwrap(),
            );

            // 新建接收者，并将他们放入list中去
        }
    }

    fn start(&mut self) {
        #[cfg(feature = "recode")]
        {
            fs::create_dir_all(format!(
                "./data/{}/{}/{}",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name()
            ))
            .unwrap();
            let file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(format!(
                    "data/{}/{}/{}/bullet.txt",
                    *EXP_NAME,
                    *TASK_NAME.lock().unwrap(),
                    self.robot.read().unwrap().get_name(),
                ))
                .unwrap();
            self.node.recoder = Some(BufWriter::new(file));
        }
    }

    fn update(&mut self) {
        // 更新 control command

        // TODO: 这里的逻辑有问题，向仿真器发送的消息应该是整个 message， 不仅仅包括了控制指令的类型，也包括了控制指令的具体内容，目前的写法中只包含了内容，而无法区分到底是什么类型的控制指令。建议将 control_command 直接序列化后发送到仿真器中。

        // let (_period, _control_command) = match self.node.control_command_queue.pop() {
        //     Some(ControlCommand::Joint(joint)) => (0.0, joint),
        //     Some(ControlCommand::JointWithPeriod(period, joint)) => (period, joint),
        //     Some(ControlCommand::Tau(tau)) => (0.0, tau),
        //     Some(ControlCommand::TauWithPeriod(period, tau)) => (period, tau),
        //     None => {
        //         eprintln!("Failed to pop control command from queue.");
        //         return;
        //     }
        // };

        let command = self.node.control_command_queue.pop();
        if command.is_none() {
            eprintln!("Failed to pop control command from queue.");
            return;
        }

        #[cfg(feature = "rszmq")]
        {
            // 获取 responder 并接受 RobotState 消息
            let responder = self.node.responder.lock().unwrap();
            let message = responder
                .recv_string(0)
                .expect("Received a message that is not a valid UTF-8 string.")
                .unwrap();
            let robot_state: RobotStateN<N> = serde_json::from_str(message.as_str()).unwrap();

            // 及时返回控制指令
            let reply = serde_json::to_string(&(command)).unwrap();
            responder.send(&reply, 0).expect("Failed to send reply");

            // 处理消息，将消息中的状态信息写入到机器人状态中
            let mut robot_write = self.robot.write().unwrap();
            match robot_state {
                RobotStateN::Joint(joint) => robot_write.set_q(joint),
                RobotStateN::Velocity(velocity) => robot_write.set_q_dot(velocity),
                RobotStateN::JointVel(joint, velocity) => {
                    robot_write.set_q(joint);
                    robot_write.set_q_dot(velocity);
                }
                _ => {}
            }
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.node.recoder {
            recoder.flush().unwrap();
        }
    }

    fn get_period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
}
