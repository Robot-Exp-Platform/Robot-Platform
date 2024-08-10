use crossbeam::queue::SegQueue;
#[cfg(feature = "ros")]
use rosrust as ros;
use serde::Deserialize;
use serde_json::{from_value, Value};
// use serde_yaml::Value;
use std::fs;
use std::io::{BufWriter, Write};
#[cfg(feature = "rszmq")]
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
#[cfg(feature = "rszmq")]
use zmq;

use crate::simulator_trait::Simulator;
use message::control_command::ControlCommand;
#[cfg(feature = "rszmq")]
use message::state::RobotState;
#[cfg(feature = "recode")]
use recoder::*;
use robot::robot_trait::Robot;
use task_manager::generate_node_method;
use task_manager::ros_thread::ROSThread;

// bullet 结构体声明，包含其名称，路径，消息节点，以及机器人
#[allow(dead_code)]
pub struct Bullet<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    params: BulletParams,

    msgnode: BulletNode,
    robot: Arc<RwLock<R>>,
}

#[derive(Deserialize)]
pub struct BulletParams {
    period: f64,
}

// 消息节点结构体声明，随条件编译的不同而不同，条件编译将决定其使用什么通讯方式
struct BulletNode {
    recoder: Option<BufWriter<fs::File>>,
    control_command_queue: Arc<SegQueue<ControlCommand>>,
    #[cfg(feature = "rszmq")]
    responder: Arc<Mutex<zmq::Socket>>,
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

// 为结构体 Bullet 实现方法，这里主要是初始化方法
impl<R: Robot + 'static, const N: usize> Bullet<R, N> {
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
            msgnode: BulletNode {
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

// 为 Bullet 实现 Simulator 特征，使得其是一个仿真器
impl<R: Robot + 'static, const N: usize> Simulator for Bullet<R, N> {
    generate_node_method!();

    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    ) {
        self.msgnode.control_command_queue = controller_command_queue;
    }
}

// 为 Bullet 实现 ROSThread 特征，使得其可以被类似 ros 的线程管理器调用，而实际上并不一定用到了ros，只是结构相似罢了
impl<R: Robot + 'static, const N: usize> ROSThread for Bullet<R, N> {
    fn init(&mut self) {
        println!("{} 向您问好. {} says hello.", self.name, self.name);
        #[cfg(feature = "rszmq")]
        {
            // 使用zmq实现程序通信，通信协议暂定为TCP
            // 以下为responder端
            // 使用锁来访问 responder
            let responder = self.msgnode.responder.clone(); // 克隆 Arc 引用
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
            self.msgnode.sub_list.push(
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
            self.msgnode.recoder = Some(BufWriter::new(file));
        }
    }

    fn update(&mut self) {
        // 更新 control command

        let (_period, _control_command) = match self.msgnode.control_command_queue.pop() {
            Some(ControlCommand::Joint(joint)) => (0.0, joint),
            Some(ControlCommand::JointWithPeriod(joint_with_period)) => {
                (joint_with_period.period, joint_with_period.joint)
            }
            None => {
                eprintln!("Failed to pop control command from queue.");
                return;
            }
        };
        println!("{} get control command: {:?}", self.name, _control_command);

        #[cfg(feature = "rszmq")]
        {
            // 获取 responder 并接受 RobotState 消息
            let responder = self.msgnode.responder.lock().unwrap();
            let message = responder
                .recv_string(0)
                .expect("Received a message that is not a valid UTF-8 string.")
                .unwrap();
            let robot_state: RobotState = serde_json::from_str(message.as_str()).unwrap();

            // 及时返回控制指令
            let reply = serde_json::to_string(&(_period, _control_command)).unwrap();
            responder.send(&reply, 0).expect("Failed to send reply");

            // 处理消息，将消息中的状态信息写入到机器人状态中
            let mut robot_write = self.robot.write().unwrap();
            match robot_state {
                RobotState::Joint(joint) => robot_write.set_q(joint),
                RobotState::Velocity(velocity) => robot_write.set_q_dot(velocity),
                RobotState::JointVelocity(joint, velocity) => {
                    robot_write.set_q(joint);
                    robot_write.set_q_dot(velocity);
                }
                _ => {}
            }
        }
    }

    fn finalize(&mut self) {
        if let Some(ref mut recoder) = self.msgnode.recoder {
            recoder.flush().unwrap();
        }
    }

    fn get_period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
}
