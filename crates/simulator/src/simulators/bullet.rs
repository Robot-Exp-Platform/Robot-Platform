use crossbeam::queue::SegQueue;
#[cfg(feature = "ros")]
use rosrust as ros;
use serde_json::Value;
// use serde_yaml::Value;
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
#[cfg(feature = "rszmq")]
use std::sync::Mutex;
use std::sync::{Arc, RwLock};
#[cfg(feature = "rszmq")]
use zmq;

use crate::simulator_trait::Simulator;
use message::control_command::ControlCommand;
use recoder::*;
use robot::robot_trait::Robot;
use task_manager::ros_thread::ROSThread;

// bullet 结构体声明，包含其名称，路径，消息节点，以及机器人
#[allow(dead_code)]
pub struct Bullet<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    msgnode: BulletNode,
    robot: Arc<RwLock<R>>,
}

// 消息节点结构体声明，随条件编译的不同而不同，条件编译将决定其使用什么通讯方式
#[allow(dead_code)]
struct BulletNode {
    recoder: BufWriter<File>,
    control_command_queue: Arc<SegQueue<ControlCommand>>,
    #[cfg(feature = "rszmq")]
    context: Arc<zmq::Context>,
    #[cfg(feature = "rszmq")]
    responder: Arc<Mutex<zmq::Socket>>,
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

// 为结构体 Bullet 实现方法，这里主要是初始化方法
impl<R: Robot + 'static, const N: usize> Bullet<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet::from_params(name, path, format!("bullet"), robot)
    }
    pub fn from_params(
        name: String,
        path: String,
        file_path: String,
        robot: Arc<RwLock<R>>,
    ) -> Bullet<R, N> {
        #[cfg(feature = "rszmq")]
        let context = Arc::new(zmq::Context::new());
        #[cfg(feature = "rszmq")]
        let responder = context.socket(zmq::REP).unwrap();

        let file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(format!("{}/{}.txt", file_path, name))
            .unwrap();

        Bullet {
            name,
            path,
            msgnode: BulletNode {
                recoder: BufWriter::new(file),
                control_command_queue: Arc::new(SegQueue::new()),
                #[cfg(feature = "rszmq")]
                context: Arc::new(zmq::Context::new()),
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
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn set_params(&mut self, _: Value) {}
    fn set_controller_command_queue(
        &mut self,
        controller_command_queue: Arc<SegQueue<ControlCommand>>,
    ) {
        self.msgnode.control_command_queue = controller_command_queue;
    }

    fn add_simulator(&mut self, _: Arc<std::sync::Mutex<dyn Simulator>>) {}
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
        let file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(format!(
                "data/{}/{}/{}/{}.txt",
                *EXP_NAME,
                *TASK_NAME.lock().unwrap(),
                self.robot.read().unwrap().get_name(),
                self.get_name()
            ))
            .unwrap();
        self.msgnode.recoder = BufWriter::new(file);
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
            // 定义一个可选的消息变量，用于在作用域外存储消息
            let message_opt: Option<String>;
            // 获取 robot 状态
            {
                let responder = self.msgnode.responder.lock().unwrap();
                match responder.recv_string(0) {
                    Ok(Ok(message)) => {
                        // 成功接收到消息，并且消息是一个有效的 UTF-8 字符串
                        println!("Received message!");
                        message_opt = Some(message); // 将消息存储到外部变量中
                    }
                    Ok(Err(_)) => {
                        eprintln!("Received a message that is not a valid UTF-8 string.");
                        message_opt = None; // 未接收到有效消息
                    }
                    Err(e) => {
                        eprintln!("Failed to receive message: {}", e);
                        message_opt = None; // 未接收到消息
                    }
                }
                // 此时 responder 锁已被释放
            } // responder 锁在这里被自动释放
            if let Some(message) = message_opt {
                // 如果成功接收到消息，则进行处理
                // 将 robot state 反序列化为 State 消息类型，并写入 robot 中去
                let robot_state: Vec<f64> = serde_json::from_str(&message).unwrap();
                {
                    let mut robot_write = self.robot.write().unwrap();
                    robot_write.set_q(robot_state[0..N].to_vec());
                    robot_write.set_q_dot(robot_state[N..2 * N].to_vec());
                }
                // 向仿真器发送控制指令
                // 将指令序列化为 JSON 字符串
                let reply = serde_json::to_string(&(_period, _control_command))
                    .expect("Failed to serialize tuple");
                // 重新获取锁并发送回复
                let responder = self.msgnode.responder.lock().unwrap();
                responder.send(&reply, 0).expect("Failed to send reply");
                println!("Sent message!");
            }
        }
    }

    fn get_period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(0.5)
    }
}
