use crossbeam::queue::SegQueue;
#[cfg(feature = "ros")]
use rosrust as ros;
use std::sync::{Arc, RwLock};
#[cfg(feature = "rszmq")]
use zmq;

use crate::simulator_trait::Simulator;
use message::control_command::ControlCommand;
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
    control_command_queue: Arc<SegQueue<ControlCommand>>,
    // #[cfg(feature = "zmq")]
    // pub sub_list: Vec<zmq::Socket>,
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

// 为结构体 Bullet 实现方法，这里主要是初始化方法
impl<R: Robot + 'static, const N: usize> Bullet<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet {
            name,
            path,
            msgnode: BulletNode {
                control_command_queue: Arc::new(SegQueue::new()),
                // #[cfg(feature = "zmq")]
                // sub_list: Vec::new(),
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot,
        }
    }
    pub fn new_without_params(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet::new(name, path, robot)
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

    fn set_params(&mut self, _: String) {}
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
            let context = zmq::Context::new();
            let responder = context.socket(zmq::REP).unwrap();
            // 绑定到TCP地址
            responder
                .bind("tcp://*:5555")
                .expect("Failed to bind socket");

            loop {
                let message = responder
                    .recv_string(0)
                    .expect("Failed to receive message")
                    .unwrap();
                let robot_state: Vec<f64> = message
                    .split(' ')
                    .map(|s| s.parse::<f64>().unwrap()) // 将每个拆分的部分解析为 f64
                    .collect();
                self.robot
                    .write()
                    .unwrap()
                    .set_q(robot_state[0..N].to_vec());
                self.robot
                    .write()
                    .unwrap()
                    .set_q_dot(robot_state[N..2 * N].to_vec());
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

    fn start(&mut self) {}

    fn update(&mut self) {}
}
