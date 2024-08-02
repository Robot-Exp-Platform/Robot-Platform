use crate::simulator_trait::Simulator;
use robot::robot_trait::Robot;
#[cfg(feature = "ros")]
use rosrust as ros;
use std::sync::{Arc, RwLock};
use task_manager::ros_thread::ROSThread;
#[cfg(feature = "rszmq")]
use zmq;

#[allow(dead_code)]
pub struct Bullet<R: Robot + 'static, const N: usize> {
    name: String,
    path: String,

    msgnode: BulletNode,
    robot: Arc<RwLock<R>>,
}

#[allow(dead_code)]
struct BulletNode {
    // #[cfg(feature = "zmq")]
    // pub sub_list: Vec<zmq::Socket>,
    #[cfg(feature = "ros")]
    sub_list: Vec<ros::Subscriber>,
}

impl<R: Robot + 'static, const N: usize> Bullet<R, N> {
    pub fn new(name: String, path: String, robot: Arc<RwLock<R>>) -> Bullet<R, N> {
        Bullet {
            name,
            path,
            msgnode: BulletNode {
                // #[cfg(feature = "zmq")]
                // sub_list: Vec::new(),
                #[cfg(feature = "ros")]
                sub_list: Vec::new(),
            },
            robot,
        }
    }
}

impl<R: Robot + 'static, const N: usize> Simulator for Bullet<R, N> {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }

    fn add_simulator(&mut self, _: Arc<std::sync::Mutex<dyn Simulator>>) {}
}

impl<R: Robot + 'static, const N: usize> ROSThread for Bullet<R, N> {
    fn init(&mut self) {
        #[cfg(feature = "zmq")]
        {
            // ! 使用 zmq 写的订阅者通讯
            // ! zmq 不容许在多个线程中使用同一个 Socket ,虽然我们只会在一个线程中调用他,但是尚且没有想到规避的办法,暂且取悦编译器,可能会导致阻塞！！！
            // TODO 找到绕开的办法了,但是需要 unsafe 代码,还没搓好,unsafe impl Send for Context {},unsafe impl Sync for Context {}
            // 在这里进行订阅者的声明
            let context = zmq::Context::new();
            let subscriber = context.socket(zmq::SUB).unwrap();
            // TODO ipc 通讯需要一个实际存在的文件路径,当前标记方案无法保证其唯一性和存在性
            assert!(subscriber
                .connect(&format!("ipc:///tmp/{}", self.name))
                .is_ok());
            // 设置过滤器并启动,所有的订阅者都需要set,当没有其他参数时表明过滤功能
            assert!(subscriber.set_subscribe("".as_bytes()).is_ok());

            loop {
                let msg = subscriber.recv_string(0).unwrap().unwrap();
                let robot_state: Vec<f64> = msg
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
