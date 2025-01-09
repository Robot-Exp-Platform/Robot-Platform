use nalgebra as na;
use robot::{DSeriseRobot, Robot, RobotLock, RobotType};
use serde::Deserialize;
use std::sync::Arc;
use std::thread;
use std::{process::Command, sync::Mutex};
use tracing::info;
use zmq;

use crate::{Node, NodeBehavior, NodeState};
use message::RobotState;

pub type Bullet<R> = Node<BulletState, BulletParams, RobotLock<R>, na::DVector<f64>>;

pub type DBullet = Bullet<DSeriseRobot>;
pub type SBullet<const N: usize> = Bullet<RobotType>;

#[derive(Default)]
pub struct BulletState {
    pybullet_thread: Option<thread::JoinHandle<()>>,
    responder: Option<Arc<Mutex<zmq::Socket>>>,
}

#[derive(Deserialize)]
pub struct BulletParams {
    period: f64,
    config_path: String,
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
        {
            // 使用zmq实现程序通信，通信协议暂定为TCP
            // 以下为responder端
            let context = Arc::new(zmq::Context::new());
            let responder = context.socket(zmq::REP).unwrap();

            match responder.bind("tcp://*:5555") {
                Ok(_) => println!("Socket successfully bound to tcp://*:5555"),
                Err(e) => eprintln!("Failed to bind socket: {}", e),
            }
            self.state.responder = Some(Arc::new(Mutex::new(responder)));
        }

        // 初始化节点状态
        self.node_state = NodeState::Init;
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
        {
            // 获取 responder 并接受 RobotState 消息
            let responder = self.state.responder.as_ref().unwrap().lock().unwrap();
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
        if self.node_state == NodeState::Init {
            self.node_state = NodeState::RelyRelease;
        } else {
            self.node_state = NodeState::Running;
        }
    }

    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }

    fn state(&mut self) -> crate::NodeState {
        self.node_state
    }
}
