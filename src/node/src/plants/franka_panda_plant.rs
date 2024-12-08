use franka;
use nalgebra as na;
use sensor::Sensor;
use serde::Deserialize;
use serde_json::{from_value, Value};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_4},
    sync::{Arc, RwLock},
    thread,
    time::Duration,
};

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue, NodeMessageQueue};
use robot::{DPanda, Panda, RobotType};

pub struct PandaPlant<V> {
    name: String,
    state: PandaPlantState,
    params: PandaPlantParams,
    node: PandaPlantNode<V>,
    robot: Option<Arc<RwLock<Panda<V>>>>,
}

pub type DPandaPlant = PandaPlant<na::DVector<f64>>;

#[derive(Default)]
pub struct PandaPlantState {
    robot: Option<franka::Robot>,
}

#[derive(Deserialize, Default)]
pub struct PandaPlantParams {
    period: f64,
    ip: String,
    control_mode: String,
    is_realtime: bool,
}

#[derive(Default)]
struct PandaPlantNode<V> {
    input_queue: NodeMessageQueue<V>,
    output_queue: NodeMessageQueue<V>,
}

impl DPandaPlant {
    pub fn from_json(name: String, json: Value) -> DPandaPlant {
        DPandaPlant::from_params(name, from_value(json).unwrap())
    }
    pub fn from_params(name: String, params: PandaPlantParams) -> DPandaPlant {
        DPandaPlant {
            name,
            state: PandaPlantState::default(),
            params,
            node: PandaPlantNode::default(),
            robot: None,
        }
    }
}

impl Node<na::DVector<f64>> for DPandaPlant {
    get_fn!((name: String));
    set_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));
    fn set_robot(&mut self, robot: RobotType) {
        if let RobotType::DSeriseRobot(robot) = robot {
            self.robot = Some(robot);
        }
    }
    fn set_params(&mut self, params: Value) {
        self.params = from_value(params).unwrap();
    }
    fn is_end(&mut self) {}
    fn set_sensor(&mut self, _sensor: Arc<RwLock<Sensor>>) {}
}

impl NodeBehavior for DPandaPlant {
    fn start(&mut self) {
        // 连接机器人及机械爪，清空错误
        let mut robot = franka::Robot::new(&self.params.ip, None, None).unwrap();
        let model = robot.load_model(true).unwrap();
        robot.set_default_behavior().unwrap();
        robot.automatic_error_recovery().unwrap();

        // 初始化机器人位置
        let q_initial =
            na::DVector::from_vec(vec![0., -FRAC_PI_4, 0., -2.3562, 0., FRAC_PI_2, FRAC_PI_4]);
        let q_goal: [f64; 7] = q_initial.as_slice().try_into().unwrap();
        robot.joint_motion(0.5, &q_goal).unwrap();

        // 实体机器人在输入非事实指令之后需要等待直到非实时指令完成，否则会出现错误
        loop {
            let state = robot.read_once().unwrap();
            if state.dq.iter().copied().sum::<f64>() < 0.01_f64 {
                break;
            }
            thread::sleep(Duration::from_millis(1));
            println!("we are waiting for the robot to stop");
        }

        if self.params.is_realtime {
            // 四种运动生成器
            let callback_joint =
                |state: &franka::RobotState, _: &Duration| -> franka::JointPositions {
                    update_state(self.robot.as_ref().unwrap(), state);
                    let out_q: [f64; 7] =
                        if let Some(DNodeMessage::Joint(joint)) = self.node.input_queue.pop() {
                            joint.as_slice().try_into().unwrap()
                        } else {
                            state.q_d
                        };
                    franka::JointPositions::new(out_q)
                };
            let callback_joint_vel =
                |state: &franka::RobotState, _: &Duration| -> franka::JointVelocities {
                    update_state(self.robot.as_ref().unwrap(), state);
                    let out_q_dot: [f64; 7] =
                        if let Some(DNodeMessage::JointVel(_, vel)) = self.node.input_queue.pop() {
                            vel.as_slice().try_into().unwrap()
                        } else {
                            state.dq
                        };
                    franka::JointVelocities::new(out_q_dot)
                };
            let callback_cartesian =
                |state: &franka::RobotState, _: &Duration| -> franka::CartesianPose {
                    update_state(self.robot.as_ref().unwrap(), state);
                    let out_pose: [f64; 16] =
                        if let Some(DNodeMessage::Pose(pose)) = self.node.input_queue.pop() {
                            pose.to_homogeneous().as_slice().try_into().unwrap()
                        } else {
                            state.O_T_EE_c
                        };
                    franka::CartesianPose::new(out_pose, None)
                };
            let callback_cartesian_vel =
                |state: &franka::RobotState, _: &Duration| -> franka::CartesianVelocities {
                    update_state(self.robot.as_ref().unwrap(), state);
                    let out_pose_dot: [f64; 6] = state.O_dP_EE_c;
                    franka::CartesianVelocities::new(out_pose_dot, None)
                };

            // 力矩控制闭包
            let callback_torque = |state: &franka::RobotState, _: &Duration| -> franka::Torques {
                update_state(self.robot.as_ref().unwrap(), state);
                let coriolis: franka::Vector7 = model.coriolis_from_state(state).into();
                let out_tau = if let DNodeMessage::Tau(tau) = self.node.input_queue.pop().unwrap() {
                    tau
                } else {
                    na::DVector::zeros(7)
                };

                let out_tau = franka::Vector7::from_row_slice(out_tau.as_slice());
                (out_tau + coriolis).into()
            };

            match self.params.control_mode.as_str() {
                "joint" => robot.control_joint_positions(callback_joint, None, None, None),
                "joint_vel" => robot.control_joint_velocities(callback_joint_vel, None, None, None),
                "cartesian" => robot.control_cartesian_pose(callback_cartesian, None, None, None),
                "cartesian_vel" => {
                    robot.control_cartesian_velocities(callback_cartesian_vel, None, None, None)
                }
                "torque" => robot.control_torques(callback_torque, None, None),
                _ => panic!("Unsupported control mode"),
            }
            .unwrap();
        }

        self.state.robot = Some(robot);
    }

    fn update(&mut self) {
        let panda = self.state.robot.as_mut().unwrap();
        let state = panda.read_once().unwrap();
        update_state(self.robot.as_ref().unwrap(), &state);

        if !self.params.is_realtime {
            if let Some(DNodeMessage::Joint(joint)) = self.node.input_queue.pop() {
                let out_q = joint.as_slice().try_into().unwrap();
                panda.joint_motion(0.5, &out_q).unwrap();
            }
        }
    }
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}

fn update_state(robot: &Arc<RwLock<DPanda>>, state: &franka::RobotState) {
    let mut robot = robot.write().unwrap();
    robot.state.q = na::DVector::from_row_slice(state.q.as_slice());
    robot.state.q_dot = na::DVector::from_row_slice(state.dq.as_slice());
}
