use kernel_macro::node_registration;
use nalgebra as na;
use serde::Deserialize;

use crate::{Node, NodeBehavior, NodeExtBehavior, NodeRegister};
use robot::{DSeriseRobot, RobotLock};

#[cfg(unix)]
use message::DNodeMessage;
#[cfg(unix)]
use robot::DPanda;
#[cfg(unix)]
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_4},
    thread,
    time::Duration,
};

pub type PandaPlant<V> = Node<PandaPlantState, PandaPlantParams, RobotLock<DSeriseRobot>, V>;
#[node_registration("panda_plant")]
pub type DPandaPlant = PandaPlant<na::DVector<f64>>;

#[derive(Default)]
pub struct PandaPlantState {
    #[cfg(unix)]
    robot: Option<franka::Robot>,
}

#[allow(dead_code)]
#[derive(Deserialize, Default)]
pub struct PandaPlantParams {
    period: f64,
    ip: String,
    control_mode: String,
    is_realtime: bool,
}

impl NodeBehavior for DPandaPlant {
    #[cfg(unix)]
    fn start(&mut self) {
        // 连接机器人及机械爪，清空错误
        let mut robot = franka::Robot::new(&self.params.ip, None, None).unwrap();
        let model = robot.load_model(true).unwrap();
        robot.set_default_behavior().unwrap();
        robot.automatic_error_recovery().unwrap();
        // 设置碰撞行为和阻抗参数
        robot
            .set_collision_behavior(
                [100.; 7], [100.; 7], [100.; 7], [100.; 7], [100.; 6], [100.; 6], [100.; 6],
                [100.; 6],
            )
            .unwrap();
        robot
            .set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.])
            .unwrap();
        robot
            .set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.])
            .unwrap();

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
        }

        println!("初始化机器人位置成功");

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
                let gravity: franka::Vector7 = model.gravity_from_state(state, None).into();

                let currect_command = self.node.input_queue.pop();
                let out_tau = if let Some(DNodeMessage::Tau(tau)) = currect_command {
                    tau
                } else {
                    println!("No tau command, use zero tau");
                    na::DVector::zeros(7)
                } / 6.;
                let out_tau = franka::Vector7::from_row_slice(out_tau.as_slice());

                println!("out_tau: {:?}", out_tau);
                println!("coriolis: {:?}", coriolis);
                println!("gravity: {:?}", gravity);

                (coriolis).into()
            };

            let result = match self.params.control_mode.as_str() {
                "joint" => robot.control_joint_positions(callback_joint, None, None, None),
                "joint_vel" => robot.control_joint_velocities(callback_joint_vel, None, None, None),
                "cartesian" => robot.control_cartesian_pose(callback_cartesian, None, None, None),
                "cartesian_vel" => {
                    robot.control_cartesian_velocities(callback_cartesian_vel, None, None, None)
                }
                "torque" => robot.control_torques(callback_torque, None, None),
                _ => panic!("Unsupported control mode"),
            };

            if let Err(e) = result {
                println!("Error: {}", e);
            }
        }

        self.state.robot = Some(robot);
    }
    #[cfg(unix)]
    fn update(&mut self) {
        // 更新机器人状态
        let panda = self.state.robot.as_mut().unwrap();
        let state = panda.read_once().unwrap();
        update_state(self.robot.as_ref().unwrap(), &state);

        // 如果存在非实时指令，则等待机器人静态后执行
        if !self.params.is_realtime {
            // 机器人静态检查，不静态的机器人不许工作
            if state.dq.iter().copied().sum::<f64>() < 0.01_f64 {
                return;
            }

            if let Some(DNodeMessage::Joint(joint)) = self.node.input_queue.pop() {
                let out_q = joint.as_slice().try_into().unwrap();
                panda.joint_motion(0.5, &out_q).unwrap();
            }
            // 实体机器人在输入非事实指令之后需要等待直到非实时指令完成，否则会出现错误
        }
    }
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}
#[cfg(unix)]
fn update_state(robot: &Arc<RwLock<DPanda>>, state: &franka::RobotState) {
    let mut robot = robot.write().unwrap();
    robot.state.q = na::DVector::from_row_slice(state.q.as_slice());
    robot.state.q_dot = na::DVector::from_row_slice(state.dq.as_slice());
}
