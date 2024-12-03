use franka;
use nalgebra as na;
use sensor::Sensor;
use serde::Deserialize;
use serde_json::{from_value, Value};
use std::{
    sync::{Arc, RwLock},
    time::Duration,
};

use crate::{Node, NodeBehavior};
use generate_tools::{get_fn, set_fn};
use message::{DNodeMessage, DNodeMessageQueue, NodeMessageQueue};
use robot::{DPanda, RobotType};

pub struct PandaPlant<R, V> {
    name: String,
    params: PandaPlantParams,
    node: PandaPlantNode<V>,
    robot: Option<Arc<RwLock<R>>>,
}

pub type DPandaPlant = PandaPlant<DPanda, na::DVector<f64>>;

#[derive(Deserialize, Default)]
pub struct PandaPlantParams {
    ip: String,
    control_mode: String,
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
        let mut robot = franka::Robot::new(&self.params.ip, None, None).unwrap();
        robot.set_default_behavior().unwrap();
        let model = robot.load_model(true).unwrap();
        let q_initial = self
            .robot
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .params
            .q_default
            .clone();
        let q_goal: [f64; 7] = q_initial.as_slice().try_into().unwrap();

        robot.joint_motion(0.5, &q_goal).unwrap();

        let callback_joint = |state: &franka::RobotState, _: &Duration| -> franka::JointPositions {
            let out_q: [f64; 7] =
                if let Some(DNodeMessage::Joint(joint)) = self.node.input_queue.pop() {
                    joint.as_slice().try_into().unwrap()
                } else {
                    state.q
                };
            franka::JointPositions::new(out_q)
        };
        let callback_torque = |state: &franka::RobotState, _: &Duration| -> franka::Torques {
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
            "torque" => robot.control_torques(callback_torque, None, None),
            _ => panic!("Unsupported control mode"),
        }
        .unwrap();
    }
    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(0.)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}
