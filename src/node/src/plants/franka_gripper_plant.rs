use generate_tools::{get_fn, todo_fn};
use nalgebra as na;
use serde::Deserialize;
use serde_json::{from_value, Value};
use std::sync::{Arc, RwLock};

use crate::{Node, NodeBehavior};
use message::DNodeMessageQueue;
use robot::{Gripper, RobotType};
use sensor::Sensor;

#[allow(dead_code)]
pub struct GripperPlant {
    name: String,
    state: GripperPlantState,
    params: GripperPlantParams,
    gripper: Option<Arc<RwLock<Gripper>>>,
}

#[allow(dead_code)]
pub struct GripperPlantState {
    width: f64,
    #[cfg(unix)]
    gripper: Option<franka::Gripper>,
}

#[derive(Deserialize)]
#[allow(dead_code)]
pub struct GripperPlantParams {
    ip: String,
    period: f64,
}

impl GripperPlant {
    pub fn from_json(name: String, json: Value) -> GripperPlant {
        GripperPlant::from_params(name, from_value(json).unwrap())
    }
    pub fn from_params(name: String, params: GripperPlantParams) -> GripperPlant {
        GripperPlant {
            name,
            state: GripperPlantState {
                width: 0.0,
                #[cfg(unix)]
                gripper: None,
            },
            params,
            gripper: None,
        }
    }
}

impl Node<na::DVector<f64>> for GripperPlant {
    get_fn!((name: String));
    todo_fn!((set_input_queue, input_queue: DNodeMessageQueue, node),
            (set_output_queue, output_queue: DNodeMessageQueue, node));
    todo_fn!((set_sensor, sensor: Arc<RwLock<Sensor>>),
            (set_params, params: Value));

    fn set_robot(&mut self, robot: robot::RobotType) {
        if let RobotType::FrankaGripper(gripper) = robot {
            self.gripper = Some(gripper);
        }
    }
}

impl NodeBehavior for GripperPlant {
    #[cfg(unix)]
    fn start(&mut self) {
        self.state.gripper = Some(franka::Gripper::new(&self.params.ip).unwrap());
    }
    #[cfg(unix)]
    fn update(&mut self) {
        let width = if let Some(gripper) = self.gripper.as_ref() {
            gripper.read().unwrap().width()
        } else {
            return;
        };

        if self.gripper.as_ref().unwrap().read().unwrap().homing {
            self.state.gripper.as_mut().unwrap().homing().unwrap();
        }

        if width != self.state.width {
            if width > self.state.width {
                self.state.gripper.as_mut().unwrap().homing().unwrap();
            }

            self.state.width = width;
            self.state
                .gripper
                .as_mut()
                .unwrap()
                .grasp(width, 0.1, 20., None, None)
                .unwrap();
            self.state.gripper.as_mut().unwrap().homing()
        }
    }

    fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(self.params.period)
    }
    fn node_name(&self) -> String {
        self.name.clone()
    }
}
