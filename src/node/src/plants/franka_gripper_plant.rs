use kernel_macro::node_registration;
use nalgebra as na;
use serde::Deserialize;

use crate::{Node, NodeBehavior, NodeExtBehavior, NodeRegister};
use robot::{Gripper, RobotLock};

#[node_registration("gripper_plant")]
pub type GripperPlant =
    Node<GripperPlantState, GripperPlantParams, RobotLock<Gripper>, na::DVector<f64>>;

#[derive(Default)]
pub struct GripperPlantState {
    #[cfg(unix)]
    width: f64,
    #[cfg(unix)]
    gripper: Option<franka::Gripper>,
}

#[derive(Deserialize)]
pub struct GripperPlantParams {
    #[cfg(unix)]
    ip: String,
    period: f64,
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
