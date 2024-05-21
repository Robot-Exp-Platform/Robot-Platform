use crate::robot_trait::{Robot, RobotParams, RobotState, RobotType};
use nalgebra as na;

pub struct RobotList {
    name: String,
    pub robots: Vec<Box<dyn Robot>>,
}

macro_rules! apply_closure_to_iter {
    ($robots:expr, $closure:expr) => {
        $robots.iter().map($closure).collect::<Vec<_>>()
    };
}

impl RobotList {
    pub fn new(name: String) -> RobotList {
        RobotList::new_with_robots(name, Vec::new())
    }

    pub fn new_with_robots(name: String, robots: Vec<Box<dyn Robot>>) -> RobotList {
        RobotList { name, robots }
    }

    pub fn add_robot(&mut self, robot: Box<dyn Robot>) {
        self.robots.push(robot)
    }
}

impl Robot for RobotList {
    fn get_name(&self) -> String {
        let names = apply_closure_to_iter!(self.robots, |robot| robot.get_name()).join(", ");
        format!("{}:{{{}}}", self.name, names)
    }
    fn get_type(&self) -> RobotType {
        RobotType::RobotListType(apply_closure_to_iter!(self.robots, |robot| robot.get_type()))
    }
    fn get_state(&self) -> RobotState {
        RobotState::RobotListState(apply_closure_to_iter!(self.robots, |robot| robot.get_state()))
    }
    fn get_params(&self) -> RobotParams {
        RobotParams::RobotListParams(apply_closure_to_iter!(self.robots, |robot| robot.get_params()))
    }
    fn get_joint_positions(&self) -> na::DVector<f64> {
        let mut joint_positions = Vec::new();
        self.robots.iter().for_each(|robot| {
            joint_positions.extend_from_slice(&robot.get_joint_positions().as_slice())
        });
        na::DVector::from_column_slice(&joint_positions)
    }
    fn get_joint_velocities(&self) -> na::DVector<f64> {
        let mut joint_velocities = Vec::new();
        self.robots.iter().for_each(|robot| {
            joint_velocities.extend_from_slice(&robot.get_joint_velocities().as_slice())
        });
        na::DVector::from_column_slice(&joint_velocities)
    }
    fn get_end_effector_pose(&self) -> Vec<crate::robot_trait::Pose> {
        self.robots
            .iter()
            .flat_map(|robot| robot.get_end_effector_pose())
            .collect()
    }

    fn reset_state(&mut self) {
        self.robots.iter_mut().for_each(|robot| robot.reset_state())
    }

    fn update_state(&mut self, new_state: RobotState) {
        if let RobotState::RobotListState(robot_list_state) = new_state {
            self.robots
                .iter_mut()
                .zip(robot_list_state)
                .for_each(|(robot, state)| robot.update_state(state))
        }
    }
}
