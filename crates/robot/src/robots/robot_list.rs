use std::sync::{Arc, RwLock};

use crate::robot_trait::{Robot, RobotType};

pub struct RobotList {
    name: String,
    path: String,

    pub robots: Vec<Arc<RwLock<dyn Robot>>>,
}

macro_rules! apply_closure_to_iter {
    ($robots:expr, $closure:expr) => {
        $robots.iter().map($closure).collect::<Vec<_>>()
    };
}

impl RobotList {
    pub fn new(name: String, path: String) -> RobotList {
        RobotList::new_with_robots(name, path, Vec::new())
    }

    pub fn new_with_robots(
        name: String,
        path: String,
        robots: Vec<Arc<RwLock<dyn Robot>>>,
    ) -> RobotList {
        RobotList { name, path, robots }
    }

    pub fn add_robot(&mut self, robot: Arc<RwLock<dyn Robot>>) {
        self.robots.push(robot)
    }
}

impl Robot for RobotList {
    fn get_name(&self) -> String {
        let names = apply_closure_to_iter!(self.robots, |robot| robot.read().unwrap().get_name())
            .join(", ");
        format!("{}:{{{}}}", self.name, names)
    }
    fn get_path(&self) -> String {
        self.path.clone()
    }
    fn get_type(&self) -> RobotType {
        RobotType::RobotListType(apply_closure_to_iter!(self.robots, |robot| robot
            .read()
            .unwrap()
            .get_type()))
    }

    fn get_end_effector_pose(&self) -> Vec<crate::robot_trait::Pose> {
        self.robots
            .iter()
            .flat_map(|robot| robot.read().unwrap().get_end_effector_pose())
            .collect()
    }

    fn set_name(&mut self, name: String) {
        self.name = name
    }
    fn set_path(&mut self, path: String) {
        self.path = path
    }

    fn reset_state(&mut self) {
        self.robots
            .iter_mut()
            .for_each(|robot| robot.write().unwrap().reset_state());
    }
}
