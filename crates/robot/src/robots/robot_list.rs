use std::sync::{Arc, RwLock};

use crate::robot_trait::Robot;
use message::{CollisionObject, Message, Pose};

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
    fn get_q_with_indptr(&self) -> (Vec<usize>, Vec<f64>) {
        let mut indptr = vec![0];
        let mut q = vec![];
        self.robots.iter().for_each(|robot| {
            let (_, q_) = robot.read().unwrap().get_q_with_indptr();
            q.extend(q_);
            indptr.push(q.len());
        });
        (indptr, q)
    }
    fn get_joint_capsules(&self) -> Vec<message::collision_object::Capsule> {
        self.robots
            .iter()
            .flat_map(|robot| robot.read().unwrap().get_joint_capsules())
            .collect()
    }
    fn get_end_effector_pose(&self) -> Vec<Pose> {
        self.robots
            .iter()
            .flat_map(|robot| robot.read().unwrap().get_end_effector_pose())
            .collect()
    }
    fn get_distance_with_slice(&self, _: &[f64], _: &CollisionObject) -> f64 {
        unimplemented!()
    }
    fn get_distance_grad_with_slice(&self, _: &[f64], _: &CollisionObject) -> Vec<f64> {
        unimplemented!()
    }
    fn get_end_effector_pose_with_q(&self, _: &nalgebra::DVector<f64>) {
        unimplemented!()
    }

    fn set_name(&mut self, name: String) {
        self.name = name
    }
    fn set_path(&mut self, path: String) {
        self.path = path
    }

    fn get_distance_to_collision(&self, _: &CollisionObject) -> f64 {
        unimplemented!()
    }

    fn reset_state(&mut self) {
        self.robots
            .iter_mut()
            .for_each(|robot| robot.write().unwrap().reset_state());
    }
}
