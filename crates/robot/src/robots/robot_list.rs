use nalgebra as na;
use std::sync::{Arc, RwLock};

use crate::pose_to_svecter;
use crate::robot_trait::{BranchRobot, Robot};
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

impl BranchRobot for RobotList {
    fn get_q(&self) -> Vec<nalgebra::DVector<f64>> {
        unimplemented!()
    }
    fn get_end_trans_difference_with_q(
        &self,
        indices: (usize, usize),
        joint1: &na::DVector<f64>,
        joint2: &na::DVector<f64>,
        trans1: &na::Isometry3<f64>,
        trans2: &na::Isometry3<f64>,
    ) -> na::SVector<f64, 6> {
        let pose1 = self.robots[indices.0]
            .read()
            .unwrap()
            .get_end_effector_pose_with_q(&joint1);
        let pose2 = self.robots[indices.1]
            .read()
            .unwrap()
            .get_end_effector_pose_with_q(&joint2);
        pose_to_svecter(&pose1.inv_mul(&trans1.inv_mul(&(pose2 * trans2))))
    }

    fn get_end_trans_difference_grad_with_q(
        &self,
        indices: (usize, usize),
        joint1: &na::DVector<f64>,
        joint2: &na::DVector<f64>,
        trans1: &na::Isometry3<f64>,
        trans2: &na::Isometry3<f64>,
    ) -> na::DMatrix<f64> {
        let robot1 = self.robots[indices.0].read().unwrap();
        let robot2 = self.robots[indices.1].read().unwrap();
        let dim1 = robot1.get_ndof();
        let dim2 = robot2.get_ndof();
        let mut gard = na::DMatrix::<f64>::zeros(6, dim1 + dim2);
        let epsilon = 1e-3;

        for i in 0..dim1 {
            let mut joint_plus = joint1.clone_owned();
            let mut joint_minus = joint1.clone_owned();
            joint_plus[i] += epsilon;
            joint_minus[i] -= epsilon;
            let plus =
                self.get_end_trans_difference_with_q(indices, &joint_plus, joint2, trans1, trans2);
            let minus =
                self.get_end_trans_difference_with_q(indices, &joint_minus, joint2, trans1, trans2);
            gard.column_mut(i)
                .copy_from(&((plus - minus) / (2.0 * epsilon)))
        }
        for i in 0..dim2 {
            let mut joint_plus = joint2.clone_owned();
            let mut joint_minus = joint2.clone_owned();
            joint_plus[i] += epsilon;
            joint_minus[i] -= epsilon;
            let plus =
                self.get_end_trans_difference_with_q(indices, joint1, &joint_plus, trans1, trans2);
            let minus =
                self.get_end_trans_difference_with_q(indices, joint1, &joint_minus, trans1, trans2);
            gard.column_mut(dim1 + i)
                .copy_from(&((plus - minus) / (2.0 * epsilon)));
        }

        gard
    }
}

impl Robot for RobotList {
    fn get_ndof(&self) -> usize {
        self.robots
            .iter()
            .map(|robot| robot.read().unwrap().get_ndof())
            .sum()
    }
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
    fn get_end_effector_pose_with_q(&self, _: &nalgebra::DVector<f64>) -> Pose {
        unimplemented!()
    }
    fn get_distance_with_slice(&self, _: &[f64], _: &CollisionObject) -> f64 {
        unimplemented!()
    }
    fn get_distance_grad_with_slice(&self, _: &[f64], _: &CollisionObject) -> Vec<f64> {
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
    fn get_robot_indices(&self, robot_names: Vec<String>) -> Vec<usize> {
        robot_names
            .into_iter()
            .map(|name| {
                self.robots
                    .iter()
                    .position(|robot| robot.read().unwrap().get_name() == name)
                    .unwrap()
            })
            .collect()
    }

    fn reset_state(&mut self) {
        self.robots
            .iter_mut()
            .for_each(|robot| robot.write().unwrap().reset_state());
    }

    fn safety_check<'a>(&self, _: Message<'a>) -> Result<Message<'a>, ()> {
        Err(())
    }
}
