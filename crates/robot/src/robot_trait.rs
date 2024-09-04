use message::{collision_object::Capsule, CollisionObject, Pose, RobotMessageN};
use nalgebra as na;

pub trait Robot: Send + Sync {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_ndof(&self) -> usize;
    fn get_q_with_indptr(&self) -> (Vec<usize>, Vec<f64>);
    fn get_end_effector_pose(&self) -> Vec<Pose>;
    fn get_end_effector_pose_with_q(&self, q: &na::DVector<f64>) -> Pose;
    fn get_joint_capsules(&self) -> Vec<Capsule>;
    fn get_distance_to_collision(&self, obj: &CollisionObject) -> f64;
    fn get_distance_with_slice(&self, q: &[f64], obj: &CollisionObject) -> f64;
    fn get_distance_grad_with_slice(&self, q: &[f64], obj: &CollisionObject) -> Vec<f64>;
    fn get_robot_indices(&self, robot_names: Vec<String>) -> Vec<usize>;

    fn set_name(&mut self, name: String);
    fn set_path(&mut self, path: String);

    fn reset_state(&mut self);
}

pub trait SeriesRobot<const N: usize>: Robot {
    fn get_q(&self) -> na::SVector<f64, N>;
    fn get_q_dot(&self) -> na::SVector<f64, N>;
    fn get_q_ddot(&self) -> na::SVector<f64, N>;
    fn get_q_jack(&self) -> na::SVector<f64, N>;
    fn get_q_min_bound(&self) -> na::SVector<f64, N>;
    fn get_q_max_bound(&self) -> na::SVector<f64, N>;
    fn get_q_dot_bound(&self) -> na::SVector<f64, N>;
    fn get_q_ddot_bound(&self) -> na::SVector<f64, N>;
    fn get_q_jack_bound(&self) -> na::SVector<f64, N>;
    fn get_base(&self) -> Pose;
    fn get_end_effector_pose_na(&self) -> Pose;
    fn get_joint_capsules_with_joint(&self, joint: &na::SVector<f64, N>) -> Vec<Capsule>;
    fn get_distance_with_joint(&self, joint: &na::SVector<f64, N>, obj: &CollisionObject) -> f64;
    fn get_distance_grad_with_joint(
        &self,
        joint: &na::SVector<f64, N>,
        obj: &CollisionObject,
    ) -> na::SVector<f64, N>;

    fn set_q(&mut self, q: na::SVector<f64, N>);
    fn set_q_dot(&mut self, q_dot: na::SVector<f64, N>);
    fn update_dh(&mut self);
    fn safety_check(&self, msg: &RobotMessageN<N>) -> bool;
}

pub trait BranchRobot: Robot {
    fn get_q(&self) -> Vec<na::DVector<f64>>;
    fn get_end_trans_difference_with_q(
        &self,
        indices: (usize, usize),
        joint1: &na::DVector<f64>,
        joint2: &na::DVector<f64>,
        trans1: &na::Isometry3<f64>,
        trans2: &na::Isometry3<f64>,
    ) -> na::SVector<f64, 6>;
    fn get_end_trans_difference_grad_with_q(
        &self,
        indices: (usize, usize),
        joint1: &na::DVector<f64>,
        joint2: &na::DVector<f64>,
        trans1: &na::Isometry3<f64>,
        trans2: &na::Isometry3<f64>,
    ) -> na::DMatrix<f64>;
}
