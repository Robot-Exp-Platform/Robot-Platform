use message::collision_object::{Capsule, CollisionObject};
use message::message_trait::Message;
use message::state::Pose;
use nalgebra as na;

pub trait Robot: Send + Sync {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_end_effector_pose(&self) -> Vec<Pose>;
    fn get_joint_capsules(&self) -> Vec<Capsule>;
    fn get_distance_to_collision(&self, obj: &CollisionObject) -> f64;

    fn set_name(&mut self, name: String);
    fn set_path(&mut self, path: String);
    fn set_q(&mut self, q: Vec<f64>);
    fn set_q_dot(&mut self, q_dot: Vec<f64>);

    fn reset_state(&mut self);

    fn check_joint(&self, joint: &Vec<f64>) -> bool;
    fn check_vel(&self, vel: &Vec<f64>) -> bool;
    fn check_acc(&self, acc: &Vec<f64>) -> bool;
    fn check_tau(&self, tau: &Vec<f64>) -> bool;
    fn safety_check<'a>(&self, msg: Message<'a>) -> Result<Message<'a>, ()>;
}

// pub trait RobotState {}

pub trait SeriesRobot<const N: usize>: Robot {
    fn get_q_na(&self) -> na::SVector<f64, N>;
    fn get_q_dot_na(&self) -> na::SVector<f64, N>;
    fn get_q_ddot_na(&self) -> na::SVector<f64, N>;
    fn get_q_jack_na(&self) -> na::SVector<f64, N>;
    fn get_q_min_bound_na(&self) -> na::SVector<f64, N>;
    fn get_q_max_bound_na(&self) -> na::SVector<f64, N>;
    fn get_q_dot_bound_na(&self) -> na::SVector<f64, N>;
    fn get_q_ddot_bound_na(&self) -> na::SVector<f64, N>;
    fn get_q_jack_bound_na(&self) -> na::SVector<f64, N>;
    fn get_base(&self) -> Pose;
    fn get_end_effector_pose_na(&self) -> Pose;
    fn get_joint_capsules_with_joint(&self, joint: &na::SVector<f64, N>) -> Vec<Capsule>;
    fn get_distance_with_joint(&self, joint: &na::SVector<f64, N>, obj: &CollisionObject) -> f64;
    fn get_distance_diff_with_joint(
        &self,
        joint: &na::SVector<f64, N>,
        bj: &CollisionObject,
    ) -> na::SVector<f64, N>;
    fn update_dh(&mut self);
}
