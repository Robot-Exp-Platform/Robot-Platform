use message::collision_object::Capsule;
use message::state::Pose;
use message::message_trait::Message;
use nalgebra as na;

pub trait Robot: Send + Sync {
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_end_effector_pose(&self) -> Vec<Pose>;
    fn get_joint_capsules(&self) -> Vec<Capsule>;

    fn set_name(&mut self, name: String);
    fn set_path(&mut self, path: String);
    fn set_q(&mut self, q: Vec<f64>);
    fn set_q_dot(&mut self, q_dot: Vec<f64>);

    fn reset_state(&mut self);

    fn safety_check(&self, msg: &Message) -> bool;
}

// pub trait RobotState {}

pub trait SeriesRobot<const N: usize>: Robot {
    fn get_q_na(&self) -> na::SVector<f64, N>;
    fn get_q_dot_na(&self) -> na::SVector<f64, N>;
    fn get_q_ddot_na(&self) -> na::SVector<f64, N>;
    fn get_q_jack_na(&self) -> na::SVector<f64, N>;
    fn get_base(&self) -> Pose;

    fn get_end_effector_pose_na(&self) -> Pose;
}
