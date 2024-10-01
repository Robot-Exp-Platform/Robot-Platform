use nalgebra as na;

use message::{Capsule, CollisionObject, Pose};

pub trait Robot: Send + Sync {
    // get functions
    fn name(&self) -> String;
    fn dof(&self) -> usize;
    fn capsules(&self) -> Vec<Capsule>;
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64;

    fn set_name(&mut self, name: String);

    fn reset(&mut self);
}

pub trait SRobot<const N: usize>: Robot {
    // get functions
    fn q(&self) -> na::SVector<f64, N>;
    fn q_dot(&self) -> na::SVector<f64, N>;
    fn q_ddot(&self) -> na::SVector<f64, N>;
    fn q_jerk(&self) -> na::SVector<f64, N>;
    fn q_min_bound(&self) -> na::SVector<f64, N>;
    fn q_max_bound(&self) -> na::SVector<f64, N>;
    fn q_dot_bound(&self) -> na::SVector<f64, N>;
    fn q_ddot_bound(&self) -> na::SVector<f64, N>;
    fn q_jerk_bound(&self) -> na::SVector<f64, N>;
    fn base(&self) -> Pose;
    fn end_effector(&self) -> Pose;

    // culculate functions
    fn cul_end_effector(&self, q: &na::SVector<f64, N>) -> Pose;
    fn cul_dis_to_collision(&self, q: &na::SVector<f64, N>, obj: &CollisionObject) -> f64;
    fn cul_dis_grad_to_collision(
        &self,
        q: &na::SVector<f64, N>,
        obj: &CollisionObject,
    ) -> na::SVector<f64, N>;

    // set functions
    fn set_q(&mut self, q: na::SVector<f64, N>);
    fn set_q_dot(&mut self, q_dot: na::SVector<f64, N>);
    fn set_q_ddot(&mut self, q_ddot: na::SVector<f64, N>);
    fn set_q_jerk(&mut self, q_jerk: na::SVector<f64, N>);
}

pub trait DRobot: Robot {
    // get functions
    fn q(&self) -> na::DVector<f64>;
    fn q_dot(&self) -> na::DVector<f64>;
    fn q_ddot(&self) -> na::DVector<f64>;
    fn q_jerk(&self) -> na::DVector<f64>;
    fn q_default(&self) -> na::DVector<f64>;
    fn q_min_bound(&self) -> na::DVector<f64>;
    fn q_max_bound(&self) -> na::DVector<f64>;
    fn q_dot_bound(&self) -> na::DVector<f64>;
    fn q_ddot_bound(&self) -> na::DVector<f64>;
    fn q_jerk_bound(&self) -> na::DVector<f64>;
    fn tau_bound(&self) -> na::DVector<f64>;
    fn tau_dot_bound(&self) -> na::DVector<f64>;
    fn base(&self) -> Pose;
    fn end_effector(&self) -> Pose;

    // culculate functions
    fn cul_end_effector(&self, q: &na::DVector<f64>) -> Pose;
    fn cul_capsules(&self, q: &na::DVector<f64>) -> Vec<Capsule>;
    fn cul_dis_to_collision(&self, q: &na::DVector<f64>, obj: &CollisionObject) -> f64;
    fn cul_dis_grad_to_collision(
        &self,
        q: &na::DVector<f64>,
        obj: &CollisionObject,
    ) -> na::DVector<f64>;

    // set functions
    fn set_q(&mut self, q: na::DVector<f64>);
    fn set_q_dot(&mut self, q_dot: na::DVector<f64>);
    fn set_q_ddot(&mut self, q_ddot: na::DVector<f64>);
    fn set_q_jerk(&mut self, q_jerk: na::DVector<f64>);
}
