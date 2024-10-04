use nalgebra as na;

use message::{Capsule, CollisionObject, Pose};

pub trait Robot<T>: Send + Sync {
    // get functions
    fn name(&self) -> String;
    fn dof(&self) -> usize;
    fn q(&self) -> T;
    fn q_dot(&self) -> T;
    fn q_ddot(&self) -> T;
    fn q_jerk(&self) -> T;
    fn q_default(&self) -> T;
    fn q_min_bound(&self) -> T;
    fn q_max_bound(&self) -> T;
    fn q_dot_bound(&self) -> T;
    fn q_ddot_bound(&self) -> T;
    fn q_jerk_bound(&self) -> T;
    fn tau_bound(&self) -> T;
    fn tau_dot_bound(&self) -> T;
    fn base(&self) -> Pose;

    // set functions
    fn set_name(&mut self, name: String);
    fn set_q(&mut self, q: T);
    fn set_q_dot(&mut self, q_dot: T);
    fn set_q_ddot(&mut self, q_ddot: T);
    fn set_q_jerk(&mut self, q_jerk: T);
}

pub trait SRobot<const N: usize>: Robot<na::SVector<f64, N>> {
    // get functions
    fn end_effector(&self) -> Pose;
    fn capsules(&self) -> Vec<Capsule>;
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64;

    // culculate functions
    fn cul_end_effector(&self, q: &na::SVector<f64, N>) -> Pose;
    fn cul_capsules(&self, q: &na::SVector<f64, N>) -> Vec<Capsule>;
    fn cul_dis_to_collision(&self, q: &na::SVector<f64, N>, obj: &CollisionObject) -> f64;
    fn cul_dis_grad_to_collision(
        &self,
        q: &na::SVector<f64, N>,
        obj: &CollisionObject,
    ) -> na::SVector<f64, N>;

    fn reset(&mut self);
}

pub trait DRobot: Robot<na::DVector<f64>> {
    // get functions
    fn end_effector(&self) -> Pose;
    fn capsules(&self) -> Vec<Capsule>;
    fn dis_to_collision(&self, obj: &CollisionObject) -> f64;

    // culculate functions
    fn cul_end_effector(&self, q: &na::DVector<f64>) -> Pose;
    fn cul_capsules(&self, q: &na::DVector<f64>) -> Vec<Capsule>;
    fn cul_dis_to_collision(&self, q: &na::DVector<f64>, obj: &CollisionObject) -> f64;
    fn cul_dis_grad_to_collision(
        &self,
        q: &na::DVector<f64>,
        obj: &CollisionObject,
    ) -> na::DVector<f64>;

    fn reset(&mut self);
}
