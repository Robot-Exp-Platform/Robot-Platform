use nalgebra as na;

use message::{Capsule, CollisionObject, Pose};

pub trait Robot<V>: Send + Sync {
    // get functions
    fn name(&self) -> String;
    fn dof(&self) -> usize;
    fn q(&self) -> V;
    fn q_dot(&self) -> V;
    fn q_ddot(&self) -> V;
    fn q_jerk(&self) -> V;
    fn q_default(&self) -> V;
    fn q_min_bound(&self) -> V;
    fn q_max_bound(&self) -> V;
    fn q_dot_bound(&self) -> V;
    fn q_ddot_bound(&self) -> V;
    fn q_jerk_bound(&self) -> V;
    fn tau_bound(&self) -> V;
    fn tau_dot_bound(&self) -> V;
    fn base(&self) -> Pose;

    // set functions
    fn set_name(&mut self, name: String);
    fn set_q(&mut self, q: V);
    fn set_q_dot(&mut self, q_dot: V);
    fn set_q_ddot(&mut self, q_ddot: V);
    fn set_q_jerk(&mut self, q_jerk: V);
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
