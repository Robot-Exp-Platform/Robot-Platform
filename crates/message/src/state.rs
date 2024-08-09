use nalgebra as na;

pub enum State {}

pub type Pose = na::Isometry3<f64>;

type Joint<const N: usize> = na::SVector<f64, N>;
type Velocity<const N: usize> = na::SVector<f64, N>;
type Acceleration<const N: usize> = na::SVector<f64, N>;

pub enum RobotState<const N: usize> {
    Poes(Pose),
    Joint(Joint<N>),
    Velocity(Velocity<N>),
    Acceleration(Acceleration<N>),
    JointVelocity(JointVelocity<N>),
    JointVelocityAcceleration(JointVelocityAcceleration<N>),
}

pub struct JointVelocity<const N: usize> {
    pub q: Joint<N>,
    pub q_dot: Velocity<N>,
}

pub struct JointVelocityAcceleration<const N: usize> {
    pub q: Joint<N>,
    pub q_dot: Velocity<N>,
    pub q_ddot: Acceleration<N>,
}
