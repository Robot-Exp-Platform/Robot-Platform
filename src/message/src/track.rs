use nalgebra::{DVector, SVector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub enum Track<V> {
    #[default]
    NoneTrack,
    Pose(V),
    Joint(V),
    Velocity(V),
    Acceleration(V),
    JointVel(V, V),
    JointVelAcc(V, V, V),
}

pub type DTrack = Track<DVector<f64>>;
pub type STrack<const N: usize> = Track<SVector<f64, N>>;
