use serde::{Deserialize, Serialize};

use crate::Pose;

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub enum NodeMessage<V> {
    #[default]
    NoneNodeMessage,
    Pose(Pose),
    Joint(V),
    JointList(Vec<V>),
    JointWithPeriod(f64, V),
    JointVel(V, V),
    JointVelWithPeriod(f64, V, V),
    JointVelAcc(V, V, V),
    JointVelAccWithPeriod(f64, V, V, V),
    Tau(V),
    TauWithPeriod(f64, V),
}
