use std::sync::Arc;

use crossbeam::queue::SegQueue;
use nalgebra as na;
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

pub type DNodeMessage = NodeMessage<na::DVector<f64>>;
pub type SNodeMessage<const N: usize> = NodeMessage<na::SVector<f64, N>>;

pub type NodeMessageQueue<V> = Arc<SegQueue<NodeMessage<V>>>;
pub type DNodeMessageQueue = Arc<SegQueue<DNodeMessage>>;
pub type SNodeMessageQueue<const N: usize> = Arc<SegQueue<SNodeMessage<N>>>;
