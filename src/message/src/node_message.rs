use std::{ops::Div, sync::Arc};

use crossbeam::queue::SegQueue;
use nalgebra as na;
use serde::{Deserialize, Serialize};

use crate::Pose;

#[derive(Debug, Default, Deserialize, Serialize, Clone)]
pub enum NodeMessage<V> {
    #[default]
    NoneNodeMessage,
    KillNode,
    NodeMessages(Vec<NodeMessage<V>>),
    Process(Box<NodeMessage<V>>, Box<NodeMessage<V>>),
    Period(f64, Box<NodeMessage<V>>),
    Relative(usize, usize, Pose),
    Pose(Pose),
    Transform(usize, Pose, Pose),
    Joint(V),
    JointList(Vec<V>),
    JointVel(V, V),
    JointVelAcc(V, V, V),
    Tau(V),
}

pub type DNodeMessage = NodeMessage<na::DVector<f64>>;
pub type SNodeMessage<const N: usize> = NodeMessage<na::SVector<f64, N>>;

pub type NodeMessageQueue<V> = Arc<SegQueue<NodeMessage<V>>>;
pub type DNodeMessageQueue = Arc<SegQueue<DNodeMessage>>;
pub type SNodeMessageQueue<const N: usize> = Arc<SegQueue<SNodeMessage<N>>>;

impl NodeMessage<na::DVector<f64>> {
    pub fn as_slice(&self) -> &[f64] {
        match self {
            Self::Period(_, msg) => msg.as_slice(),
            Self::Pose(pose) => pose.translation.vector.as_slice(),
            Self::Joint(joint) => joint.as_slice(),
            Self::JointVel(joint, _) => joint.as_slice(),
            Self::JointVelAcc(joint, _, _) => joint.as_slice(),
            Self::Tau(tau) => tau.as_slice(),
            _ => panic!("This type does not support as_slice"),
        }
    }
}

impl Div for NodeMessage<na::DVector<f64>> {
    type Output = f64;
    fn div(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (Self::Pose(lhs), Self::Pose(rhs)) => {
                let diff = lhs / rhs;
                diff.rotation.vector().norm()
            }
            (Self::Joint(lhs), Self::Joint(rhs)) => {
                let diff = lhs - rhs;
                diff.norm() / diff.len() as f64
            }
            (Self::JointVel(lhs, lvel), Self::JointVel(rhs, rvel)) => {
                let diff = lhs - rhs;
                let vdiff = lvel - rvel;
                diff.norm() / diff.len() as f64 + vdiff.norm() / vdiff.len() as f64
            }
            (Self::JointVelAcc(lhs, lvel, lacc), Self::JointVelAcc(rhs, rvel, racc)) => {
                let diff = lhs - rhs;
                let vdiff = lvel - rvel;
                let adiff = lacc - racc;
                diff.norm() / diff.len() as f64
                    + vdiff.norm() / vdiff.len() as f64
                    + adiff.norm() / adiff.len() as f64
            }
            _ => panic!("Divide operation not supported for this type"),
        }
    }
}

impl<const N: usize> Div for NodeMessage<na::SVector<f64, N>> {
    type Output = f64;
    fn div(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (Self::Joint(lhs), Self::Joint(rhs)) => {
                let diff = lhs - rhs;
                diff.norm() / diff.len() as f64
            }
            (Self::JointVel(lhs, lvel), Self::JointVel(rhs, rvel)) => {
                let diff = lhs - rhs;
                let vdiff = lvel - rvel;
                diff.norm() / diff.len() as f64 + vdiff.norm() / vdiff.len() as f64
            }
            (Self::JointVelAcc(lhs, lvel, lacc), Self::JointVelAcc(rhs, rvel, racc)) => {
                let diff = lhs - rhs;
                let vdiff = lvel - rvel;
                let adiff = lacc - racc;
                diff.norm() / diff.len() as f64
                    + vdiff.norm() / vdiff.len() as f64
                    + adiff.norm() / adiff.len() as f64
            }
            _ => panic!("Divide operation not supported for this type"),
        }
    }
}
