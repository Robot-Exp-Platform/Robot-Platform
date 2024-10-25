use nalgebra::{DVector, SVector};

use crate::NodeMessage;

pub type Track<V> = NodeMessage<V>;

pub type DTrack = Track<DVector<f64>>;
pub type STrack<const N: usize> = Track<SVector<f64, N>>;
