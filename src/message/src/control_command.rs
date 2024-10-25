use nalgebra::{DVector, SVector};

use crate::NodeMessage;

pub type ControlCommand<V> = NodeMessage<V>;

pub type DControlCommand = ControlCommand<DVector<f64>>;
pub type SControlCommand<const N: usize> = ControlCommand<SVector<f64, N>>;
