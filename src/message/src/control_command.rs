use nalgebra::{DVector, SVector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub enum ControlCommand<V> {
    NoneCmd,
    Joint(V),
    JointWithPeriod(f64, V),
    JointVel(V, V),
    JointVelWithPeriod(f64, V, V),
    JointVelAcc(V, V, V),
    JointVelAccWithPeriod(f64, V, V, V),
    Tau(V),
    TauWithPeriod(f64, V),
}

pub type DControlCommand = ControlCommand<DVector<f64>>;
pub type SControlCommand<const N: usize> = ControlCommand<SVector<f64, N>>;

impl<V> Default for ControlCommand<V> {
    fn default() -> Self {
        ControlCommand::NoneCmd
    }
}
