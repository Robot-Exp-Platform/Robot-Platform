use nalgebra as na;
use serde::Deserialize;
use std::f64;
use std::time::Duration;

use crate::{Node, NodeBehavior};

pub type Position<V> = Node<(), PositionParams, (), V>;

pub type DPosition = Position<na::DVector<f64>>;
pub type SPosition<const N: usize> = Position<na::SVector<f64, N>>;

#[derive(Deserialize)]
pub struct PositionParams {
    period: f64,
}

impl NodeBehavior for DPosition {
    fn update(&mut self) {
        if let Some(control_message) = self.input_queue.pop() {
            if self.is_end {
                todo!();
            } else {
                self.output_queue.push(control_message);
            }
        }
    }
    fn period(&self) -> Duration {
        Duration::from_secs_f64(self.params.period)
    }

    fn node_name(&self) -> String {
        self.name.clone()
    }
}
