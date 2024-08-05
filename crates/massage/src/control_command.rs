use nalgebra as na;

#[derive(Debug)]
pub enum ControlCommand {
    Joint7(Joint<7>),
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Joint<const N: usize> {
    q: na::SVector<f64, N>,
}

impl<const N: usize> Joint<N> {
    pub fn new(q: na::SVector<f64, N>) -> Self {
        Self { q }
    }
}
