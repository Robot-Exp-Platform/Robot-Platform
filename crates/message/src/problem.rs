use nalgebra as na;

use crate::constraint::*;

#[derive(Debug)]
pub enum Problem<'a> {
    QuadraticProgramming(QuadraticProgramming<'a>),
}

#[derive(Debug)]
pub struct QuadraticProgramming<'a> {
    pub h: &'a na::DMatrix<f64>,
    pub f: &'a na::DVector<f64>,
    pub constraints: Constraint,
}
