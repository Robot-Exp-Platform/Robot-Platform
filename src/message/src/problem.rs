use crate::constraint::*;
use osqp::CscMatrix;

#[derive(Debug)]
pub enum Problem<'a> {
    QuadraticProgramming(QuadraticProgramming<'a>),
}

#[derive(Debug)]
pub struct QuadraticProgramming<'a> {
    pub h: &'a CscMatrix<'a>,
    pub f: &'a [f64],
    pub constraints: Constraint,
}
