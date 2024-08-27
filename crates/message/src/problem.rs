use nalgebra as na;
use osqp::CscMatrix;
use std::borrow::Cow;

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

impl<'a> QuadraticProgramming<'a> {
    pub fn get_csc_h(&self) -> CscMatrix {
        let h = self.h;
        let (nrows, ncols) = h.shape();

        let mut indptr = vec![0];
        let mut indices = vec![];
        let mut data = vec![];

        for j in 0..ncols {
            for i in 0..nrows {
                if h[(i, j)] != 0.0 {
                    data.push(h[(i, j)]);
                    indices.push(i);
                }
            }
            indptr.push(indices.len());
        }

        CscMatrix {
            nrows,
            ncols,
            indptr: Cow::Owned(indptr),
            indices: Cow::Owned(indices),
            data: Cow::Owned(data),
        }
    }
}
