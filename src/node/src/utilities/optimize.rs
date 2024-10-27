use nalgebra as na;
use osqp::CscMatrix;
use std::borrow::Cow;

pub fn get_optimize_function<'a>(
    dim: usize,
    offset: usize,
    const_weight: Vec<f64>,
) -> CscMatrix<'a> {
    let q1 = na::DMatrix::<f64>::identity(dim, dim);

    let mut v_diff = na::DMatrix::<f64>::identity(dim - offset, dim);
    for i in 0..dim - offset {
        v_diff[(i, i + offset)] = -1.0;
    }
    let q2 = v_diff.transpose() * v_diff;

    let mut a_diff = na::DMatrix::<f64>::identity(dim - 2 * offset, dim);
    for i in 0..dim - 2 * offset {
        a_diff[(i, i + offset)] = -2.0;
        a_diff[(i, i + 2 * offset)] = 1.0;
    }
    let q3 = a_diff.transpose() * a_diff;

    let h = q1 * const_weight[0] + q2 * const_weight[1] + q3 * const_weight[2];

    matrix_to_csc(&h)
}

pub fn matrix_to_csc<'a>(h: &na::DMatrix<f64>) -> CscMatrix<'a> {
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
