use nalgebra as na;
use osqp::CscMatrix;
use std::borrow::Cow;

#[derive(Debug, Default)]
pub enum Constraint {
    #[default]
    NoConstraint, // 无约束
    Zero, // 零约束：x = 0

    Equared(Vec<f64>),             // 等式约束：x = b
    AffineSpace,                   // 仿射空间约束：Ax = b
    EpigraphSquaredNorm(f64),      // 二次范数上确界约束：||x||^2 <= b
    FiniteSet(Vec<Vec<f64>>),      // 有限集约束：x in {x1, x2, ..., xn}
    Halfspace(Vec<f64>, f64),      // 半空间约束：ax <= b
    Hyperplane(Vec<f64>, f64),     // 超平面约束：ax = b
    Rectangle(Vec<f64>, Vec<f64>), // 矩形约束：a <= x <= b
    Simplex,                       // 单纯形约束：x in Δ
    SecondOrderCone,               // 二阶锥约束

    Intersection(usize, usize, Vec<Constraint>), // 交集约束
    Union(usize, usize, Vec<Constraint>),        // 并集约束
    CartesianProduct(usize, usize, Vec<Constraint>), // 笛卡尔积约束
}

impl Constraint {
    pub fn nrows(&self) -> usize {
        match self {
            Constraint::NoConstraint => 0,
            Constraint::Zero => 1,
            Constraint::Equared(b) => b.len(),
            Constraint::Halfspace(_, _) => 1,
            Constraint::Hyperplane(_, _) => 1,
            Constraint::Rectangle(a, _) => a.len(),

            Constraint::CartesianProduct(nrows, _, _) => *nrows,
            Constraint::Intersection(nrows, _, _) => *nrows,
            Constraint::Union(nrows, _, _) => *nrows,
            _ => panic!("This constraint has no rows func"),
        }
    }

    pub fn ncols(&self) -> usize {
        match self {
            Constraint::NoConstraint => 0,
            Constraint::Zero => 1,
            Constraint::Equared(b) => b.len(),
            Constraint::Halfspace(a, _) => a.len(),
            Constraint::Hyperplane(a, _) => a.len(),
            Constraint::Rectangle(a, _) => a.len(),

            Constraint::CartesianProduct(_, ncols, _) => *ncols,
            Constraint::Intersection(_, ncols, _) => *ncols,
            Constraint::Union(_, ncols, _) => *ncols,
            _ => panic!("This constraint has no cols func"),
        }
    }

    pub fn push(&mut self, con: Constraint) {
        match self {
            Constraint::CartesianProduct(nrows, ncols, constraint) => {
                *nrows += con.nrows();
                *ncols += con.ncols();
                constraint.push(con);
            }
            Constraint::Intersection(nrows, ncols, constraint) => {
                *nrows += con.nrows();
                *ncols = con.ncols();

                constraint.push(con);
            }
            _ => panic!("Only CartesianProduct and Intersection can push"),
        }
    }

    pub fn to_inequation(&self) -> (usize, usize, Vec<f64>, Vec<f64>, Vec<f64>) {
        match self {
            Constraint::NoConstraint => (0, 0, vec![], vec![], vec![]),
            Constraint::Zero => (1, 1, vec![0.0], vec![0.0], vec![0.0]),
            Constraint::Equared(b) => {
                let n = b.len();
                let mut t = vec![0.0; n * n];

                for i in 0..n {
                    t[i * n + i] = 1.0;
                }

                (n, n, t, b.clone(), b.clone())
            }
            Constraint::Halfspace(a, b) => {
                (1, a.len(), a.clone(), vec![f64::NEG_INFINITY], vec![*b])
            }
            Constraint::Hyperplane(a, b) => (1, a.len(), a.clone(), vec![*b], vec![*b]),
            Constraint::Rectangle(a, b) => {
                let n = a.len();
                let mut t = vec![0.0; n * n];

                for i in 0..n {
                    t[i * n + i] = 1.0;
                }

                (n, n, t, a.clone(), b.clone())
            }

            Constraint::Intersection(nrows, ncols, constraint) => {
                let mut all_t = Vec::with_capacity(nrows * ncols);
                let mut all_l = Vec::with_capacity(*ncols);
                let mut all_u = Vec::with_capacity(*ncols);
                let mut total_nrows = 0;

                for con in constraint {
                    let (nrows, _, t, l, u) = con.to_inequation();
                    all_t.extend(t);
                    all_l.extend(l);
                    all_u.extend(u);

                    total_nrows += nrows;
                }

                assert_eq!(total_nrows, *nrows);

                (*nrows, *ncols, all_t, all_l, all_u)
            }
            Constraint::CartesianProduct(all_nrows, all_ncols, constraints) => {
                let mut all_t = vec![0.0; all_nrows * all_ncols];
                let mut all_l = Vec::with_capacity(*all_ncols);
                let mut all_u = Vec::with_capacity(*all_ncols);
                let mut total_nrows = 0;
                let mut total_ncols = 0;

                for constraint in constraints {
                    let (nrows, ncols, t, l, u) = constraint.to_inequation();
                    all_l.extend(l);
                    all_u.extend(u);

                    // 约束维度检查输出
                    // println!(
                    //     "cartesian product: [({},{})|({},{})/({},{})]",
                    //     nrows, ncols, total_nrows, total_ncols, all_nrows, all_ncols
                    // );
                    for i in 0..nrows {
                        for j in 0..ncols {
                            all_t[(i + total_nrows) * all_ncols + (total_ncols + j)] =
                                t[i * ncols + j];
                        }
                    }

                    total_nrows += nrows;
                    total_ncols += ncols;
                }

                assert_eq!(total_nrows, *all_nrows);
                assert_eq!(total_ncols, *all_ncols);

                (*all_nrows, *all_ncols, all_t, all_l, all_u)
            }
            _ => unimplemented!(),
        }
    }

    pub fn to_cscmatrix(&self) -> (usize, usize, CscMatrix, Vec<f64>, Vec<f64>) {
        match self {
            Constraint::NoConstraint => (
                0,
                0,
                CscMatrix {
                    nrows: 0,
                    ncols: 0,
                    indptr: Cow::Owned(vec![]),
                    indices: Cow::Owned(vec![]),
                    data: Cow::Owned(vec![]),
                },
                vec![],
                vec![],
            ),
            Constraint::Zero => (
                1,
                1,
                CscMatrix {
                    nrows: 1,
                    ncols: 1,
                    indptr: Cow::Owned(vec![0, 1]),
                    indices: Cow::Owned(vec![0]),
                    data: Cow::Owned(vec![1.0]), // x = 0 represented by 1*x = 0
                },
                vec![0.0],
                vec![0.0],
            ),
            Constraint::Equared(b) => {
                let n = b.len();
                let mut indptr = Vec::with_capacity(n + 1);
                let mut indices = Vec::with_capacity(n);
                let mut data = Vec::with_capacity(n);

                for i in 0..n {
                    indptr.push(i);
                    indices.push(i);
                    data.push(1.0);
                }
                indptr.push(n);

                (
                    n,
                    n,
                    CscMatrix {
                        nrows: n,
                        ncols: n,
                        indptr: Cow::Owned(indptr),
                        indices: Cow::Owned(indices),
                        data: Cow::Owned(data),
                    },
                    b.clone(),
                    b.clone(),
                )
            }
            Constraint::Halfspace(a, b) => {
                let n = a.len();
                (
                    1,
                    n,
                    CscMatrix {
                        nrows: 1,
                        ncols: n,
                        indptr: Cow::Owned((0..=n).collect()),
                        indices: Cow::Owned(vec![0; n]),
                        data: Cow::Owned(a.clone()),
                    },
                    vec![f64::NEG_INFINITY],
                    vec![*b],
                )
            }
            Constraint::Hyperplane(a, b) => {
                let n = a.len();
                (
                    1,
                    n,
                    CscMatrix {
                        nrows: 1,
                        ncols: n,
                        indptr: Cow::Owned((0..=n).collect()),
                        indices: Cow::Owned(vec![0; n]),
                        data: Cow::Owned(a.clone()),
                    },
                    vec![*b],
                    vec![*b],
                )
            }
            Constraint::Rectangle(a, b) => {
                let n = a.len();
                let mut indptr = Vec::with_capacity(n + 1);
                let mut indices = Vec::with_capacity(n);
                let mut data = Vec::with_capacity(n);

                for i in 0..n {
                    indptr.push(i);
                    indices.push(i);
                    data.push(1.0);
                }
                indptr.push(n);

                (
                    n,
                    n,
                    CscMatrix {
                        nrows: n,
                        ncols: n,
                        indptr: Cow::Owned(indptr),
                        indices: Cow::Owned(indices),
                        data: Cow::Owned(data),
                    },
                    a.clone(),
                    b.clone(),
                )
            }
            Constraint::Intersection(nrows, ncols, constraints) => {
                let mut all_indptr = vec![0; *ncols + 1];
                let mut all_indices = vec![];
                let mut all_data = vec![];
                let mut all_l = Vec::with_capacity(*ncols);
                let mut all_u = Vec::with_capacity(*ncols);
                let mut total_nrows = 0;

                let cons: Vec<CscMatrix> = constraints
                    .iter()
                    .map(|con| {
                        let (_, _, t, l, u) = con.to_cscmatrix();
                        all_l.extend(l.iter());
                        all_u.extend(u);
                        t
                    })
                    .collect();

                for i in 0..*ncols {
                    total_nrows = 0;
                    cons.iter().for_each(|t| {
                        all_indptr[i + 1] += t.indptr[i + 1];
                        all_indices.extend(
                            t.indices[t.indptr[i]..t.indptr[i + 1]]
                                .iter()
                                .map(|&i| i + total_nrows),
                        );
                        all_data.extend(&t.data[t.indptr[i]..t.indptr[i + 1]]);

                        total_nrows += t.nrows;
                    });
                }

                assert_eq!(total_nrows, *nrows);

                (
                    *nrows,
                    *ncols,
                    CscMatrix {
                        nrows: *nrows,
                        ncols: *ncols,
                        indptr: Cow::Owned(all_indptr),
                        indices: Cow::Owned(all_indices),
                        data: Cow::Owned(all_data),
                    },
                    all_l,
                    all_u,
                )
            }
            Constraint::CartesianProduct(all_nrows, all_ncols, constraints) => {
                let mut all_indptr = vec![];
                let mut all_indices = vec![];
                let mut all_data = vec![];
                let mut all_l = Vec::with_capacity(*all_ncols);
                let mut all_u = Vec::with_capacity(*all_ncols);
                let mut total_nrows = 0;
                let mut total_ncols = 0;

                for con in constraints {
                    let (nrows, ncols, t, l, u) = con.to_cscmatrix();
                    all_l.extend(l);
                    all_u.extend(u);

                    // 约束维度检查输出
                    // println!(
                    //     "cartesian product: [({},{})|({},{})/({},{})]",
                    //     nrows, ncols, total_nrows, total_ncols, all_nrows, all_ncols
                    // );

                    let total_indptr = all_indptr.pop().unwrap_or(0);

                    all_indptr.extend(t.indptr.iter().map(|&i| i + total_indptr));
                    all_indices.extend(t.indices.iter().map(|&i| i + total_nrows));
                    all_data.extend(t.data.iter().cloned());

                    total_nrows += nrows;
                    total_ncols += ncols;
                }
                // all_indptr.push(all_indices.len() + 1);

                assert_eq!(total_nrows, *all_nrows);
                assert_eq!(total_ncols, *all_ncols);

                (
                    *all_nrows,
                    *all_ncols,
                    CscMatrix {
                        nrows: *all_nrows,
                        ncols: *all_ncols,
                        indptr: Cow::Owned(all_indptr),
                        indices: Cow::Owned(all_indices),
                        data: Cow::Owned(all_data),
                    },
                    all_l,
                    all_u,
                )
            }
            _ => unimplemented!(),
        }
    }
    pub fn to_namatrix(
        &self,
    ) -> (
        usize,
        usize,
        na::DMatrix<f64>,
        na::DVector<f64>,
        na::DVector<f64>,
    ) {
        match self {
            Constraint::NoConstraint => (
                0,
                0,
                na::DMatrix::zeros(0, 0),
                na::DVector::zeros(0),
                na::DVector::zeros(0),
            ),
            Constraint::Zero => (
                1,
                1,
                na::DMatrix::from_element(1, 1, 1.0),
                na::DVector::from_element(1, 0.0),
                na::DVector::from_element(1, 0.0),
            ),
            Constraint::Equared(b) => {
                let n = b.len();
                let mut t = na::DMatrix::zeros(n, n);

                for i in 0..n {
                    t[(i, i)] = 1.0;
                }

                (
                    n,
                    n,
                    t,
                    na::DVector::from_vec(b.clone()),
                    na::DVector::from_vec(b.clone()),
                )
            }
            Constraint::Halfspace(a, b) => (
                1,
                a.len(),
                na::DMatrix::from_row_slice(1, a.len(), a),
                na::DVector::from_element(1, f64::NEG_INFINITY),
                na::DVector::from_element(1, *b),
            ),
            Constraint::Hyperplane(a, b) => (
                1,
                a.len(),
                na::DMatrix::from_row_slice(1, a.len(), a),
                na::DVector::from_element(1, *b),
                na::DVector::from_element(1, *b),
            ),
            Constraint::Rectangle(a, b) => {
                let n = a.len();
                let mut t = na::DMatrix::zeros(n, n);

                for i in 0..n {
                    t[(i, i)] = 1.0;
                }

                (
                    n,
                    n,
                    t,
                    na::DVector::from_vec(a.clone()),
                    na::DVector::from_vec(b.clone()),
                )
            }
            Constraint::Intersection(nrows, ncols, constraints) => {
                let mut all_t = na::DMatrix::zeros(*nrows, *ncols);
                let mut all_l = na::DVector::zeros(*nrows);
                let mut all_u = na::DVector::zeros(*nrows);
                let mut total_nrows = 0;

                for con in constraints {
                    let (nrows, ncols, t, l, u) = con.to_namatrix();
                    all_t
                        .view_mut((total_nrows, 0), (nrows, ncols))
                        .copy_from(&t);
                    all_l.rows_mut(total_nrows, nrows).copy_from(&l);
                    all_u.rows_mut(total_nrows, nrows).copy_from(&u);

                    total_nrows += nrows;
                }

                assert_eq!(total_nrows, *nrows);

                (*nrows, *ncols, all_t, all_l, all_u)
            }
            Constraint::CartesianProduct(nrows, ncols, constraint) => {
                let mut all_t = na::DMatrix::zeros(*nrows, *ncols);
                let mut all_l = na::DVector::zeros(*nrows);
                let mut all_u = na::DVector::zeros(*nrows);
                let mut total_nrows = 0;
                let mut total_ncols = 0;

                for con in constraint {
                    let (nrows, ncols, t, l, u) = con.to_namatrix();
                    all_t
                        .view_mut((total_nrows, total_ncols), (nrows, ncols))
                        .copy_from(&t);
                    all_l.rows_mut(total_nrows, nrows).copy_from(&l);
                    all_u.rows_mut(total_nrows, nrows).copy_from(&u);

                    total_nrows += nrows;
                    total_ncols += ncols;
                }

                assert_eq!(total_nrows, *nrows);
                assert_eq!(total_ncols, *ncols);

                (*nrows, *ncols, all_t, all_l, all_u)
            }

            _ => unimplemented!(),
        }
    }
}
