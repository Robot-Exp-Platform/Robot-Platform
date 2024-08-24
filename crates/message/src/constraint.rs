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
            Constraint::CartesianProduct(nrows, _, _) => *nrows,
            Constraint::Intersection(nrows, _, _) => *nrows,
            Constraint::Union(nrows, _, _) => *nrows,
            _ => 0,
        }
    }

    pub fn ncols(&self) -> usize {
        match self {
            Constraint::CartesianProduct(_, ncols, _) => *ncols,
            Constraint::Intersection(_, ncols, _) => *ncols,
            Constraint::Union(_, ncols, _) => *ncols,
            _ => 0,
        }
    }

    pub fn push(&mut self, dim: usize, con: Constraint) {
        match self {
            Constraint::CartesianProduct(nrows, ncols, constraint) => {
                *nrows += dim;
                *ncols += dim;
                constraint.push(con);
            }
            Constraint::Intersection(nrows, ncols, constraint) => {
                *nrows += con.nrows();
                assert_eq!(dim, *ncols);
                constraint.push(con);
            }
            _ => panic!("Only CartesianProduct and Intersection can push"),
        }
    }

    pub fn to_inequation(&self) -> (usize, usize, Vec<f64>, Vec<f64>, Vec<f64>) {
        match self {
            Constraint::NoConstraint => (0, 0, vec![], vec![], vec![]),
            Constraint::Zero => (1, 1, vec![0.0], vec![0.0], vec![0.0]),
            Constraint::Equared(b) => (1, b.len(), b.clone(), b.clone(), b.clone()),
            Constraint::Halfspace(a, b) => {
                (1, a.len(), a.clone(), vec![f64::NEG_INFINITY], vec![*b])
            }
            Constraint::Hyperplane(a, b) => (1, a.len(), a.clone(), vec![*b], vec![*b]),
            Constraint::Rectangle(a, b) => {
                let n = a.len();
                let mut t = Vec::with_capacity(n * n);

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
                let mut all_t = vec![0.0; all_ncols * all_ncols];
                let mut all_l = Vec::with_capacity(*all_ncols);
                let mut all_u = Vec::with_capacity(*all_ncols);
                let mut total_nrows = 0;
                let mut total_ncols = 0;

                for constraint in constraints {
                    let (nrows, ncols, t, l, u) = constraint.to_inequation();
                    all_l.extend(l);
                    all_u.extend(u);
                    for i in 0..nrows {
                        for j in 0..ncols {
                            all_t[(i + total_nrows) * all_ncols + (total_ncols + j)] =
                                t[i * ncols + j];
                        }
                    }

                    total_nrows += nrows;
                    total_ncols += ncols;
                }

                assert_eq!(total_ncols, *all_ncols);
                assert_eq!(total_nrows, *all_nrows);

                (*all_nrows, *all_ncols, all_t, all_l, all_u)
            }
            _ => unimplemented!(),
        }
    }
}
