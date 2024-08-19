#[derive(Debug, Default)]
pub enum Constraint {
    #[default]
    NoConstraint, // 无约束
    Zero, // 零约束：x = 0

    Equared(Vec<f64>),             // 等式约束：Ax = b
    AffineSpace,                   // 仿射空间约束：Ax = b
    EpigraphSquaredNorm(f64),      // 二次范数上确界约束：||x||^2 <= b
    FiniteSet(Vec<Vec<f64>>),      // 有限集约束：x in {x1, x2, ..., xn}
    Halfspace(Vec<f64>, f64),      // 半空间约束：ax <= b
    Hyperplane(Vec<f64>, f64),     // 超平面约束：ax = b
    Rectangle(Vec<f64>, Vec<f64>), // 矩形约束：a <= x <= b
    Simplex,                       // 单纯形约束：x in Δ
    SecondOrderCone,               // 二阶锥约束

    Intersection(Vec<Constraint>),                 // 交集约束
    Union(Vec<Constraint>),                        // 并集约束
    CartesianProduct(Vec<usize>, Vec<Constraint>), // 笛卡尔积约束
}

impl Constraint {
    pub fn push(&mut self, int: usize, con: Constraint) {
        match self {
            Constraint::CartesianProduct(index, constraint) => {
                index.push(int);
                constraint.push(con);
            }
            Constraint::Intersection(constraint) => {
                constraint.push(con);
            }
            Constraint::Union(constraint) => constraint.push(con),
            _ => panic!("Only CartesianProduct and Intersection can push"),
        }
    }
}
