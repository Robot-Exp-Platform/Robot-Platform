#[derive(Debug, Default)]
pub enum Constraint<'a> {
    #[default]
    NoConstraint, // 无约束
    Zero, // 零约束：x = 0

    AffineSpace,                     // 仿射空间约束：Ax = b
    EpigraphSquaredNorm(f64),        // 二次范数上确界约束：||x||^2 <= b
    FiniteSet(Vec<&'a [f64]>),       // 有限集约束：x in {x1, x2, ..., xn}
    Halfspace(&'a [f64], f64),       // 半空间约束：ax <= b
    Hyperplane(&'a [f64], f64),      // 超平面约束：ax = b
    Rectangle(&'a [f64], &'a [f64]), // 矩形约束：a <= x <= b
    Simplex,                         // 单纯形约束：x in Δ
    SecondOrderCone,                 // 二阶锥约束

    Intersection(Vec<Constraint<'a>>),                 // 交集约束
    Union(Vec<Constraint<'a>>),                        // 并集约束
    CartesianProduct(Vec<usize>, Vec<Constraint<'a>>), // 笛卡尔积约束
}

impl<'a> Constraint<'a> {
    pub fn push(&mut self, int: usize, con: Constraint<'a>) {
        match self {
            Constraint::CartesianProduct(index, constraint) => {
                index.push(int);
                constraint.push(con);
            }
            Constraint::Intersection(index) => {
                index.push(con);
            }
            _ => panic!("Only CartesianProduct and Intersection can push"),
        }
    }
}
