use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub enum Constraint {
    Equality(Vec<f64>, f64),   // 线性等式约束：ax = b
    Inequality(Vec<f64>, f64), // 线性不等式约束：ax <= b
    Bound(Vec<f64>, Vec<f64>), // 变量的上下界约束：l <= x <= u
}
