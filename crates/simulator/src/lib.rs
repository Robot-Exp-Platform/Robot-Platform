pub mod plant_trait;
pub mod plants;

#[cfg(test)]
mod tests {
    use nalgebra as na;

    #[test]
    fn judge_dmatrix() {
        // !只有静态矩阵才会在编译前检查结构，动态矩阵只能在运行时检查问题
        let a = na::SMatrix::<f64, 3, 4>::from_element(1.0);
        // let a_prime = na::SMatrix::<f64, 3, 3>::from_element(2.0);
        let b = na::SMatrix::<f64, 4, 3>::from_element(2.0);
        let c = na::DMatrix::from_element(3, 4, 3.0);
        let d = na::DMatrix::from_element(4, 3, 4.0);

        let a_dot_b = a * b;
        let c_dot_d = c * d;
        // !panic
        // let a_dot_c = a * c;
        // let a_dot_a_prime = a * a_prime;

        println!("{:?}", a_dot_b);
        println!("{:?}", c_dot_d);
        // print!("{:?}", a_dot_c);
    }
}
