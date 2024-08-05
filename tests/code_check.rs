use nalgebra as na;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector_addition() {
        let v1 = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let v2 = na::SVector::<f64, 3>::new(4.0, 5.0, 6.0);
        let expected_result = na::SVector::<f64, 3>::new(5.0, 7.0, 9.0);
        let result = v1 + v2;
        assert_eq!(result, expected_result);
    }

    #[test]
    fn test_matrix_multiplication() {
        let m1 = na::SMatrix::<f64, 2, 2>::new(1.0, 2.0, 3.0, 4.0);
        let m2 = na::SMatrix::<f64, 2, 2>::new(5.0, 6.0, 7.0, 8.0);
        let expected_result = na::SMatrix::<f64, 2, 2>::new(19.0, 22.0, 43.0, 50.0);
        let result = m1 * m2;
        assert_eq!(result, expected_result);
    }
}
