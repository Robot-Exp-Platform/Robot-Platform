pub mod solver_trait;
pub mod solvers;

#[cfg(test)]
mod tests {
    #[test]
    fn osqp_check() {
        use osqp::{CscMatrix, Problem, Settings};

        // Define problem data
        let p = &[[4.0, 1.0], [1.0, 2.0]];
        let q = &[1.0, 1.0];
        let a = &[[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        let l = &[1.0, 0.0, 0.0];
        let u = &[1.0, 0.7, 0.7];

        // Extract the upper triangular elements of `p`
        let p = CscMatrix::from(p).into_upper_tri();

        // Disable verbose output
        let settings = Settings::default().verbose(false);

        // Create an OSQp problem
        let mut prob = Problem::new(p, q, a, l, u, &settings).expect("failed to setup problem");

        // Solve problem
        let result = prob.solve();

        // print the solution
        println!("{:?}", result.x().expect("failed to solve problem"));
    }
}
