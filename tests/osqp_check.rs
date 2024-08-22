#[cfg(tset)]
mod tests {
    #[test]
    fn osqp_check() {
        use osqp::{CscMatrix, Problem, Settings};

        // Define problem data
        let P = &[[4.0, 1.0], [1.0, 2.0]];
        let q = &[1.0, 1.0];
        let A = &[[1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        let l = &[1.0, 0.0, 0.0];
        let u = &[1.0, 0.7, 0.7];

        // Extract the upper triangular elements of `P`
        let P = CscMatrix::from(P).into_upper_tri();

        // Disable verbose output
        let settings = Settings::default().verbose(false);

        // Create an OSQP problem
        let mut prob = Problem::new(P, q, A, l, u, &settings).expect("failed to setup problem");

        // Solve problem
        let result = prob.solve();

        // Print the solution
        println!("{:?}", result.x().expect("failed to solve problem"));
    }
}
