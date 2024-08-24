use crate::solver_trait::Solver;
use message::problem::QuadraticProgramming;
use osqp::CscMatrix;

pub struct OsqpSolver {
    problem: osqp::Problem,
}

impl OsqpSolver {
    pub fn from_problem(problem: QuadraticProgramming) -> OsqpSolver {
        let p = CscMatrix::from_column_iter_dense(1, 2, problem.h.iter().cloned()).into_upper_tri();
        let q = problem.f.as_slice();
        let (a, l, u) = problem.constraints.to_inequation();

        let settings = osqp::Settings::default().verbose(false);

        OsqpSolver {
            problem: osqp::Problem::new(p, q, &a, &l, &u, &settings)
                .expect("failed to setup problem"),
        }
    }
}

impl Solver for OsqpSolver {
    fn solve(&mut self) -> Vec<f64> {
        self.problem.solve().x().unwrap().to_vec()
    }
}
