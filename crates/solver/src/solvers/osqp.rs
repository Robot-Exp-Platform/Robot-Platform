use crate::solver_trait::Solver;
use message::problem::QuadraticProgramming;

pub struct OsqpSolver {
    problem: osqp::Problem,
}

impl OsqpSolver {
    pub fn from_problem(problem: QuadraticProgramming) -> OsqpSolver {
        let (_, _, a, l, u) = problem.constraints.to_cscmatrix();

        let p = problem.get_csc_h().into_upper_tri();
        let q = problem.f.as_slice();

        // println!("{:?}", problem.h.as_slice());
        // println!("{:?}", problem.f.as_slice());
        // println!("{:?}", problem.constraints.to_inequation().2);
        // println!("{:?}", problem.constraints.to_inequation().3);
        // println!("{:?}", problem.constraints.to_inequation().4);

        let settings = osqp::Settings::default();

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
