use crate::Solver;
use message::{Constraint, QuadraticProgramming};

pub struct OsqpSolver {
    problem: osqp::Problem,
}

impl OsqpSolver {
    pub fn from_problem(problem: QuadraticProgramming) -> OsqpSolver {
        let (_, _, a, l, u) = problem.constraints.to_cscmatrix();

        let p = problem.h.clone().into_upper_tri();
        let q = problem.f;

        // let (_, _, _a, _l, _u) = problem.constraints.to_namatrix();
        // println!("p: {:?}", p);
        // println!("q: {:?}", q);
        // println!("a: {:?}", _a);
        // println!("l: {:?}", _l);
        // println!("u: {:?}", _u);

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

    fn update_constraints(&mut self, constraints: Constraint) {
        let (_, _, a, l, u) = constraints.to_cscmatrix();
        self.problem.update_A(a);
        self.problem.update_bounds(&l, &u);
    }
}
