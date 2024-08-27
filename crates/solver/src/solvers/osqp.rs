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

        let nonz_h: Vec<&f64> = problem.h.into_iter().filter(|&x| *x != 0.0).collect();
        let nonz_a: Vec<f64> = problem
            .constraints
            .to_inequation()
            .2
            .into_iter()
            .filter(|&x| x != 0.0)
            .collect();

        println!("h: {}", problem.h);
        println!("nonz_h longs {} {:?}", nonz_h.len(), nonz_h);
        println!("nonz_a longs {} {:?}", nonz_a.len(), nonz_a);

        println!("{:?}", p);
        println!("{:?}", a);

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
