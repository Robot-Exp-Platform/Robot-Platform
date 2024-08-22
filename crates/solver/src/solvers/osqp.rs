use crate::solver_trait::Solver;
use message::{problem::QuadraticProgramming, track::Track};

pub struct OsqpSolver {
    problem: osqp::Problem,
}

impl OsqpSolver {
    pub fn from_problem(_problem: QuadraticProgramming) -> OsqpSolver {
        unimplemented!()
    }
}

impl Solver for OsqpSolver {
    fn solve(&self) -> Vec<Track> {
        unimplemented!()
    }
}
