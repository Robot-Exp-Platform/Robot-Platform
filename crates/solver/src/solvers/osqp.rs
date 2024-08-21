use crate::solver_trait::Solver;
use message::{problem::Problem, track::Track};

pub struct OsqpSolver {}

impl OsqpSolver {
    pub fn from_problem(_problem: Problem) -> OsqpSolver {
        unimplemented!()
    }
}

impl Solver for OsqpSolver {
    fn solve(&self) -> Vec<Track> {
        unimplemented!()
    }
}
