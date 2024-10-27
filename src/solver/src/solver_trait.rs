use message::Constraint;

pub trait Solver {
    fn solve(&mut self) -> Vec<f64>;

    fn update_constraints(&mut self, constraints: Constraint);
}
