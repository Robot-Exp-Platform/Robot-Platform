use manager::Node;

pub trait Planner: Node {
    fn name(&self) -> String;
}

pub trait DPlanner: Planner {}
