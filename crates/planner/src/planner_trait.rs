#[derive(Clone, Copy)]
pub enum PlannerState {
    Unknow,
    Uninit,
    Running,
}

pub trait Planner {
    fn get_planner_state(&self) -> PlannerState {
        PlannerState::Unknow
    }

    fn get_params(&self) -> Vec<f64>;

    // TODO add plan function
}
