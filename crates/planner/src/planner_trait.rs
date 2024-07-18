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
    fn get_name(&self) -> String;
    fn get_path(&self) -> String;
    fn get_params(&self) -> Vec<f64>;

    fn add_planner(&mut self, planner: Box<dyn Planner>);

    fn init(&self) {}
    fn start(&self) {}
    fn update(&mut self) {}

    // TODO add plan function
}
