#[derive(Debug)]
pub enum ControlCommand {
    Joint(Vec<f64>),
}
