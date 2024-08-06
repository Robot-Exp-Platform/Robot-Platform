use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub enum Track {
    Joint(Vec<f64>),
}
