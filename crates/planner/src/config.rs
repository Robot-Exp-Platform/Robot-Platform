use crate::planner_trait::Planner;
use crate::planners::linear::{Linear, LinearParams};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct PlannerConfig {
    pub planner_type: String,
    pub params: serde_json::Value, // 使用 serde_json::Value 来灵活处理不同的参数
}

pub fn init_planner(config: Option<PlannerConfig>) -> Result<Option<Box<dyn Planner>>, String> {
    match config {
        Some(planner_config) => match planner_config.planner_type.as_str() {
            "linear" => match serde_json::from_value::<LinearParams>(planner_config.params) {
                Ok(linear_params) => Ok(Some(Box::new(Linear::new(linear_params)))),
                Err(_) => Err("Failed to parse Linear parameters".to_string()),
            },
            _ => Err("unknow planner".to_string()),
        },
        None => Ok(None),
    }
}
