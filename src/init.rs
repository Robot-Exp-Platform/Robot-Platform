use crate::exp::Exp;
use controller::config::{init_controller, ControllerConfig};
use planner::config::{init_planner, PlannerConfig};
use serde::{Deserialize, Serialize};
use std::fs;

#[derive(Serialize, Deserialize)]
struct Config {
    controller: Option<ControllerConfig>,
    planner: Option<PlannerConfig>,
}

impl Exp {
    pub fn init() -> Result<Exp, String> {
        let data = match fs::read_to_string("../config.json") {
            Ok(data) => data,
            Err(_) => return Err("read config error".to_string()),
        };

        let config: Config = match serde_json::from_str(&data) {
            Ok(config) => config,
            Err(_) => return Err("parse config error".to_string()),
        };

        let controller_exp = match init_controller(config.controller) {
            Ok(controller_exp) => controller_exp,
            Err(e) => return Err(e),
        };

        let planner_exp = match init_planner(config.planner) {
            Ok(planner_exp) => planner_exp,
            Err(e) => return Err(e),
        };

        Ok(Exp {
            controller_exp: controller_exp.unwrap(),
            planner_exp: planner_exp.unwrap(),
        })
    }
}
