use crate::exp::Exp;
use serde::{Deserialize, Serialize};
// use std::fs;

#[derive(Serialize, Deserialize)]
struct Config {
    name: String,
    robot_type: String,
    controller: String,
    planner: String,
    robots: Option<Vec<Config>>,
}

impl Exp {}
