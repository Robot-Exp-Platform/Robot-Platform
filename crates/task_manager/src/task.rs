use message::target::Target;
use serde::{Deserialize, Serialize};
use serde_json::Value;
// use serde_yaml::Value;

#[derive(Debug, Serialize, Deserialize)]
pub struct RobotTasks {
    pub name: String,
    pub targets: Vec<Target>,
}
#[derive(Debug, Deserialize)]
pub struct Node {
    pub node_type: String,
    pub name: String,
    pub param: Value,
}

#[derive(Debug, Deserialize)]
pub struct Task {
    pub robot_tasks: Vec<RobotTasks>,
    pub nodes: Vec<Node>,
}
