use message::target::Target;
use serde::Deserialize;
use serde_json::Value as JsonValue;

#[derive(Debug, Deserialize)]
pub struct RobotTasks {
    pub name: String,
    pub targets: Vec<Target>,
}
#[derive(Debug, Deserialize)]
pub struct Node {
    pub node_type: String,
    pub name: String,
    pub param: JsonValue,
}

#[derive(Debug, Deserialize)]
pub struct Task {
    pub robot_tasks: Vec<RobotTasks>,
    pub nodes: Vec<Node>,
}
