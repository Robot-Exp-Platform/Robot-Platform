use message::target::Target;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct RobotTasks {
    pub name: String,
    pub targets: Vec<Target>,
}
#[derive(Debug, Deserialize)]
pub struct Node {
    pub node_type: String,
    pub name: String,
    pub param: String,
}

#[derive(Debug, Deserialize)]
pub struct Task {
    pub robot_tasks: Vec<RobotTasks>,
    pub nodes: Vec<Node>,
}
