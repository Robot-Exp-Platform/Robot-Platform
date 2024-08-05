use serde::Deserialize;

use robot::robot_trait::Pose;
#[derive(Debug, Deserialize)]
pub struct RobotTasks {
    pub name: String,
    pub targets: Vec<Pose>,
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
