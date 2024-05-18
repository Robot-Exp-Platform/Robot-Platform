use crate::robot_trait::Robot;

pub struct RobotList {
    name: String,
    pub robots: Vec<Box<dyn Robot>>,
}

impl RobotList {
    pub fn new(name: String) -> RobotList {
        RobotList {
            name,
            robots: Vec::new(),
        }
    }

    pub fn new_with_robots(name: String, robots: Vec<Box<dyn Robot>>) -> RobotList {
        RobotList { name, robots }
    }
}

impl Robot for RobotList {
    fn get_name(&self) -> String {
        let names = self
            .robots
            .iter()
            .map(|robot| robot.get_name())
            .collect::<Vec<_>>()
            .join(", ");

        format!("{}:{{{}}}", self.name, names)
    }
}
