pub mod robot_trait;
pub mod robots;
pub mod ros_thread;

pub use robot_trait::{Robot, RobotState};

#[cfg(test)]
mod tests {
    use super::*;
    use robot_trait::Robot;
    use robots::panda::Panda;
    // use robots::robot_list::RobotList;

    #[test]
    fn init_a_robot() {
        let my_panda = Panda::new("robot/".to_string());
        print!("{}", my_panda.get_name());
    }

    // #[test]
    // fn init_robots() {
    //     let my_robots = RobotList::new_with_robots(
    //         "large_list".to_string(),
    //         "robot/".to_string(),
    //         vec![
    //             Box::new(Panda::new("robot/panda1".to_string())),
    //             Box::new(RobotList::new_with_robots(
    //                 "small_list".to_string(),
    //                 "robot/large_list/".to_string(), // "robot/large_list/
    //                 vec![
    //                     Box::new(Panda::new("robot/panda2".to_string())),
    //                     Box::new(Panda::new("robot/panda3".to_string())),
    //                 ],
    //             )),
    //         ],
    //     );
    //     print!("{}", my_robots.get_name());
    // }
}
