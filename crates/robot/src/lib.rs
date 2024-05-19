pub mod robot_trait;
pub mod robots;

#[cfg(test)]
mod tests {
    use super::*;
    use robot_trait::Robot;
    use robots::panda::Panda;
    use robots::robot_list::RobotList;

    #[test]
    fn init_a_robot() {
        let my_panda = Panda::new();
        print!("{}", my_panda.get_name());
    }

    #[test]
    fn init_robots() {
        let my_robots = RobotList::new_with_robots(
            "large_list".to_string(),
            vec![
                Box::new(Panda::new()),
                Box::new(RobotList::new_with_robots(
                    "small_list".to_string(),
                    vec![Box::new(Panda::new()), Box::new(Panda::new())],
                )),
            ],
        );
        print!("{}", my_robots.get_name());
    }
}
