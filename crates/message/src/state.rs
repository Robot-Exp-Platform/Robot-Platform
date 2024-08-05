pub enum State {
    RobotState(RobotState),
}

pub struct RobotState {
    pub q: Vec<f64>,
    pub q_dot: Vec<f64>,
}

// #[cfg(feature = "ros")]
// rosrust::rosmsg_include!(message / RobotState);
