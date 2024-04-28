mod plants;

use crate::plants::{first_order_lti::FirstOrderLTI, plant_trait::Plant};
use nalgebra as na;
use rosrust as ros;

fn main() {
    ros::init("simuator");

    // let pub_robot_state = ros::publish("robot_state", 100).unwrap();

    let mut plant = FirstOrderLTI::<7>::new();

    let rate = ros::rate(10.0);
    print!("ros is ok");

    while ros::is_ok() {
        print!("ros is ok");
        plant.update(na::SVector::from_element(1.0));
        // plant.publish_state(&pub_robot_state);
        rate.sleep();
    }
}
