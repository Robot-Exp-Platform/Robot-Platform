mod plant_trait;
mod plants;

use crate::plants::{first_order_lti::FirstOrderLTI, linear_system::LinearSystem};
use nalgebra as na;
use plant_trait::Plant;

#[cfg(target_os = "unix")]
use rosrust as ros;

fn main() {
    const N: usize = 7;
    let mut plant_list: Vec<Box<dyn Plant<N, N, N>>> = vec![Box::new(FirstOrderLTI::<N>::new())];
    plant_list.push(Box::new(LinearSystem::<N, N, N>::new()));

    for plant in &mut plant_list {
        plant.update(na::SVector::zeros());
        // 做想做的事情
    }

    #[cfg(target_os = "unix")]
    {
        ros::init("simuator");

        let pub_robot_state = ros::publish("robot_state", 100).unwrap();

        let mut plant = FirstOrderLTI::<7>::new();

        let rate = ros::rate(10.0);
        print!("ros is ok");
        while ros::is_ok() {
            let mut msg = ros_msg::std_msgs::String::default();

            plant.update(na::SVector::from_element(1.0));
            // plant.publish_state(&pub_robot_state);
            rate.sleep();
        }
    }
}
