use nalgebra as na;

pub trait Plant<const N: usize, const I: usize, const O: usize> {
    fn get_state(&self) -> (na::SVector<f64, N>, na::SVector<f64, N>);
    fn init_state(&mut self, q: na::SVector<f64, N>, q_dot: na::SVector<f64, N>);
    fn update(&mut self, u: na::SVector<f64, I>) -> na::SVector<f64, O>;

    // fn publish_state(&self, pub_robot_state: &ros::Publisher);
}
