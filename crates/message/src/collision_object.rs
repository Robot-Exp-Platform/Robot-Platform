use nalgebra as na;

#[derive(Debug)]
pub enum CollisionObject {
    Sphere(Sphere),
    Cylinder(Cylinder),
    Capsule(Capsule),
}

#[derive(Debug)]
pub struct Sphere {
    pub center: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug)]
pub struct Cylinder {
    pub start: na::Point3<f64>,
    pub end: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug)]
pub struct Capsule {
    pub ball_center1: na::Point3<f64>,
    pub ball_center2: na::Point3<f64>,
    pub radius: f64,
}
