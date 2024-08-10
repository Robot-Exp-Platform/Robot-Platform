use nalgebra as na;

#[derive(Debug)]
pub enum CollisionObject {
    Sphere(Sphere),
    LineSegment(LineSegment),
    Capsule(Capsule),
}

#[derive(Debug)]
pub struct Sphere {
    pub center: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug)]
pub struct LineSegment {
    pub start: na::Point3<f64>,
    pub end: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug)]
pub struct Capsule {
    pub start: na::Point3<f64>,
    pub end: na::Point3<f64>,
    pub radius: f64,
}
