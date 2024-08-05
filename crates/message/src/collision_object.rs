use nalgebra as na;

#[allow(dead_code)]
pub enum CollisionObject {
    Sphere(Sphere),
    LineSegment(LineSegment),
    Capsule(Capsule),
}

#[allow(dead_code)]
pub struct Sphere {
    center: na::Point3<f64>,
    radius: f64,
}

#[allow(dead_code)]
pub struct LineSegment {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}

#[allow(dead_code)]
pub struct Capsule {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}
