use nalgebra as na;

#[allow(dead_code)]
enum CillisionObject {
    Sphere(Sphere),
    LineSegment(LineSegment),
    Capsule(Capsule),
}

#[allow(dead_code)]
struct Sphere {
    center: na::Point3<f64>,
    radius: f64,
}

#[allow(dead_code)]
struct LineSegment {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}

#[allow(dead_code)]
struct Capsule {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}
