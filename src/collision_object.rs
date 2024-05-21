use nalgebra as na;

enum CillisionObject {
    Sphere(Sphere),
    LineSegment(LineSegment),
    Capsule(Capsule),
}

struct Sphere {
    center: na::Point3<f64>,
    radius: f64,
}

struct LineSegment {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}

struct Capsule {
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    radius: f64,
}
