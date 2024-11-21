use crate::Pose;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub enum CollisionObject {
    Sphere(Sphere),
    Cylinder(Cylinder),
    Capsule(Capsule),
    Cuboid(Cuboid),
    Cone(Cone),
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Collision<T> {
    pub id: usize,
    pub pose: Pose,
    pub params: T,
}

type Radius = f64;
type Leight = f64;

pub type Sphere = Collision<Radius>;
pub type Cylinder = Collision<(Radius, Leight)>;
pub type Cone = Collision<(Radius, Leight)>;
pub type Capsule = Collision<(Radius, Leight)>;
pub type Cuboid = Collision<(Leight, Leight, Leight)>;

impl<T> Collision<T> {
    pub fn new(id: usize, pose: Pose, params: T) -> Collision<T> {
        Collision { id, pose, params }
    }
}

impl Capsule {
    // impl Capsule {
    pub fn from_vec(vec: Vec<f64>) -> Capsule {
        let axisangle = na::Vector3::z();
        let translation = na::Vector3::new(vec[0], vec[1], vec[2]);
        let end = na::Vector3::new(vec[3], vec[4], vec[5]);

        let length = (translation - end).norm();
        let radius = vec[6];

        Capsule {
            id: 0,
            pose: Pose::new(translation, axisangle),
            params: (radius, length),
        }
    }
}

impl CollisionObject {
    pub fn get_distance(a: &Self, b: &Self) -> f64 {
        // 计算两个障碍物之间的距离
        match (a, b) {
            (Self::Sphere(a), Self::Sphere(b)) => sphere_sphere_distance(a, b),
            (Self::Capsule(a), Self::Capsule(b)) => capsule_capsule_distance(a, b),

            (Self::Sphere(a), Self::Capsule(b)) | (Self::Capsule(b), Self::Sphere(a)) => {
                sphere_capsule_distance(a, b)
            }

            // 圆柱的暂未实现
            _ => 0.0,
        }
    }
}

fn sphere_sphere_distance(a: &Sphere, b: &Sphere) -> f64 {
    a.pose.inv_mul(&b.pose).translation.vector.norm() - a.params - b.params
}
fn sphere_capsule_distance(sphere: &Sphere, capsule: &Capsule) -> f64 {
    let sphere_center = sphere.pose.translation.vector;
    let capsule_start = capsule.pose.translation.vector;
    let capsule_end = capsule.pose * na::Vector3::new(0.0, capsule.params.1, 0.0);

    let closest_point = get_closest_point_on_line_segment(
        na::Point3::from(capsule_start),
        na::Point3::from(capsule_end),
        na::Point3::from(sphere_center),
    );

    let center_distance = (na::Point3::from(sphere_center) - closest_point).norm();
    (center_distance - sphere.params - capsule.params.0).max(0.0)
}

fn capsule_capsule_distance(a: &Capsule, b: &Capsule) -> f64 {
    let start1 = a.pose.translation.vector;
    let end1 = a.pose * na::Vector3::new(0.0, a.params.1, 0.0);
    let start2 = b.pose.translation.vector;
    let end2 = b.pose * na::Vector3::new(0.0, b.params.1, 0.0);

    let (_, _, distance_sqr) = get_closest_points_between_lines(
        na::Point3::from(start1),
        na::Point3::from(end1),
        na::Point3::from(start2),
        na::Point3::from(end2),
    );

    let distance = distance_sqr.sqrt(); // 对平方距离开平方，得到实际距离

    (distance - a.params.0 - b.params.0).max(0.0)
}

fn is_equal(x: f64, y: f64) -> bool {
    if (x - y).abs() < 1e-7 {
        return true;
    }
    false
}

/// 计算两条线段之间的最短距离的平方，并返回最近的两个点
fn get_closest_points_between_lines(
    start1: na::Point3<f64>,
    end1: na::Point3<f64>,
    start2: na::Point3<f64>,
    end2: na::Point3<f64>,
) -> (na::Point3<f64>, na::Point3<f64>, f64) {
    // ref: https://www.cnblogs.com/zmy--blog/p/15049721.html
    let u = end1 - start1;
    let v = end2 - start2;
    let w = start1 - start2;
    let a = u.x * u.x + u.y * u.y + u.z * u.z; // u*u
    let b = u.x * v.x + u.y * v.y + u.z * v.z; // u*v
    let c = v.x * v.x + v.y * v.y + v.z * v.z; // v*v
    let d = u.x * w.x + u.y * w.y + u.z * w.z; // u*w
    let e = v.x * w.x + v.y * w.y + v.z * w.z; // v*w
    let dt = a * c - b * b;
    let mut sd = dt;
    let mut td = dt;
    let mut sn; // sn = be - cd
    let mut tn; // tn - ae - bd

    if is_equal(dt, 0.0) {
        // 两直线平行
        sn = 0.0;
        sd = 1.00; // 防止除以0
        tn = e;
        td = c;
    } else {
        sn = b * e - c * d;
        tn = a * e - b * d;
        if sn < 0.0 {
            // 最近点在s起点之外，同平行处理
            sn = 0.0;
            tn = e;
            td = c;
        } else if sn > sd {
            // 最近点在s终点以外
            sn = sd;
            tn = e + b;
            td = c;
        }
    }

    if tn < 0.0 {
        // 最近点在t起点之外
        tn = 0.0;
        if -d < 0.0 {
            sn = 0.0;
        } else if -d > a {
            sn = sd;
        } else {
            sn = -d;
            sd = a;
        }
    } else if tn > td {
        tn = td;
        if (-d + b) < 0.0 {
            sn = 0.0;
        } else if (-d + b) > a {
            sn = sd;
        } else {
            sn = -d + b;
            sd = a;
        }
    }

    let sc = if is_equal(sn, 0.0) { 0.0 } else { sn / sd };

    let tc = if is_equal(tn, 0.0) { 0.0 } else { tn / td };

    let dx = w.x + (sc * u.x) - (tc * v.x);
    let dy = w.y + (sc * u.y) - (tc * v.y);
    let dz = w.z + (sc * u.z) - (tc * v.z);
    let dis_sqr = dx * dx + dy * dy + dz * dz;

    let cp1_x = start1.x + (sc * u.x);
    let cp1_y = start1.y + (sc * u.y);
    let cp1_z = start1.z + (sc * u.z);
    let cp2_x = start2.x + (tc * v.x);
    let cp2_y = start2.y + (tc * v.y);
    let cp2_z = start2.z + (tc * v.z);
    let closest_point1 = na::Point3::new(cp1_x, cp1_y, cp1_z);
    let closest_point2 = na::Point3::new(cp2_x, cp2_y, cp2_z);

    (closest_point1, closest_point2, dis_sqr)
}

/// 获取线段上距离某点最近的点
fn get_closest_point_on_line_segment(
    start: na::Point3<f64>,
    end: na::Point3<f64>,
    point: na::Point3<f64>,
) -> na::Point3<f64> {
    let line = end - start;
    let t = (point - start).dot(&line) / line.norm_squared();
    start + line * t.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    #[test]
    fn collision_object_to_json() {}
}
