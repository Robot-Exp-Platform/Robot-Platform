use nalgebra as na;

#[derive(Debug)]
pub enum CollisionObject {
    Sphere(Sphere),
    Cylinder(Cylinder),
    Capsule(Capsule),
    Cuboid(Cuboid),
    Cone(Cone),
}

#[derive(Debug, Clone, Copy)]
pub struct Sphere {
    pub center: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Cylinder {
    pub start: na::Point3<f64>,
    pub end: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Capsule {
    pub ball_center1: na::Point3<f64>,
    pub ball_center2: na::Point3<f64>,
    pub radius: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct Cuboid {
    pub point1: na::Point3<f64>,
    pub point2: na::Point3<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct Cone {
    pub bottom_center: na::Point3<f64>,
    pub up_point: na::Point3<f64>,
    pub radius: f64,
}

impl Capsule {
    pub fn from_vec(vec: Vec<f64>) -> Capsule {
        Capsule {
            ball_center1: na::Point3::new(vec[0], vec[1], vec[2]),
            ball_center2: na::Point3::new(vec[3], vec[4], vec[5]),
            radius: vec[6],
        }
    }
}

pub fn get_distance(a: &CollisionObject, b: &CollisionObject) -> f64 {
    // 计算两个障碍物之间的距离
    match (a, b) {
        (CollisionObject::Sphere(a), CollisionObject::Sphere(b)) => {
            // 计算球体与球体之间的距离
            sphere_sphere_distance(a, b)
        }
        (CollisionObject::Capsule(a), CollisionObject::Capsule(b)) => {
            // 计算胶囊与胶囊之间的距离
            capsule_capsule_distance(a, b)
        }
        (CollisionObject::Cylinder(a), CollisionObject::Cylinder(b)) => {
            // 计算圆柱与圆柱之间的距离
            cylinder_cylinder_distance(a, b)
        }

        (CollisionObject::Sphere(a), CollisionObject::Capsule(b))
        | (CollisionObject::Capsule(b), CollisionObject::Sphere(a)) => {
            // 计算球体与胶囊之间的距离
            sphere_capsule_distance(a, b)
        }
        (CollisionObject::Sphere(a), CollisionObject::Cylinder(b))
        | (CollisionObject::Cylinder(b), CollisionObject::Sphere(a)) => {
            // 计算球体与圆柱之间的距离
            sphere_cylinder_distance(a, b)
        }
        (CollisionObject::Capsule(a), CollisionObject::Cylinder(b))
        | (CollisionObject::Cylinder(b), CollisionObject::Capsule(a)) => {
            // 计算胶囊与圆柱之间的距离
            capsule_cylinder_distance(a, b)
        }

        // 圆柱的暂未实现
        _ => 0.0,
    }
}

fn sphere_sphere_distance(a: &Sphere, b: &Sphere) -> f64 {
    let center_distance = (a.center - b.center).norm();
    (center_distance - a.radius - b.radius).max(0.0)
}

fn sphere_capsule_distance(sphere: &Sphere, capsule: &Capsule) -> f64 {
    let closest_point = get_closest_point_on_line_segment(
        sphere.center,
        capsule.ball_center1,
        capsule.ball_center2,
    );
    let center_distance = (sphere.center - closest_point).norm();
    (center_distance - sphere.radius - capsule.radius).max(0.0)
}

fn capsule_capsule_distance(a: &Capsule, b: &Capsule) -> f64 {
    let start1 = a.ball_center1;
    let end1 = a.ball_center2;
    let start2 = b.ball_center1;
    let end2 = b.ball_center2;

    let (_, _, distance_sqr) = get_closest_points_between_lines(start1, end1, start2, end2);

    let distance = distance_sqr.sqrt(); // 对平方距离开平方，得到实际距离

    (distance - a.radius - b.radius).max(0.0)
}

fn cylinder_cylinder_distance(a: &Cylinder, b: &Cylinder) -> f64 {
    let start1 = a.start;
    let end1 = a.end;
    let start2 = b.start;
    let end2 = b.end;

    let (_, _, distance_sqr) = get_closest_points_between_lines(start1, end1, start2, end2);

    let distance = distance_sqr.sqrt();

    (distance - a.radius - b.radius).max(0.0)
}

fn sphere_cylinder_distance(sphere: &Sphere, cylinder: &Cylinder) -> f64 {
    let closest_point =
        get_closest_point_on_line_segment(sphere.center, cylinder.start, cylinder.end);
    let center_distance = (sphere.center - closest_point).norm();
    (center_distance - sphere.radius - cylinder.radius).max(0.0)
}

fn capsule_cylinder_distance(capsule: &Capsule, cylinder: &Cylinder) -> f64 {
    let start1 = capsule.ball_center1;
    let end1 = capsule.ball_center2;
    let start2 = cylinder.start;
    let end2 = cylinder.end;

    let (_, _, distance_sqr) = get_closest_points_between_lines(start1, end1, start2, end2);

    let distance = distance_sqr.sqrt();

    (distance - capsule.radius - cylinder.radius).max(0.0)
}

/// 计算两条线段之间的最短距离的平方，并返回最近的两个点
fn get_closest_points_between_lines(
    start1: na::Point3<f64>,
    end1: na::Point3<f64>,
    start2: na::Point3<f64>,
    end2: na::Point3<f64>,
) -> (na::Point3<f64>, na::Point3<f64>, f64) {
    let line1 = end1 - start1;
    let line2 = end2 - start2;

    // 判断完全平行
    let is_parallel = line1.normalize() == line2.normalize();
    let (closest_point1, closest_point2, dis_sqr): (na::Point3<f64>, na::Point3<f64>, f64);

    if is_parallel {
        // 完全平行
        let len1 = line1.norm_squared();
        let len2 = line2.norm_squared();

        let dis_start;
        let dis_end;

        if len1 > len2 {
            let cp_start = get_closest_point_on_line_segment(start1, end1, start2);
            let cp_end = get_closest_point_on_line_segment(start1, end1, end2);
            dis_start = (cp_start - start2).norm_squared();
            dis_end = (cp_end - end2).norm_squared();

            if dis_start < dis_end {
                closest_point1 = cp_start;
                closest_point2 = start2;
            } else {
                closest_point1 = cp_end;
                closest_point2 = end2;
            }
        } else {
            let cp_start = get_closest_point_on_line_segment(start2, end2, start1);
            let cp_end = get_closest_point_on_line_segment(start2, end2, end1);
            dis_start = (cp_start - start1).norm_squared();
            dis_end = (cp_end - end1).norm_squared();

            if dis_start < dis_end {
                closest_point1 = start1;
                closest_point2 = cp_start;
            } else {
                closest_point1 = end1;
                closest_point2 = cp_end;
            }
        }

        dis_sqr = dis_start.min(dis_end);
    } else {
        let normal = line1.cross(&line2);
        let len = normal.norm_squared();
        let dis2_line = (start2 - start1).dot(&normal).abs().powi(2) / len;

        // 判断是否在同一个平面上
        if dis2_line == 0.0 {
            // 同面
            // 检测线段是否相交
            let is_line_cross = check_line_cross(start1, end1, start2, end2);
            if is_line_cross {
                closest_point1 = start1;
                closest_point2 = start2;
                dis_sqr = 0.0;
            } else {
                let (cp1, d1) = get_closest_point_on_segments(start1, end1, start2, end2);
                let (cp2, d2) = get_closest_point_on_segments(start2, end2, start1, end1);
                if d1 < d2 {
                    closest_point1 = cp1.0;
                    closest_point2 = cp1.1;
                    dis_sqr = d1;
                } else {
                    closest_point1 = cp2.1;
                    closest_point2 = cp2.0;
                    dis_sqr = d2;
                }
            }
        } else {
            let offset = dis2_line.sqrt();
            // 计算line2相对line1的方向
            let direction_start = start2 - start1;
            let direction = if direction_start.dot(&normal) > 0.0 {
                1.0
            } else {
                -1.0
            };

            // 检测线段是否相交
            let is_line_cross = check_line_cross(
                start1,
                end1,
                start2 - normal.normalize() * (offset * direction),
                end2 - normal.normalize() * (offset * direction),
            );

            if is_line_cross {
                closest_point1 = start1;
                closest_point2 = start2;
                dis_sqr = dis2_line;
            } else {
                let (cp1, d1) = get_closest_point_on_segments(start1, end1, start2, end2);
                let (cp2, d2) = get_closest_point_on_segments(start2, end2, start1, end1);
                if d1 < d2 {
                    closest_point1 = cp1.0;
                    closest_point2 = cp1.1;
                    dis_sqr = d1;
                } else {
                    closest_point1 = cp2.1;
                    closest_point2 = cp2.0;
                    dis_sqr = d2;
                }
            }
        }
    }

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

/// 检测两条线段是否相交
fn check_line_cross(
    start1: na::Point3<f64>,
    end1: na::Point3<f64>,
    start2: na::Point3<f64>,
    end2: na::Point3<f64>,
) -> bool {
    let dir1 = end1 - start1;
    let dir2 = end2 - start2;

    let cross = dir1.cross(&dir2);
    let denom = cross.norm_squared();

    // 处理平行或共线情况
    if denom == 0.0 {
        return false;
    }

    let w = start2 - start1;
    let t1 = w.cross(&dir2).dot(&cross) / denom;
    let t2 = w.cross(&dir1).dot(&cross) / denom;

    t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0
}

/// 计算两个线段之间最近的点对及其距离平方
fn get_closest_point_on_segments(
    start1: na::Point3<f64>,
    end1: na::Point3<f64>,
    start2: na::Point3<f64>,
    end2: na::Point3<f64>,
) -> ((na::Point3<f64>, na::Point3<f64>), f64) {
    let cp1_start = get_closest_point_on_line_segment(start1, end1, start2);
    let d1_start = (cp1_start - start2).norm_squared();

    let cp1_end = get_closest_point_on_line_segment(start1, end1, end2);
    let d1_end = (cp1_end - end2).norm_squared();

    let cp2_start = get_closest_point_on_line_segment(start2, end2, start1);
    let d2_start = (cp2_start - start1).norm_squared();

    let cp2_end = get_closest_point_on_line_segment(start2, end2, end1);
    let d2_end = (cp2_end - end1).norm_squared();

    if d1_start < d1_end && d1_start < d2_start && d1_start < d2_end {
        ((cp1_start, start2), d1_start)
    } else if d1_end < d1_start && d1_end < d2_start && d1_end < d2_end {
        ((cp1_end, end2), d1_end)
    } else if d2_start < d1_start && d2_start < d1_end && d2_start < d2_end {
        ((start1, cp2_start), d2_start)
    } else {
        ((end1, cp2_end), d2_end)
    }
}
