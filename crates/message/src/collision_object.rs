use nalgebra as na;

#[derive(Debug, Clone, Copy)]
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

fn is_equal(x:f64, y:f64) -> bool{
    if (x - y).abs() < 1e-7{
        return true
    }
    false
}

/// 计算两条线段之间的最短距离的平方，并返回最近的两个点
fn get_closest_points_between_lines(
    start1: na::Point3<f64>,
    end1: na::Point3<f64>,
    start2: na::Point3<f64>,
    end2: na::Point3<f64>,
) -> (na::Point3<f64>, na::Point3<f64>, f64){
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
    } 
    else {
        sn = b * e - c * d;
        tn = a * e - b * d;
        if sn < 0.0 {
            // 最近点在s起点之外，同平行处理
            sn = 0.0;
            tn = e;
            td = c;
        }
        else if sn > sd {
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
        }
        else if -d > a{
            sn = sd;
        }
        else{
            sn = -d;
            sd = a;
        }
    }
    else if tn > td {
        tn = td;
        if (-d + b) < 0.0 {
            sn = 0.0;
        }
        else if (-d + b) > a {
            sn = sd;
        }
        else {
            sn = -d + b;
            sd = a;
        }
    }

    let sc = if is_equal(sn, 0.0){
        0.0
    }
    else{
        sn / sd
    };

    let tc = if is_equal(tn, 0.0){
        0.0
    }
    else{
        tn / td
    };

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
