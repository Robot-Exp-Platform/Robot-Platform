#[cfg(test)]
mod tests {
    use nalgebra as na;
    use std::f64::consts::FRAC_PI_2;

    const PANDA_DOF: usize = 7;

    #[test]
    fn dh_check() {
        #[rustfmt::skip]
        let dh = na::DMatrix::from_row_slice(7, 4, &[
            0.0,  0.333,  0.0,     0.0,
            0.0,  0.0,    0.0,     -FRAC_PI_2,
            0.0,  0.316,  0.0,     FRAC_PI_2,
            0.0,  0.0,    0.0825,  FRAC_PI_2,
            0.0,  0.384,  -0.0825, -FRAC_PI_2,
            0.0,  0.0,    0.0,     FRAC_PI_2,
            0.0,  0.0,    0.088,   FRAC_PI_2,
        ]);
        println!("{}", dh);

        let q = na::DVector::from_vec(vec![
            0.3056, -0.6165, 0.9617, -2.1034, 0.2473, 2.0862, 1.2662,
        ]);

        let mut transform = na::Isometry3::identity();
        println!("{}", transform.to_homogeneous());
        for i in 0..PANDA_DOF {
            let d = dh[(i, 1)];
            let a = dh[(i, 2)];
            let alpha = dh[(i, 3)];
            let theta = q[i] + dh[(i, 0)];

            println!("d: {}, a: {}, alpha: {}, theta: {}", d, a, alpha, theta);

            let rot_z_theta =
                na::UnitQuaternion::<f64>::from_axis_angle(&na::Vector3::z_axis(), theta);
            let rot_x_alpha =
                na::UnitQuaternion::<f64>::from_axis_angle(&na::Vector3::x_axis(), alpha);
            let rotation = rot_z_theta * rot_x_alpha;

            // 计算平移向量
            // 先沿 Z 轴移动 d，再沿 X 轴移动 a
            let translation =
                na::Translation3::new(0.0, 0.0, d) * na::Translation3::new(a, 0.0, 0.0);

            // 当前关节的变换
            let current_transform = na::Isometry3::from_parts(translation, rotation);

            transform *= current_transform;
            println!("{}", transform.to_homogeneous());
        }
    }
}
