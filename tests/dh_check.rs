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
            -0.5873950719833374,
            -0.15685543417930603,
            0.7545418739318848,
            -2.6158623695373535,
            0.2868352234363556,
            4.088727951049805,
            0.5751898288726807,
        ]);

        let mut transform = na::Isometry3::identity();
        // println!("{}", transform.to_homogeneous());
        for i in 0..PANDA_DOF {
            let d = dh[(i, 1)];
            let a = dh[(i, 2)];
            let alpha = dh[(i, 3)];
            let theta = q[i] + dh[(i, 0)];

            // println!("d: {}, a: {}, alpha: {}, theta: {}", d, a, alpha, theta);

            let rotation = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), alpha)
                * na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), theta);

            let isometry_increment = na::Isometry3::from_parts(
                na::Translation3::new(a, -d * alpha.sin(), d * alpha.cos()),
                rotation,
            );

            // 当前关节的变换
            let current_transform = isometry_increment;
            // println!("current_transform: {}", current_transform.to_homogeneous());
            transform *= current_transform;
            // println!("transform: {}", transform.to_homogeneous());
        }

        println!("final transform: {}", transform);
        println!("final transform: {}", transform.to_homogeneous());
    }

    #[test]
    fn rotation_check() {
        let alpha = FRAC_PI_2;
        let theta = FRAC_PI_2;
        let rotation = na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), alpha)
            * na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), theta);
        println!("{}", rotation);
        println!(
            "{}",
            rotation.to_rotation_matrix() * rotation.to_rotation_matrix().transpose()
        );
    }

    #[test]
    fn homogeneous_check() {
        let alpha = FRAC_PI_2;
        let theta = FRAC_PI_2;
        let transform = na::Isometry3::from_parts(
            na::Translation3::new(1.0, 2.0, 3.0),
            na::UnitQuaternion::from_axis_angle(&na::Vector3::x_axis(), alpha)
                * na::UnitQuaternion::from_axis_angle(&na::Vector3::z_axis(), theta),
        );
        println!("{}", transform.to_homogeneous());
        println!(
            "{}",
            transform.to_homogeneous().transpose() * transform.to_homogeneous()
        );
    }
}
