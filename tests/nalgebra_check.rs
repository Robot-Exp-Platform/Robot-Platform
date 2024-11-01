#![feature(more_float_constants)]

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use nalgebra as na;
    use rand::Rng;
    use std::{
        f64::consts::{SQRT_2, SQRT_3},
        time::Instant,
    };

    const N: usize = 10_000;

    #[test]
    fn test_transformations_consistency() {
        let mut rng = rand::thread_rng();

        let mut iso_list = Vec::new();
        let mut matrix4x4_list = Vec::new();
        let mut rotation_list = Vec::new();
        let mut translation_list = Vec::new();
        let mut point_list = Vec::new();

        for _ in 0..N {
            // Generate random Euler angles for rotation
            let roll = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
            let pitch = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);
            let yaw = rng.gen_range(-std::f64::consts::PI..std::f64::consts::PI);

            // Generate a random translation vector
            let tx = rng.gen_range(-100.0..100.0);
            let ty = rng.gen_range(-100.0..100.0);
            let tz = rng.gen_range(-100.0..100.0);

            iso_list.push(na::Isometry3::from_parts(
                na::Translation3::new(tx, ty, tz),
                na::UnitQuaternion::from_euler_angles(roll, pitch, yaw),
            ));

            matrix4x4_list.push(iso_list.last().unwrap().to_homogeneous());

            // Separate the immutable and mutable borrow operations
            rotation_list.push(iso_list.last().unwrap().rotation.to_rotation_matrix());

            translation_list.push(iso_list.last().unwrap().translation);

            // Generate a random point in 3D space
            point_list.push(na::Point3::new(
                rng.gen_range(-100.0..100.0),
                rng.gen_range(-100.0..100.0),
                rng.gen_range(-100.0..100.0),
            ));
        }

        // 对点变换测试
        println!("Start point transformation test");
        let mut ans_iso_list = Vec::new();
        let mut ans_matrix4x4_list = Vec::new();
        let mut ans_rotation_translation_list = Vec::new();

        let start_time = Instant::now();
        for i in 0..N {
            ans_iso_list.push(iso_list[i] * point_list[i]);
        }
        let elapsed_time = start_time.elapsed();
        println!("Elapsed time for isometry * point: {:?}", elapsed_time);

        let start_time = Instant::now();
        for i in 0..N {
            ans_matrix4x4_list.push(matrix4x4_list[i] * point_list[i].coords.push(1.0));
        }
        let elapsed_time = start_time.elapsed();
        println!("Elapsed time for matrix4x4 * point: {:?}", elapsed_time);

        let start_time = Instant::now();
        for i in 0..N {
            ans_rotation_translation_list
                .push(rotation_list[i] * point_list[i].coords + translation_list[i].vector);
        }
        let elapsed_time = start_time.elapsed();
        println!(
            "Elapsed time for rotation * point + translation: {:?}",
            elapsed_time
        );

        for i in 0..N {
            assert_relative_eq!(
                ans_iso_list[i].coords,
                ans_matrix4x4_list[i].xyz(),
                epsilon = 1e-6
            );
            assert_relative_eq!(
                ans_iso_list[i].coords,
                ans_rotation_translation_list[i],
                epsilon = 1e-6
            );
        }

        // 结合律测试
        println!("Start associativity test");
        let mut ans_iso_list = Vec::new();
        let mut ans_matrix4x4_list = Vec::new();
        let mut ans_rotation_translation_list = Vec::new();

        let start_time = Instant::now();
        for i in 0..N {
            ans_iso_list.push(iso_list[i] * iso_list[i]);
        }
        let elapsed_time = start_time.elapsed();
        println!("Elapsed time for isometry * isometry: {:?}", elapsed_time);

        let start_time = Instant::now();
        for i in 0..N {
            ans_matrix4x4_list.push(matrix4x4_list[i] * matrix4x4_list[i]);
        }
        let elapsed_time = start_time.elapsed();
        println!("Elapsed time for matrix4x4 * matrix4x4: {:?}", elapsed_time);

        let start_time = Instant::now();
        for i in 0..N {
            ans_rotation_translation_list.push((
                rotation_list[i] * rotation_list[i],
                rotation_list[i] * translation_list[i],
            ));
        }
        let elapsed_time = start_time.elapsed();
        println!(
            "Elapsed time for rotation * rotation , rotation * translation : {:?}",
            elapsed_time
        );

        for i in 0..N {
            assert_relative_eq!(
                ans_iso_list[i].to_homogeneous(),
                ans_matrix4x4_list[i],
                epsilon = 1e-6
            );
            assert_relative_eq!(
                ans_iso_list[i].rotation.to_rotation_matrix(),
                ans_rotation_translation_list[i].0,
                epsilon = 1e-6
            );
        }
    }

    #[test]
    fn cul_check() {
        let n1 = na::Vector3::new(-0.5, -SQRT_3 / 6.0, -SQRT_3 * SQRT_2 / 12.0);
        let n2 = na::Vector3::new(0.5, -SQRT_3 / 6.0, -SQRT_3 * SQRT_2 / 12.0);
        let n3 = na::Vector3::new(0.0, -SQRT_3 / 3.0, -SQRT_3 * SQRT_2 / 12.0);
        let n4 = na::Vector3::new(0.0, 0.0, -SQRT_3 * SQRT_2 / 4.0);

        // assert_eq!(n1.transpose() * n1, na::SVector::<f64, 1>::new(0.375));
        // assert_eq!(n2.transpose() * n2, na::SVector::<f64, 1>::new(0.375));
        // assert_eq!(n3.transpose() * n3, na::SVector::<f64, 1>::new(0.375));
        // assert_eq!(n4.transpose() * n4, na::SVector::<f64, 1>::new(0.375));

        let e =
            (n1 * n1.transpose() + n2 * n2.transpose() + n3 * n3.transpose() + n4 * n4.transpose())
                / 0.375;

        println!("{:?}", e);
    }

    #[test]
    fn cul_iso() {
        let iso1 = na::Isometry3::from_parts(
            na::Translation3::new(1.0, 2.0, 3.0),
            na::UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        );
        let iso2 = na::Isometry3::from_parts(
            na::Translation3::new(4.0, 5.0, 6.0),
            na::UnitQuaternion::from_euler_angles(0.4, 0.5, 0.6),
        );

        let iso3 = iso1 * iso2;

        println!("{:?}", iso3);
    }
}
