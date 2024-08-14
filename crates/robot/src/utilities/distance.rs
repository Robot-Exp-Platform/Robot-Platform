fn get_joint_capsules_with_joint<const N: usize>(
    robot: dyn SeriesRobot<N>,
    joint: nalgebra::SVector<f64, N>,
) -> Vec<Capsule> {
    let nlink = N;
    let dh = robot.params.denavit_hartenberg;
    let mut joint_capsules = Vec::new();
    let mut isometry = self.state.base_pose;

    for i in 0..nlink + 1 {
        let isometry_increment = Isometry::from_parts(
            na::Translation3::new(
                dh[(i, 2)],
                -dh[(i, 1)] * dh[(i, 3)].sin(),
                dh[(i, 1)] * dh[(i, 3)].cos(),
            ),
            na::UnitQuaternion::from_euler_angles(joint[i], 0.0, dh[(i, 3)]),
        );

        // Update the cumulative transformation matrix
        isometry = isometry * isometry_increment;

        // Calculate the positions of the capsule's end points in the global frame
        let capsule_start = isometry * self.params.capsules[i].ball_center1;
        let capsule_end = isometry * self.params.capsules[i].ball_center1;

        // Create a new Capsule object and add it to the vector
        joint_capsules.push(Capsule {
            ball_center1: capsule_start,
            ball_center2: capsule_end,
            radius: self.params.capsules[i].radius,
        });
    }
    joint_capsules
}
fn get_distance_with_joint<const N: usize>(
    robot: dyn SeriesRobot<N>,
    joint: nalgebra::SVector<f64, N>,
    obj: &CollisionObject,
) -> f64 {
    let joint_capsules = get_joint_capsules_with_joint(joint);
    joint_capsules
        .iter()
        .map(|&x| get_distance(&CollisionObject::Capsule(x), obj))
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap()
}
fn get_distance_diff_with_joint<const N: usize>(
    robot: dyn SeriesRobot<N>,
    joint: nalgebra::SVector<f64, N>,
    bj: &CollisionObject,
) -> nalgebra::SVector<f64, N> {
    let mut distance_diff = na::SVector::from_element(0.0);
    let epsilon = 1e-6;
    for i in 0..N {
        let mut joint_plus = joint.clone();
        joint_plus[i] += epsilon;
        let mut joint_minus = joint.clone();
        joint_minus[i] -= epsilon;
        let distance_plus = self.get_distance_with_joint(robot, joint_plus, bj);
        let distance_minus = self.get_distance_with_joint(robot, joint_minus, bj);
        distance_diff[i] = (distance_plus - distance_minus) / (2.0 * epsilon);
    }
    distance_diff
}
