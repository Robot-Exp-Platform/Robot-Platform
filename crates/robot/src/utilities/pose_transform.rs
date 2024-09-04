use nalgebra as na;

use message::Pose;

pub fn pose_to_svecter(pose: &Pose) -> na::SVector<f64, 6> {
    let mut dv = na::SVector::<f64, 6>::zeros();
    // 将 Unit<Quaternion<f64>> 转换为欧拉角
    let euler_angles = pose.rotation.euler_angles();
    dv[3] = euler_angles.0;
    dv[4] = euler_angles.1;
    dv[5] = euler_angles.2;
    dv
}
