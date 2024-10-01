use nalgebra as na;

/// 线性插值函数，用于在一系列点中实现插值，插值数为 ninterp
pub fn lerp(
    start: na::DVector<f64>,
    jointlist: &Vec<na::DVector<f64>>,
    ninterp: usize,
) -> Vec<na::DVector<f64>> {
    let mut track = Vec::new();
    let mut start = &start;
    for end in jointlist {
        for i in 0..ninterp {
            track.push(start + (end - start) * (i as f64) / (ninterp as f64));
        }
        start = end;
    }
    track
}
