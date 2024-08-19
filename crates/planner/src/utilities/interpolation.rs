use nalgebra as na;

// 在 start 和 end 之间进行插值为 interpolation 段
pub fn interpolation<const N: usize>(
    start: &na::SVector<f64, N>,
    end: &na::SVector<f64, N>,
    interpolation: usize,
) -> Vec<na::SVector<f64, N>> {
    let mut track_list = Vec::new();
    for i in 1..interpolation {
        let track = start + (end - start) * (i as f64 / interpolation as f64);
        track_list.push(track);
    }
    track_list
}
