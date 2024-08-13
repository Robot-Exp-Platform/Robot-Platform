use nalgebra as na;

pub fn interpolation<const N: usize>(
    start: &na::SVector<f64, N>,
    end: &na::SVector<f64, N>,
    interpolation: usize,
) -> Vec<na::SVector<f64, N>> {
    let mut track_list = Vec::new();
    for i in 0..interpolation {
        let mut track = na::SVector::from_vec(vec![0.0; N]);
        for j in 0..N {
            track[j] = start[j] + (end[j] - start[j]) * (i as f64 / interpolation as f64);
        }
        track_list.push(track);
    }
    track_list
}
