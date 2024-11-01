use nalgebra as na;

pub fn iso_to_vec(iso: na::Isometry3<f64>) -> na::DVector<f64> {
    let rotation = iso.rotation.vector();
    let translation = iso.translation.vector;
    na::DVector::from_vec(vec![
        rotation[0],
        rotation[1],
        rotation[2],
        translation[0],
        translation[1],
        translation[2],
    ])
}
