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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exchange_div_vec() {
        let iso1 = na::Isometry3::new(
            na::Vector3::new(1.0, 2.0, 3.0),
            na::Vector3::new(4.0, 5.0, 6.0),
        );
        let iso2 = na::Isometry3::new(
            na::Vector3::new(7.0, 8.0, 9.0),
            na::Vector3::new(10.0, 11.0, 12.0),
        );
        let vec1 = iso_to_vec(iso1);
        let vec2 = iso_to_vec(iso2);
        println!("iso1/iso2             : {:?}", iso1 / iso2);
        println!("iso1.inv_mul(iso2)    : {:?}", iso1.inv_mul(&iso2));
        println!("iso1 * iso2.inverse() : {:?}", iso1 * iso2.inverse());
        assert_eq!(vec1.clone() - vec2.clone(), iso_to_vec(iso1 / iso2));
        assert_eq!(vec1.clone() - vec2.clone(), iso_to_vec(iso1.inv_mul(&iso2)));
        assert_eq!(
            vec1.clone() - vec2.clone(),
            iso_to_vec(iso1 * iso2.inverse())
        );
    }
}
