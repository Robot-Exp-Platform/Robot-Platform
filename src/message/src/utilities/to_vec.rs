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

pub fn vec_to_iso(vec: na::DVector<f64>) -> na::Isometry3<f64> {
    assert_eq!(vec.len(), 6);
    let quaternion = na::Quaternion {
        coords: na::Vector4::new(
            vec[0],
            vec[1],
            vec[2],
            -(1.0 - (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])).sqrt(),
        ),
    };
    println!("quaternion: {:?}", quaternion);

    let rotation = na::UnitQuaternion::from_quaternion(quaternion);
    let translation = na::Translation3::new(vec[3], vec[4], vec[5]);
    na::Isometry3::from_parts(translation, rotation)
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

    #[test]
    fn check_iso_vec() {
        let iso = na::Isometry3::new(
            na::Vector3::new(1.0, 2.0, 3.0),
            na::Vector3::new(4.0, 5.0, 6.0),
        );
        let vec = iso_to_vec(iso);
        println!("vec: {:?}", vec);
        let iso2 = vec_to_iso(vec);
        assert_eq!(iso, iso2);
    }
}
