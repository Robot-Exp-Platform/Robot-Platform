#[cfg(test)]
mod tests {
    use std::vec;

    use nalgebra as na;
    use serde_json;

    #[test]
    fn na_serialize_check() {
        let svec3 = na::SVector::<f64, 3>::from_vec(vec![1.0, 2.0, 3.0]);
        let dvec3 = na::DVector::<f64>::from_vec(vec![1.0, 2.0, 3.0]);
        let smat2x2 = na::SMatrix::<f64, 2, 2>::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let dmat2x2 = na::DMatrix::<f64>::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let iso3 = na::Isometry3::new(
            na::Vector3::new(1.0, 2.0, 3.0),
            na::Vector3::new(0.0, 0.0, 0.0),
        );

        let svec3_serialized = serde_json::to_string(&svec3).unwrap();
        let dvec3_serialized = serde_json::to_string(&dvec3).unwrap();
        let smat2x2_serialized = serde_json::to_string(&smat2x2).unwrap();
        let dmat2x2_serialized = serde_json::to_string(&dmat2x2).unwrap();
        let iso3_serialized = serde_json::to_string(&iso3).unwrap();

        let svec3_json: serde_json::Value = serde_json::to_value(&svec3).unwrap();
        let dvec3_json: serde_json::Value = serde_json::to_value(&dvec3).unwrap();
        let smat2x2_json: serde_json::Value = serde_json::to_value(&smat2x2).unwrap();
        let dmat2x2_json: serde_json::Value = serde_json::to_value(&dmat2x2).unwrap();
        let iso3_json: serde_json::Value = serde_json::to_value(&iso3).unwrap();

        println!("print SVector: {}", svec3);
        println!("serialized: {}", svec3_serialized);
        println!("json: {}", svec3_json);

        println!("print DVector: {}", dvec3);
        println!("serialized: {}", dvec3_serialized);
        println!("json: {}", dvec3_json);

        println!("print SMatrix: {}", smat2x2);
        println!("serialized: {}", smat2x2_serialized);
        println!("json: {}", smat2x2_json);

        println!("print DMatrix: {}", dmat2x2);
        println!("serialized: {}", dmat2x2_serialized);
        println!("json: {}", dmat2x2_json);

        println!("print Isometry3: {}", iso3);
        println!("serialized: {}", iso3_serialized);
        println!("json: {}", iso3_json);

        let svec3_deserialized: na::SVector<f64, 3> =
            serde_json::from_str(&svec3_serialized).unwrap();
        let dvec3_deserialized: na::DVector<f64> = serde_json::from_str(&dvec3_serialized).unwrap();
        let smat2x2_deserialized: na::SMatrix<f64, 2, 2> =
            serde_json::from_str(&smat2x2_serialized).unwrap();
        let dmat2x2_deserialized: na::DMatrix<f64> =
            serde_json::from_str(&dmat2x2_serialized).unwrap();
        let iso3_deserialized: na::Isometry3<f64> = serde_json::from_str(&iso3_serialized).unwrap();

        assert_eq!(svec3, svec3_deserialized);
        assert_eq!(dvec3, dvec3_deserialized);
        assert_eq!(smat2x2, smat2x2_deserialized);
        assert_eq!(dmat2x2, dmat2x2_deserialized);
        assert_eq!(iso3, iso3_deserialized);

        let svec3_deserialized_json: na::SVector<f64, 3> =
            serde_json::from_value(svec3_json).unwrap();
        let dvec3_deserialized_json: na::DVector<f64> = serde_json::from_value(dvec3_json).unwrap();
        let smat2x2_deserialized_json: na::SMatrix<f64, 2, 2> =
            serde_json::from_value(smat2x2_json).unwrap();
        let dmat2x2_deserialized_json: na::DMatrix<f64> =
            serde_json::from_value(dmat2x2_json).unwrap();
        let iso3_deserialized_json: na::Isometry3<f64> = serde_json::from_value(iso3_json).unwrap();

        assert_eq!(svec3, svec3_deserialized_json);
        assert_eq!(dvec3, dvec3_deserialized_json);
        assert_eq!(smat2x2, smat2x2_deserialized_json);
        assert_eq!(dmat2x2, dmat2x2_deserialized_json);
        assert_eq!(iso3, iso3_deserialized_json);
    }
}
