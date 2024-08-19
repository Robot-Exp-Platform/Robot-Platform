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

    use serde::{Deserialize, Serialize};
    #[test]
    fn enum_serialize_check() {
        #[derive(Serialize, Deserialize, Debug, PartialEq)]
        enum MyEnum {
            Int(i32),
            Float(f64),
            String(String),
        }

        let int = MyEnum::Int(42);
        let float = MyEnum::Float(3.14);
        let string = MyEnum::String("hello".to_string());

        let int_serialized_json = serde_json::to_string(&int).unwrap();
        let float_serialized_json = serde_json::to_string(&float).unwrap();
        let string_serialized_json = serde_json::to_string(&string).unwrap();

        let int_serialized_yaml = serde_yaml::to_string(&int).unwrap();
        let float_serialized_yaml = serde_yaml::to_string(&float).unwrap();
        let string_serialized_yaml = serde_yaml::to_string(&string).unwrap();

        let int_json: serde_json::Value = serde_json::to_value(&int).unwrap();
        let float_json: serde_json::Value = serde_json::to_value(&float).unwrap();
        let string_json: serde_json::Value = serde_json::to_value(&string).unwrap();

        let int_yaml: serde_yaml::Value = serde_yaml::to_value(&int).unwrap();
        let float_yaml: serde_yaml::Value = serde_yaml::to_value(&float).unwrap();
        let string_yaml: serde_yaml::Value = serde_yaml::to_value(&string).unwrap();

        println!("print Int: {:?}", int);
        println!("serialized json: {}", int_serialized_json);
        println!("serialized yaml: {}", int_serialized_yaml);
        println!("json: {}", int_json);
        println!("yaml: {:?}", int_yaml);

        println!("print Float: {:?}", float);
        println!("serialized json: {}", float_serialized_json);
        println!("serialized yaml: {}", float_serialized_yaml);
        println!("json: {}", float_json);
        println!("yaml: {:?}", float_yaml);

        println!("print String: {:?}", string);
        println!("serialized json: {}", string_serialized_json);
        println!("serialized yaml: {}", string_serialized_yaml);
        println!("json: {}", string_json);
        println!("yaml: {:?}", string_yaml);
    }

    #[test]
    fn enum_vec_serialize_check() {
        #[derive(Serialize, Deserialize, Debug, PartialEq)]
        enum MyEnum {
            Int(i32),
            Float(f64),
            String(String),
        }

        let vec = vec![
            MyEnum::Int(42),
            MyEnum::Float(3.14),
            MyEnum::String("hello".to_string()),
        ];

        let vec_serialized = serde_json::to_string(&vec).unwrap();
        let vec_json: serde_json::Value = serde_json::to_value(&vec).unwrap();

        println!("print Vec: {:?}", vec);
        println!("serialized: {}", vec_serialized);
        println!("json: {}", vec_json);
    }

    #[test]
    fn enum_target_serialize() {
        use message::target::Target;

        let target_vec = vec![
            Target::Joint(vec![0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]),
            Target::Joint(vec![0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]),
            Target::Joint(vec![0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]),
        ];

        let target_vec_serialized = serde_json::to_string(&target_vec).unwrap();
        let target_vec_json: serde_json::Value = serde_json::to_value(&target_vec).unwrap();

        println!("print Vec: {:?}", target_vec);
        println!("serialized: {}", target_vec_serialized);
        println!("json: {}", target_vec_json);
    }

    #[test]
    fn task_serialize_check() {}

    #[test]
    fn option_serialize_check() {
        let none: Option<i32> = None;
        let some = Some(42);

        let none_serialized = serde_json::to_string(&none).unwrap();
        let some_serialized = serde_json::to_string(&some).unwrap();

        let none_json: serde_json::Value = serde_json::to_value(&none).unwrap();
        let some_json: serde_json::Value = serde_json::to_value(&some).unwrap();

        println!("print None: {:?}", none);
        println!("serialized: {}", none_serialized);
        println!("json: {}", none_json);

        println!("print Some: {:?}", some);
        println!("serialized: {}", some_serialized);
        println!("json: {}", some_json);
    }
}
