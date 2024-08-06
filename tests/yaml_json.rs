use serde_json::Value as JsonValue;
use serde_yaml::Value as YamlValue;
use std::fs::File;

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path; // 在测试模块中也导入 std::path::Path

    #[test]
    fn yaml_to_json() {
        let yaml_file = File::open(Path::new("tests/yaml_file.yaml")).unwrap();
        let json_file = File::create(Path::new("tests/json_file.json")).unwrap();
        let yaml: YamlValue = serde_yaml::from_reader(yaml_file).unwrap();
        let json = serde_json::to_string(&yaml).unwrap();
        serde_json::to_writer(json_file, &json).unwrap();
    }

    #[test]
    fn json_to_yaml() {
        let json_file = File::open(Path::new("tests/task.json")).unwrap();
        let yaml_file = File::create(Path::new("tests/task.yaml")).unwrap();
        let json: JsonValue = serde_json::from_reader(json_file).unwrap();
        let yaml = serde_yaml::to_string(&json).unwrap();
        serde_yaml::to_writer(yaml_file, &yaml).unwrap();
    }
}
