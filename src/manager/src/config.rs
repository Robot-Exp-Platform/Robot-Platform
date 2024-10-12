use serde::Deserialize;
use robot::RobotConfig;

/// 设置文件对应的结构体，主要用于反序列化得到机器人池及传感器池的配置
#[derive(Debug, Deserialize)]
pub struct Config {
    pub robots: Vec<RobotConfig>,
    pub sensors: Vec<SensorConfig>,
}

#[derive(Debug, Deserialize)]
pub struct SensorConfig {
    pub name: String,
    pub sensor_type: String,
}
