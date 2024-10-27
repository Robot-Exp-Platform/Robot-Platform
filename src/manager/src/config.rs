use robot::RobotConfig;
use sensor::SensorConfig;
use serde::Deserialize;

/// 设置文件对应的结构体，主要用于反序列化得到机器人池及传感器池的配置
#[derive(Debug, Deserialize)]
pub struct Config {
    pub robots: Vec<RobotConfig>,
    pub sensors: Vec<SensorConfig>,
}
