use serde::de;
use serde::{Deserialize, Deserializer};

use robot::ros_thread::ROSThread;

#[derive(Debug, Deserialize)]
pub enum TaskType {
    #[serde(rename = "targets")]
    Targets(Vec<f64>),
}

#[derive(Debug, Deserialize)]
pub struct TaskParam {
    pub node_type: String,
    pub path: String,
    pub param: String,
}

#[derive(Debug)]
pub struct Task {
    pub task_type: TaskType,
    pub params: Vec<TaskParam>,
}

// 实现 Task 的自定义反序列化
impl<'de> Deserialize<'de> for Task {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct TaskHelper {
            task_type: String,
            targets_list: Option<Vec<f64>>,
            params: Vec<TaskParam>,
        }

        let helper = TaskHelper::deserialize(deserializer)?;
        let task_type = match helper.task_type.as_str() {
            "targets" => TaskType::Targets(
                helper
                    .targets_list
                    .ok_or_else(|| de::Error::missing_field("targets_list"))?,
            ),
            _ => return Err(de::Error::unknown_variant(&helper.task_type, &["targets"])),
        };

        Ok(Task {
            task_type,
            params: helper.params,
        })
    }
}

impl ROSThread for Task {
    fn init(&self) {
        unimplemented!()
    }
    fn start(&self) {
        unimplemented!()
    }
    fn update(&self) {
        unimplemented!()
    }
}
