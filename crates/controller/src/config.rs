// use crate::controller_trait::Controller;
// use crate::controllers::pid::{Pid, PidParams};
// use serde::{Deserialize, Serialize};

// #[derive(Serialize, Deserialize)]
// pub struct ControllerConfig {
//     name: String,
//     params: serde_json::Value, // 使用 serde_json::Value 来灵活处理不同的参数
// }

// pub fn init_controller(
//     config: Option<ControllerConfig>,
// ) -> Result<Option<Box<dyn Controller>>, String> {
//     match config {
//         Some(controller_config) => match controller_config.name.as_str() {
//             "pid" => match serde_json::from_value::<PidParams>(controller_config.params) {
//                 Ok(pid_params) => Ok(Some(Box::new(Pid::new(pid_params)))),
//                 Err(_) => Err("Failed to parse PID parameters".to_string()),
//             },
//             _ => Err("unknow controller".to_string()),
//         },
//         None => Ok(None),
//     }
// }
