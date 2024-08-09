pub mod recoder_trait;

pub use chrono::Local;
pub use recoder_trait::Recoder;

use lazy_static::lazy_static;
use std::sync::Mutex;

lazy_static! {
    pub static ref EXP_NAME: String = Local::now().format("%Y-%m-%d|%H:%M").to_string();
    pub static ref TASK_NAME: Mutex<String> = Mutex::new(String::from("task_name"));
}
