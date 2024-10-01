use chrono::Local;
use message::Message;
use std::{
    fs::File,
    io::{BufWriter, Write},
};

pub trait Recoder {
    fn record_msg(recoder: &mut BufWriter<File>, msg: Message) {
        // 打上时间戳，以 add 的方式写入文件
        writeln!(
            recoder,
            "{} {:?}",
            Local::now().format("%Y-%m-%d %H:%M:%S"),
            msg
        )
        .expect("Failed to write message");
    }
}

#[macro_export]
macro_rules! recode {
    ($recoder:expr, $msg:expr) => {
        writeln!(
            $recoder,
            "{} {:?}",
            Local::now().format("%Y-%m-%d %H:%M:%S"),
            $msg
        )
        .expect("Failed to write message");
    };
}
