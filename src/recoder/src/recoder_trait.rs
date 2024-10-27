pub trait Recoder {}

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
