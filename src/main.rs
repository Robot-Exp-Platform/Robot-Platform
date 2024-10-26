#![feature(trait_alias)]
#![feature(trait_upcasting)]
#![feature(more_float_constants)]

pub mod exp;

use std::{thread, time::Duration};

use exp::Exp;
use node::NodeBehavior;
fn main() {
    let mut exp = Exp::from_json("./config.json", "./task.json");

    exp.init();

    while exp.is_running() {
        exp.update();
        thread::sleep(Duration::from_secs(10));
    }
}
