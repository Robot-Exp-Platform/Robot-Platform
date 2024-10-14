#![feature(trait_alias)]
#![feature(trait_upcasting)]

pub mod exp;

use exp::Exp;
use manager::Node;
fn main() {
    let mut exp = Exp::default();

    exp.init();

    while exp.is_running() {
        exp.update();
    }
}
