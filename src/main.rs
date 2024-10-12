#![feature(trait_alias)]
#![feature(trait_upcasting)]

pub mod exp;

use exp::Exp;
fn main() {
    let mut exp = Exp::default();

    exp.init();
    exp.start();

    while exp.is_running() {
        exp.update();
    }
}
