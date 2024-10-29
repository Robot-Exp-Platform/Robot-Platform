#![feature(trait_alias)]
#![feature(trait_upcasting)]
#![feature(more_float_constants)]

pub mod exp;

use exp::Exp;
use node::NodeBehavior;
fn main() {
    let mut exp = Exp::from_json("./config.json", "./task.json");

    exp.init();

    while exp.state() == node::NodeState::Running {
        exp.update();
    }
}
