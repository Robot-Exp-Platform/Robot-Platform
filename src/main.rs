use {controller, iterator, planner};
fn get_state() -> String {
    let mut state = String::new();
    state.push_str(&controller::get_contoller_state());
    state.push('\n');
    state.push_str(&planner::get_planner_state());
    state
}
fn main() {
    println!("{}", get_state());
    println!("Hello, world!");
}
