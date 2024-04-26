pub mod bin;
use bin::control_trait::Controller;

fn control(cont: Box<dyn Controller>) {
    cont.get_contoller_state();
}
