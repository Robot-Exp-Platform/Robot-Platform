#[macro_export]
macro_rules! get_fn {
    ($(($name:ident, $field:ident)),*) => {
        $(
            fn $name(&self) -> na::DVector<f64> {
                self.$field.$name.clone()
            }
        )*
    };
}

#[macro_export]
macro_rules! set_fn {
    ($(($fn_name:ident,$name:ident, $field:ident)),*) => {
        $(
            fn $fn_name(&mut self, $name: na::DVector<f64>) {
                self.$field.$name = $name;
            }
        )*
    };
}
