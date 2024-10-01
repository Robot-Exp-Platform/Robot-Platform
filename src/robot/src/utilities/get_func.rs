#[macro_export]
macro_rules! get_functions {
    ($(($name:ident, $field:ident)),*) => {
        $(
            fn $name(&self) -> na::DVector<f64> {
                self.$field.clone()
            }
        )*
    };
}
