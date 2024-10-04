#[macro_export]
macro_rules! get_fn {
    ($(($name:ident : $T:ty)),*) => {
        $(
            fn $name(&self) -> $T {
                self.$name.clone()
            }
        )*
    };
    ($(($name:ident : $T:ty, $field:ident)),*) => {
        $(
            fn $name(&self) -> $T {
                self.$field.$name.clone()
            }
        )*
    };
}

#[macro_export]
macro_rules! set_fn {
    ($(($fn_name:ident, $name:ident : $T:ty)),*) => {
        $(
            fn $fn_name(&mut self, $name: $T) {
                self.$name = $name;
            }
        )*
    };
    ($(($fn_name:ident, $name:ident : $T:ty, $field:ident)),*) => {
        $(
            fn $fn_name(&mut self, $name: $T) {
                self.$field.$name = $name;
            }
        )*
    };
}
