/// Macro to create From implementations by transmuting.
/// This should only ever be used if the from type is a subset of the to type.
macro_rules! transmute_from {
    () => {};
    ($(#[$attr:meta])* $from:ty => $to:ty) => {
        impl From<$from> for $to {
            #[inline]
            $(#[$attr])*
            fn from(v: $from) -> Self {
                unsafe { ::std::mem::transmute(v) }
            }
        }
    };
    ($(#[$attr:meta])* $from:ty => $to:ty, $($rest:tt)*) => {
        transmute_from!($(#[$attr])* $from => $to);
        transmute_from!($($rest)*);
    }
}
