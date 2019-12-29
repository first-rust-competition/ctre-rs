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

/// Creates enum-like constants for a tuple newtype, and implements Debug.
macro_rules! newtype_consts {
    ($type:ty { $($(#[$attr:meta])* $name:ident = $val:expr,)* }) => {
        impl $type {
            $(
                $(#[$attr])*
                pub const $name: Self = Self($val);
            )*
        }

        impl std::fmt::Debug for $type {
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                match self {
                    $(
                        &Self::$name => f.write_str(stringify!($name)),
                    )*
                    _ => {
                        f.debug_tuple(stringify!($type))
                            .field(&self.0)
                            .finish()
                    }
                }
            }
        }
    }
}
