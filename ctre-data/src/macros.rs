/// Macro to create simple default implementations.
// TODO: Can we avoid the repetition of the type name for enums?
macro_rules! defaults {
    () => {};
    ($(#[$attr:meta])* $type:ty => $default:expr) => {
        impl Default for $type {
            #[inline]
            $(#[$attr])*
            fn default() -> Self {
                $default
            }
        }
    };
    ($(#[$attr:meta])* $type:ty => $default:expr, $($rest:tt)*) => {
        defaults!($(#[$attr])* $type => $default);
        defaults!($($rest)*);
    };
}

/// Macro to create a public data-less enum that uses its primitive
/// values as its serialisation.  Appropriated from the serde docs.
macro_rules! enum_number {
    ($(#[$attr:meta])* pub enum $name:ident { $($(#[$var_attr:meta])* $variant:ident = $value:expr,)* }) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
        $(#[$attr])*
        pub enum $name {
            $( $(#[$var_attr])* $variant = $value, )*
        }

        #[cfg(feature = "serde")]
        impl ::serde::Serialize for $name {
            fn serialize<S: ::serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                // Serialize the enum as a u64.
                serializer.serialize_u64(*self as u64)
            }
        }

        #[cfg(feature = "serde")]
        impl<'de> ::serde::Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: ::serde::Deserializer<'de>,
            {
                use {core::fmt, serde::de};

                struct Visitor;

                impl<'de> de::Visitor<'de> for Visitor {
                    type Value = $name;

                    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                        formatter.write_str(concat!("an integer from:", $(" ", $value),*))
                    }

                    fn visit_u64<E: de::Error>(self, value: u64) -> Result<$name, E> {
                        // Rust does not come with a simple way of converting a
                        // number to an enum, so use a big `match`.
                        match value {
                            $( $value => Ok($name::$variant), )*
                            _ => Err(E::invalid_value(de::Unexpected::Unsigned(value), &self)),
                        }
                    }
                }

                // Deserialize the enum from a u64.
                deserializer.deserialize_u64(Visitor)
            }
        }
    }
}
