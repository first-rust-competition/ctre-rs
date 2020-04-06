/*
 * This file is licensed under MIT.
 */

/// Macro to create simple Default implementations for data-less enums.
macro_rules! enum_defaults {
    () => {};
    ($(#[$attr:meta])* $type:ident :: $default:ident) => {
        impl Default for $type {
            #[inline]
            $(#[$attr])*
            fn default() -> Self {
                $type::$default
            }
        }
    };
    ($(#[$attr:meta])* $type:ident :: $default:ident; $($rest:tt)*) => {
        enum_defaults!($(#[$attr])* $type::$default);
        enum_defaults!($($rest)*);
    };
}

/// Macro to create a data-less enum that uses its primitive
/// values as its serialisation.  Appropriated from the serde docs.
///
/// This assumes that all values are non-negative.
macro_rules! enum_number {
    (
        $(#[$attr:meta])*
        $vis:vis enum $name:ident {
            $($(#[$var_attr:meta])* $variant:ident = $value:expr,)*
        }
    ) => {
        #[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
        $(#[$attr])*
        $vis enum $name {
            $( $(#[$var_attr])* $variant = $value, )*
        }

        #[cfg(feature = "serde")]
        impl serde::Serialize for $name {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                // Serialize the enum as a u64.
                serializer.serialize_u64(*self as u64)
            }
        }

        #[cfg(feature = "serde")]
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: serde::Deserializer<'de>,
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
