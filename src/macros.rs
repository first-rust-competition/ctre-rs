/// Convenience wrapper for making simple get calls.
macro_rules! cci_get_call {
    ($function:ident($($arg0:expr,)+ _: $type:ty $(, $arg1:expr)*$(,)*)) => ({
        let mut value: $type = unsafe { ::std::mem::uninitialized() };
        let error = unsafe { $function($($arg0,)* &mut value, $($arg1),* ) };
        if error == ErrorCode::OK { Ok(value) } else { Err(error) }
    });
    /*
    ($function:ident($($arg0:expr,)+ _: $type:ty, $($arg1:expr,)*)) => (
        cci_get_call!($function($($arg0,)* _: $type, $($arg1),*))
    );
    ($function:ident($($arg0:expr,)+ _: $type:ty)) => (
        cci_get_call!($function($($arg0,)* _: $type,))
    );
    */
}

/// Convenience wrapper for making simple get calls, ignoring the ErrorCode.
macro_rules! cci_get_only {
    ($function:ident($($arg0:expr,)+ _: $type:ty $(, $arg1:expr)*$(,)*)) => ({
        let mut value: $type = unsafe { ::std::mem::uninitialized() };
        unsafe { $function($($arg0,)* &mut value, $($arg1,)*) };
        value
    });
    /*
    ($function:ident($($arg0:expr,)+ _: $type:ty, $($arg1:expr,)*)) => (
        cci_get_only!($function($($arg0,)* _: $type, $($arg1),*))
    );
    ($function:ident($($arg0:expr,)+ _: $type:ty)) => (
        cci_get_only!($function($($arg0,)* _: $type,))
    );
    */
}

/*
/// Create CCI getter wrappers, because metaprogramming.
macro_rules! make_cci_getter {
    () => {};
    ($(#[$attr:meta])* fn $rust_fn:ident = $cci_fn:ident -> $type:ty) => {
        $(#[$attr])*
        fn $rust_fn(&self) -> Result<$type> {
            cci_get_call!($cci_fn(self.handle(), _: $type))
        }
    };
    ($(#[$attr:meta])* fn $rust_fn:ident = $cci_fn:ident -> $type:ty; $($rest:tt)*) => {
        make_cci_getter!($(#[$attr])* fn $rust_fn = $cci_fn -> $type);
        make_cci_getter!($($rest)*);
    };
}
*/

macro_rules! impl_binary_fmt {
    ($type:ty) => {
        impl ::std::fmt::Binary for $type {
            fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
                self.0.fmt(f)
            }
        }
        impl ::std::fmt::Octal for $type {
            fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
                self.0.fmt(f)
            }
        }
        impl ::std::fmt::LowerHex for $type {
            fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
                self.0.fmt(f)
            }
        }
        impl ::std::fmt::UpperHex for $type {
            fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
                self.0.fmt(f)
            }
        }
    };
}

macro_rules! f64_to_enum {
    ($expr:expr => $enum:ty) => {
        <$enum as ::num_traits::cast::FromPrimitive>::from_f64($expr).unwrap_or(Default::default())
    };
}
