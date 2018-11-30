/// Convenience wrapper for making simple get calls.
macro_rules! cci_get_call {
    ($function:ident($($arg0:expr,)+ _: $type:ty $(, $arg1:expr)*$(,)*)) => ({
        let mut value: $type = Default::default();
        let error = unsafe { $function($($arg0,)* &mut value, $($arg1,)*) };
        if error == ErrorCode::OK { Ok(value) } else { Err(error) }
    })
}

/// Convenience wrapper for making simple get calls, ignoring the ErrorCode.
macro_rules! cci_get_only {
    ($function:ident($($arg0:expr,)+ _: $type:ty $(, $arg1:expr)*$(,)*)) => ({
        let mut value: $type = Default::default();
        unsafe { $function($($arg0,)* &mut value, $($arg1,)*) };
        value
    })
}

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
