extern crate ctre_sys;

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

/// Convenience wrapper for making simple get calls which expect fixed size arrays.
macro_rules! cci_get_call_array {
    ($function:ident($($arg0:expr,)+ _: $type:ty $(, $arg1:expr)*$(,)*)) => ({
        let mut value: $type = Default::default();
        let error = unsafe { $function($($arg0,)* value.as_mut_ptr(), $($arg1,)*) };
        if error == ErrorCode::OK { Ok(value) } else { Err(error) }
    })
}

mod ctre;
pub use ctre::*;
