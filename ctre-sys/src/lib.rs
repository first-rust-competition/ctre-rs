//! Rust bindings for the CTRE Phoenix CCI libraries.

mod enums;
pub use self::enums::*;

#[macro_use]
mod macros;

pub mod canifier;
pub mod logger;
pub mod mot;
pub mod pigeon;

use std::fmt;

impl ErrorCode {
    /// Returns `true` if the error code is `OK`.
    #[inline]
    pub fn is_ok(self) -> bool {
        self == ErrorCode::OK
    }

    /// Returns `true` if the error code is not `OK`.
    #[inline]
    pub fn is_err(self) -> bool {
        self != ErrorCode::OK
    }

    /// Returns the first error code which is not `OK`, or `OK` otherwise.
    /// Intended for use by the `ctre` crate only.
    pub fn or(self, err: Self) -> Self {
        match self {
            ErrorCode::OK => err,
            _ => self,
        }
    }

    /// Returns an `Ok` if the error code is `OK`, or an `Err` otherwise.
    pub fn into_res(self) -> Result<(), Self> {
        match self {
            ErrorCode::OK => Ok(()),
            _ => Err(self),
        }
    }
}

impl std::error::Error for ErrorCode {
    fn description(&self) -> &str {
        "Error in CTRE Phoenix"
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // yeah uhm CTRE pls fix your Logger CCI
        write!(f, "{:?}", self)
    }
}

#[cfg(feature = "try_trait")]
impl std::ops::Try for ErrorCode {
    type Ok = ();
    type Error = Self;

    fn into_result(self) -> Result<(), Self> {
        self.into_res()
    }

    fn from_error(v: Self) -> Self {
        v
    }
    fn from_ok(v: ()) -> Self {
        ErrorCode::OK
    }
}
