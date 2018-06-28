//! Rust bindings for the CTRE Phoenix CCI libraries.

mod enums;
pub use enums::*;

pub mod canifier;
pub mod logger;
pub mod mot;
pub mod pigeon;

impl Into<Result<(), ErrorCode>> for ErrorCode {
    /// Take an ErrorCode and return an `Ok(())` if there is no error, or an `Err` otherwise.
    fn into(self) -> Result<(), ErrorCode> {
        if self == ErrorCode::OK {
            Ok(())
        } else {
            Err(self)
        }
    }
}
