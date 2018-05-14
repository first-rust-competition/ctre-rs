pub mod canifier;
pub mod motor_control;
pub mod motion;
pub mod sensors;

pub use ctre_sys::{ErrorCode, ParamEnum};
pub type Result<T> = ::std::result::Result<T, ErrorCode>;

pub use self::canifier::CANifier;
