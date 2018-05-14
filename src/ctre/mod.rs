pub mod motor_control;
pub mod motion;

pub use ctre_sys::{ErrorCode, ParamEnum};
pub type Result<T> = ::std::result::Result<T, ErrorCode>;
