#[macro_use]
pub mod bindings;

pub mod motor_control;
pub mod motion;

pub use ctre::bindings::{ErrorCode, ParamEnum};
pub type Result<T> = ::std::result::Result<T, ErrorCode>;
