//! CTRE Phoenix bindings for Rust

extern crate ctre_sys;
pub use ctre_sys::{ErrorCode, ParamEnum};
pub type Result<T> = std::result::Result<T, ErrorCode>;

#[macro_use]
mod macros;

pub mod canifier;
pub mod motion;
pub mod motor_control;
pub mod sensors;

pub use canifier::CANifier;
