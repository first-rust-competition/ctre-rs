//! CTRE Phoenix bindings for Rust
//!
//! All the methods here that immediately cause a CAN frame to be sent take a
//! `timeout_ms` parameter. This is a timeout value in milliseconds.
//! If nonzero, function will wait for config success and report an error if it times out.
//! If zero, no blocking or checking is performed.

extern crate ctre_data;
extern crate ctre_sys;
extern crate num_traits;
#[macro_use]
extern crate smart_default;
#[cfg(feature = "usage-reporting")]
extern crate wpilib_sys;

pub use ctre_data::CustomParamConfiguration;
pub use ctre_sys::{ErrorCode, ParamEnum};

/// A specialised `Result` for CTRE methods.
pub type Result<T> = std::result::Result<T, ErrorCode>;

#[macro_use]
mod macros;

pub mod canifier;
pub mod motion;
pub mod motor_control;
pub mod sensors;

pub use self::canifier::CANifier;
pub use self::motor_control as mot;
pub use self::sensors::pigeon::{self, PigeonIMU};

/// Custom parameter index.
/// Used in place of the custom parameter config `paramIndex` argument in the C++/Java API.
#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum CustomParam {
    /// Parameter index 0
    A = 0,
    /// Parameter index 1
    B = 1,
}

pub trait ConfigAll {
    type Configuration;

    /// Configures all persistent settings.
    fn config_all_settings(
        &mut self,
        all_configs: &Self::Configuration,
        timeout_ms: i32,
    ) -> ErrorCode;

    fn get_all_configs(&self, timeout_ms: i32) -> Result<Self::Configuration>;
}
