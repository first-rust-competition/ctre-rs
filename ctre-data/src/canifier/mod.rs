//! CANifier enums and structs.

mod api;
mod cci;
mod config;

pub use self::{api::*, cci::*, config::*};

// pub type ControlFrame = CANifierControlFrame;
// pub type StatusFrame = CANifierStatusFrame;

/// Alias for CANifierVelocityMeasPeriod.
pub type VelocityMeasPeriod = CANifierVelocityMeasPeriod;

/// Alias for CANifierConfiguration.
pub type Configuration = CANifierConfiguration;
