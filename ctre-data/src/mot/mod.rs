//! CTRE motor controller enums and structs.

pub mod api;
pub mod cci;
pub mod config;

pub use self::{api::*, cci::*, config::*};

use core::mem;

enum_defaults! {
    DemandType::Neutral;
    FollowerType::PercentOutput;

    FeedbackDevice::QuadEncoder;
    RemoteFeedbackDevice::FactoryDefaultOff;

    LimitSwitchSource::FeedbackConnector;
    RemoteLimitSwitchSource::Deactivated;
    LimitSwitchNormal::NormallyOpen;

    RemoteSensorSource::Off;
    /// Returns the default measurement period of 100 ms.
    VelocityMeasPeriod::Period_100Ms;
}

impl From<RemoteFeedbackDevice> for FeedbackDevice {
    #[inline]
    /// Cast a RemoteFeedbackDevice to the corresponding FeedbackDevice.
    fn from(v: RemoteFeedbackDevice) -> Self {
        unsafe { mem::transmute(v) }
    }
}

impl From<RemoteLimitSwitchSource> for LimitSwitchSource {
    #[inline]
    /// Cast a RemoteLimitSwitchSource to the corresponding LimitSwitchSource.
    fn from(v: RemoteLimitSwitchSource) -> Self {
        unsafe { mem::transmute(v) }
    }
}

impl LimitSwitchSource {
    /// Checks if a limit switch is a one of the remote values.
    pub fn is_remote(self) -> bool {
        match self {
            LimitSwitchSource::RemoteTalonSRX | LimitSwitchSource::RemoteCANifier => true,
            _ => false,
        }
    }
}
