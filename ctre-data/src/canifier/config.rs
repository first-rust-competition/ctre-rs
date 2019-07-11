//! Configuration structs for the CANifier.

use super::super::CustomParamConfiguration;
use super::cci::CANifierVelocityMeasPeriod;

#[derive(Debug, SmartDefault, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct CANifierConfiguration {
    pub custom_param: CustomParamConfiguration,
    pub velocity_measurement_period: CANifierVelocityMeasPeriod,
    #[default = "64"]
    pub velocity_measurement_window: i32,
    /// Whether to clear sensor position on forward limit
    pub clear_position_on_limit_f: bool,
    /// Whether to clear sensor position on reverse limit
    pub clear_position_on_limit_r: bool,
    /// Whether to clear sensor position on index
    pub clear_position_on_quad_idx: bool,
}
