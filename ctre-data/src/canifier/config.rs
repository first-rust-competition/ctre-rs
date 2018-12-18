//! Configuration structs for the CANifier.

use super::super::CustomParamConfiguration;
use super::cci::CANifierVelocityMeasPeriod;

#[derive(Debug, Default, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CANifierConfiguration {
    pub custom_param: CustomParamConfiguration,
    pub velocity_measurement_period: CANifierVelocityMeasPeriod,
    pub velocity_measurement_window: i32,
    pub clear_position_on_limit_f: bool,
    pub clear_position_on_limit_r: bool,
    pub clear_position_on_quad_idx: bool,
}
