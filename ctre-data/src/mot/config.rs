//! Configuration structs for CTRE motor controllers.
use super::super::CustomParamConfiguration;
use super::api::AuxPIDPolarity;
use super::cci::*;

#[derive(Debug, Clone, SmartDefault, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BasePIDSetConfiguration {
    #[default = "1.0"]
    pub selected_feedback_coefficient: f64,
}

#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FilterConfiguration {
    pub remote_sensor_device_id: i32,
    pub remote_sensor_source: RemoteSensorSource,
}

#[derive(Debug, SmartDefault, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SlotConfiguration {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub kf: f64,
    pub integral_zone: i32,
    pub allowable_closed_loop_error: i32,
    pub max_integral_accumulator: f64,
    #[default = "1.0"]
    pub closed_loop_peak_output: f64,
    #[default = "1"]
    pub closed_loop_period: i32,
}

#[derive(Debug, SmartDefault, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct BaseMotorControllerConfiguration {
    pub custom_param: CustomParamConfiguration,
    pub open_loop_ramp: f64,
    pub closed_loop_ramp: f64,
    #[default = "1.0"]
    pub peak_output_forward: f64,
    #[default = "-1.0"]
    pub peak_output_reverse: f64,
    pub nominal_output_forward: f64,
    pub nominal_output_reverse: f64,
    #[default = "0.04"]
    pub neutral_deadband: f64,
    pub voltage_comp_saturation: f64,
    #[default = "32"]
    pub voltage_measurement_filter: i32,
    pub velocity_measurement_period: VelocityMeasPeriod,
    #[default = "64"]
    pub velocity_measurement_window: i32,
    /// Limit Switch device id isn't used unless device is a remote
    pub forward_limit_switch_device_id: i32,
    pub reverse_limit_switch_device_id: i32,
    pub forward_limit_switch_normal: LimitSwitchNormal,
    pub reverse_limit_switch_normal: LimitSwitchNormal,
    pub forward_soft_limit_threshold: i32,
    pub reverse_soft_limit_threshold: i32,
    pub forward_soft_limit_enable: bool,
    pub reverse_soft_limit_enable: bool,
    pub slot_0: SlotConfiguration,
    pub slot_1: SlotConfiguration,
    pub slot_2: SlotConfiguration,
    pub slot_3: SlotConfiguration,
    pub aux_pid_polarity: AuxPIDPolarity,
    pub filter_0: FilterConfiguration,
    pub filter_1: FilterConfiguration,
    pub motion_cruise_velocity: i32,
    pub motion_acceleration: i32,
    pub motion_profile_trajectory_period: i32,
    pub feedback_not_continuous: bool,
    pub remote_sensor_closed_loop_disable_neutral_on_los: bool,
    pub clear_position_on_limit_f: bool,
    pub clear_position_on_limit_r: bool,
    pub clear_position_on_quad_idx: bool,
    pub limit_switch_disable_neutral_on_los: bool,
    pub soft_limit_disable_neutral_on_los: bool,
    #[default = "1"]
    pub pulse_width_period_edges_per_rot: i32,
    #[default = "1"]
    pub pulse_width_period_filter_window_sz: i32,
}

#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TalonSRXPIDSetConfiguration {
    // #[cfg_attr(feature = "serde", serde(flatten))]
    pub _base: BasePIDSetConfiguration,
    pub selected_feedback_sensor: FeedbackDevice,
}
/// CTRE Talon SRX Motor Configuration settings.
#[derive(Debug, SmartDefault, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TalonSRXConfiguration {
    // #[cfg_attr(feature = "serde", serde(flatten))]
    pub _base: BaseMotorControllerConfiguration,
    pub primary_pid: TalonSRXPIDSetConfiguration,
    pub auxiliary_pid: TalonSRXPIDSetConfiguration,
    pub forward_limit_switch_source: LimitSwitchSource,
    pub reverse_limit_switch_source: LimitSwitchSource,
    pub sum_0: FeedbackDevice,
    pub sum_1: FeedbackDevice,
    pub diff_0: FeedbackDevice,
    pub diff_1: FeedbackDevice,
    #[default = "1"]
    pub peak_current_limit: i32,
    #[default = "1"]
    pub peak_current_duration: i32,
    #[default = "1"]
    pub continuous_current_limit: i32,
}

#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VictorSPXPIDSetConfiguration {
    // #[cfg_attr(feature = "serde", serde(flatten))]
    pub _base: BasePIDSetConfiguration,
    pub selected_feedback_sensor: RemoteFeedbackDevice,
}
/// VEX Victor SPX Motor Configuration settings.
#[derive(Debug, Default, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VictorSPXConfiguration {
    // #[cfg_attr(feature = "serde", serde(flatten))]
    pub _base: BaseMotorControllerConfiguration,
    pub primary_pid: VictorSPXPIDSetConfiguration,
    pub auxiliary_pid: VictorSPXPIDSetConfiguration,
    pub forward_limit_switch_source: RemoteLimitSwitchSource,
    pub reverse_limit_switch_source: RemoteLimitSwitchSource,
    pub sum_0: RemoteFeedbackDevice,
    pub sum_1: RemoteFeedbackDevice,
    pub diff_0: RemoteFeedbackDevice,
    pub diff_1: RemoteFeedbackDevice,
}
