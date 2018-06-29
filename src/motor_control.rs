use ctre_sys::mot::*;
pub use ctre_sys::mot::{ControlFrame, ControlFrameEnhanced, ControlMode, DemandType,
                        FeedbackDevice, FollowerType, LimitSwitchNormal, LimitSwitchSource,
                        RemoteFeedbackDevice, RemoteLimitSwitchSource, StatusFrame,
                        StatusFrameEnhanced, VelocityMeasPeriod};
use motion::{MotionProfileStatus, TrajectoryPoint};
use {ErrorCode, ParamEnum, Result};

#[derive(Default, Debug, Copy, Clone)]
pub struct Faults(i32);
impl Faults {
    pub fn under_voltage(&self) -> bool {
        self.0 & (1 << 0) != 0
    }
    pub fn forward_limit_switch(&self) -> bool {
        self.0 & (1 << 1) != 0
    }
    pub fn reverse_limit_switch(&self) -> bool {
        self.0 & (1 << 2) != 0
    }
    pub fn forward_soft_limit(&self) -> bool {
        self.0 & (1 << 3) != 0
    }
    pub fn reverse_soft_limit(&self) -> bool {
        self.0 & (1 << 4) != 0
    }
    pub fn hardware_failure(&self) -> bool {
        self.0 & (1 << 5) != 0
    }
    pub fn reset_during_en(&self) -> bool {
        self.0 & (1 << 6) != 0
    }
    pub fn sensor_overflow(&self) -> bool {
        self.0 & (1 << 7) != 0
    }
    pub fn sensor_out_of_phase(&self) -> bool {
        self.0 & (1 << 8) != 0
    }
    pub fn hardware_esd_reset(&self) -> bool {
        self.0 & (1 << 9) != 0
    }
    pub fn remote_loss_of_signal(&self) -> bool {
        self.0 & (1 << 10) != 0
    }
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct StickyFaults(i32);
impl StickyFaults {
    pub fn under_voltage(&self) -> bool {
        self.0 & (1 << 0) != 0
    }
    pub fn forward_limit_switch(&self) -> bool {
        self.0 & (1 << 1) != 0
    }
    pub fn reverse_limit_switch(&self) -> bool {
        self.0 & (1 << 2) != 0
    }
    pub fn forward_soft_limit(&self) -> bool {
        self.0 & (1 << 3) != 0
    }
    pub fn reverse_soft_limit(&self) -> bool {
        self.0 & (1 << 4) != 0
    }
    pub fn reset_during_en(&self) -> bool {
        self.0 & (1 << 5) != 0
    }
    pub fn sensor_overflow(&self) -> bool {
        self.0 & (1 << 6) != 0
    }
    pub fn sensor_out_of_phase(&self) -> bool {
        self.0 & (1 << 7) != 0
    }
    pub fn hardware_esd_reset(&self) -> bool {
        self.0 & (1 << 8) != 0
    }
    pub fn remote_loss_of_signal(&self) -> bool {
        self.0 & (1 << 9) != 0
    }
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}

pub trait BaseMotorController {
    /// Constructor for motor controllers.
    fn new(device_number: i32) -> Self;

    fn get_handle(&self) -> Handle;
    fn get_base_id(&self) -> i32;
    fn get_device_id(&self) -> i32 {
        cci_get_only!(c_MotController_GetDeviceNumber(self.get_handle(), _: i32))
    }

    /**
     * * `mode` - Sets the appropriate output on the talon, depending on the mode.
     * * `demand0` - The output value to apply.
     *   such as advanced feed forward and/or auxiliary close-looping in firmware.
     *   * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
     *   * In Current mode, output value is in amperes.
     *   * In Velocity mode, output value is in position change / 100ms.
     *   * In Position mode, output value is in encoder ticks or an analog value,
     *     depending on the sensor. See
     *   * In Follower mode, the output value is the integer device ID of the talon to duplicate.
     * * `demand1Type` - The demand type for demand1.
     * * `demand1` - Supplmental output value.  Units match the set mode.
     *
     * Arcade Drive Example:
     * ```
     * talonLeft.set(ControlMode::PercentOutput, joyForward, DemandType::ArbitraryFeedForward, +joyTurn);
     * talonRght.set(ControlMode::PercentOutput, joyForward, DemandType::ArbitraryFeedForward, -joyTurn);
     * ```
     *
     * Drive Straight Example:
     * Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
     * ```
     * talonLeft.follow(talonRght, FollowerType::AuxOutput1);
     * talonRght.set(ControlMode::PercentOutput, joyForward, DemandType::AuxPID, desiredRobotHeading);
     * ```
     *
     * Drive Straight to a Distance Example:
     * Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
     * ```
     * talonLeft.follow(talonRght, FollowerType::AuxOutput1);
     * talonRght.set(ControlMode::MotionMagic, targetDistance, DemandType::AuxPID, desiredRobotHeading);
     * ```
     */
    fn set(&self, mode: ControlMode, demand0: f64, demand1_type: DemandType, demand1: f64) {
        match mode {
            ControlMode::Follower => {
                // did caller specify device ID
                let work = if 0.0 <= demand0 && demand0 <= 62.0 {
                    ((self.get_base_id() as u32 >> 16) << 8) | (demand0 as u32)
                } else {
                    demand0 as u32
                };
                unsafe {
                    /* single precision guarantees 16bits of integral precision,
                     *  so float/double cast on work is safe */
                    c_MotController_Set_4(
                        self.get_handle(),
                        mode as _,
                        work as f64,
                        demand1,
                        demand1_type as _,
                    )
                }
            }
            ControlMode::Current => unsafe {
                // milliamps
                c_MotController_SetDemand(self.get_handle(), mode as _, (1000.0 * demand0) as _, 0)
            },
            ControlMode::PercentOutput
            //| ControlMode::TimedPercentOutput
            | ControlMode::Velocity
            | ControlMode::Position
            | ControlMode::MotionMagic
            //| ControlMode::MotionMagicArc
            | ControlMode::MotionProfile
            | ControlMode::MotionProfileArc => unsafe {
                c_MotController_Set_4(
                    self.get_handle(),
                    mode as _,
                    demand0,
                    demand1,
                    demand1_type as _,
                )
            },
            ControlMode::Disabled => unsafe {
                c_MotController_SetDemand(self.get_handle(), mode as _, 0, 0)
            },
        };
    }

    /// Neutral the motor output by setting control mode to disabled.
    fn neutral_output(&self) {
        self.set(ControlMode::Disabled, 0.0, DemandType::Neutral, 0.0)
    }
    /// Sets the mode of operation during neutral throttle output.
    fn set_neutral_mode(&self, neutral_mode: NeutralMode) {
        unsafe { c_MotController_SetNeutralMode(self.get_handle(), neutral_mode as _) }
    }

    /**
     * Sets the phase of the sensor. Use when controller forward/reverse output
     * doesn't correlate to appropriate forward/reverse reading of sensor.
     * Pick a value so that positive PercentOutput yields a positive change in sensor.
     * After setting this, user can freely call [`set_inverted`] with any value.
     *
     * * `phase_sensor` - Indicates whether to invert the phase of the sensor.
     */
    fn set_sensor_phase(&self, phase_sensor: bool) {
        unsafe { c_MotController_SetSensorPhase(self.get_handle(), phase_sensor) }
    }
    /**
     * Inverts the hbridge output of the motor controller.
     *
     * This does not impact sensor phase and should not be used to correct sensor polarity.
     *
     * This will invert the hbridge output but NOT the LEDs.
     * This ensures....
     *  - Green LEDs always represents positive request from robot-controller/closed-looping mode.
     *  - Green LEDs correlates to forward limit switch.
     *  - Green LEDs correlates to forward soft limit.
     */
    fn set_inverted(&self, invert: bool) {
        unsafe { c_MotController_SetInverted(self.get_handle(), invert) }
    }

    fn config_openloop_ramp(
        &self,
        seconds_from_neutral_to_full: f64,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigOpenLoopRamp(
                self.get_handle(),
                seconds_from_neutral_to_full,
                timeout_ms,
            )
        }
    }
    fn config_closedloop_ramp(
        &self,
        seconds_from_neutral_to_full: f64,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigClosedLoopRamp(
                self.get_handle(),
                seconds_from_neutral_to_full,
                timeout_ms,
            )
        }
    }

    fn config_peak_output_forward(&self, percent_out: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigPeakOutputForward(self.get_handle(), percent_out, timeout_ms)
        }
    }
    fn config_peak_output_reverse(&self, percent_out: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigPeakOutputReverse(self.get_handle(), percent_out, timeout_ms)
        }
    }

    fn config_nominal_output_forward(&self, percent_out: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigNominalOutputForward(self.get_handle(), percent_out, timeout_ms)
        }
    }
    fn config_nominal_output_reverse(&self, percent_out: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigNominalOutputReverse(self.get_handle(), percent_out, timeout_ms)
        }
    }

    fn config_neutral_deadband(&self, percent_deadband: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigNeutralDeadband(self.get_handle(), percent_deadband, timeout_ms)
        }
    }

    fn config_voltage_comp_saturation(&self, voltage: f64, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigVoltageCompSaturation(self.get_handle(), voltage, timeout_ms)
        }
    }
    fn config_voltage_measurement_filter(
        &self,
        filter_window_samples: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigVoltageMeasurementFilter(
                self.get_handle(),
                filter_window_samples,
                timeout_ms,
            )
        }
    }
    fn enable_voltage_compensation(&self, enable: bool) {
        unsafe { c_MotController_EnableVoltageCompensation(self.get_handle(), enable) }
    }

    fn get_bus_voltage(&self) -> Result<f64> {
        cci_get_call!(c_MotController_GetBusVoltage(self.get_handle(), _: f64))
    }
    /// Gets the output percentage of the motor controller, in the interval \[-1, +1].
    fn get_motor_output_percent(&self) -> Result<f64> {
        cci_get_call!(c_MotController_GetMotorOutputPercent(self.get_handle(), _: f64))
    }
    fn get_motor_output_voltage(&self) -> Result<f64> {
        Ok(self.get_bus_voltage()? * self.get_motor_output_percent()?)
    }
    fn get_output_current(&self) -> Result<f64> {
        cci_get_call!(c_MotController_GetOutputCurrent(self.get_handle(), _: f64))
    }
    /// Gets the temperature of the motor controller in degrees Celsius.
    fn get_temperature(&self) -> Result<f64> {
        cci_get_call!(c_MotController_GetTemperature(self.get_handle(), _: f64))
    }

    fn config_selected_feedback_sensor(
        &self,
        feedback_device: RemoteFeedbackDevice,
        pid_idx: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSelectedFeedbackSensor(
                self.get_handle(),
                feedback_device as _,
                pid_idx,
                timeout_ms,
            )
        }
    }
    /**
     * The Feedback Coefficient is a scalar applied to the value of the
     * feedback sensor.  Useful when you need to scale your sensor values
     * within the closed-loop calculations.  Default value is 1.
     *
     * Selected Feedback Sensor register in firmware is the decoded sensor value
     * multiplied by the Feedback Coefficient.
     *
     * * `coefficient` - Feedback Coefficient value.  Maximum value of 1.
     *   Resolution is 1/(2^16).  Cannot be 0.
     */
    fn config_selected_feedback_coefficient(
        &self,
        coefficient: f64,
        pid_idx: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSelectedFeedbackCoefficient(
                self.get_handle(),
                coefficient,
                pid_idx,
                timeout_ms,
            )
        }
    }

    /**
     * Select what remote device and signal to assign to Remote Sensor 0 or Remote Sensor 1.
     * After binding a remote device and signal to Remote Sensor X, you may select Remote Sensor X
     * as a PID source for closed-loop features.
     */
    fn config_remote_feedback_filter(
        &self,
        device_id: i32,
        remote_sensor_source: RemoteSensorSource,
        remote_ordinal: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigRemoteFeedbackFilter(
                self.get_handle(),
                device_id,
                remote_sensor_source as _,
                remote_ordinal,
                timeout_ms,
            )
        }
    }

    /**
     * Select what sensor term should be bound to switch feedback device.
     * Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1
     * Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1
     * The four terms are specified with this routine.  Then Sensor Sum/Difference
     * can be selected for closed-looping.
     */
    fn config_sensor_term(
        &self,
        sensor_term: SensorTerm,
        feedback_device: FeedbackDevice,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSensorTerm(
                self.get_handle(),
                sensor_term as _,
                feedback_device as _,
                timeout_ms,
            )
        }
    }

    /// Get the selected sensor position (in raw sensor units).
    fn get_selected_sensor_position(&self, pid_idx: i32) -> Result<i32> {
        cci_get_call!(c_MotController_GetSelectedSensorPosition(
            self.get_handle(),
            _: i32,
            pid_idx,
        ))
    }
    fn get_selected_sensor_velocity(&self, pid_idx: i32) -> Result<i32> {
        cci_get_call!(c_MotController_GetSelectedSensorVelocity(
            self.get_handle(),
            _: i32,
            pid_idx,
        ))
    }
    fn set_selected_sensor_position(
        &self,
        sensor_pos: i32,
        pid_idx: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_SetSelectedSensorPosition(
                self.get_handle(),
                sensor_pos,
                pid_idx,
                timeout_ms,
            )
        }
    }

    fn set_control_frame_period(&self, frame: ControlFrame, period_ms: i32) -> ErrorCode {
        unsafe { c_MotController_SetControlFramePeriod(self.get_handle(), frame as _, period_ms) }
    }
    fn set_status_frame_period(
        &self,
        frame: StatusFrame,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_SetStatusFramePeriod(
                self.get_handle(),
                frame as _,
                period_ms,
                timeout_ms,
            )
        }
    }
    fn get_status_frame_period(&self, frame: StatusFrame, timeout_ms: i32) -> Result<i32> {
        cci_get_call!(c_MotController_GetStatusFramePeriod(
            self.get_handle(),
            frame as _,
            _: i32,
            timeout_ms,
        ))
    }

    fn config_forward_limit_switch_source(
        &self,
        type_: RemoteLimitSwitchSource,
        normal_open_or_close: LimitSwitchNormal,
        device_id: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigForwardLimitSwitchSource(
                self.get_handle(),
                type_ as _,
                normal_open_or_close as _,
                device_id,
                timeout_ms,
            )
        }
    }
    fn config_reverse_limit_switch_source(
        &self,
        type_: RemoteLimitSwitchSource,
        normal_open_or_close: LimitSwitchNormal,
        device_id: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigReverseLimitSwitchSource(
                self.get_handle(),
                type_ as _,
                normal_open_or_close as _,
                device_id,
                timeout_ms,
            )
        }
    }
    fn override_limit_switches_enable(&self, enable: bool) {
        unsafe { c_MotController_OverrideLimitSwitchesEnable(self.get_handle(), enable) }
    }

    fn config_forward_soft_limit_threshold(
        &self,
        forward_sensor_limit: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigForwardSoftLimitThreshold(
                self.get_handle(),
                forward_sensor_limit,
                timeout_ms,
            )
        }
    }
    fn config_reverse_soft_limit_threshold(
        &self,
        reverse_sensor_limit: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigReverseSoftLimitThreshold(
                self.get_handle(),
                reverse_sensor_limit,
                timeout_ms,
            )
        }
    }
    fn config_forward_soft_limit_enable(&self, enable: bool, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigForwardSoftLimitEnable(self.get_handle(), enable, timeout_ms)
        }
    }
    fn config_reverse_soft_limit_enable(&self, enable: bool, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_ConfigReverseSoftLimitEnable(self.get_handle(), enable, timeout_ms)
        }
    }
    fn override_soft_limits_enable(&self, enable: bool) {
        unsafe { c_MotController_OverrideSoftLimitsEnable(self.get_handle(), enable) }
    }

    // current limiting is Talon-specific

    fn config_kp(&self, slot_idx: i32, value: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_Config_kP(self.get_handle(), slot_idx, value, timeout_ms) }
    }
    fn config_ki(&self, slot_idx: i32, value: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_Config_kI(self.get_handle(), slot_idx, value, timeout_ms) }
    }
    fn config_kd(&self, slot_idx: i32, value: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_Config_kD(self.get_handle(), slot_idx, value, timeout_ms) }
    }
    fn config_kf(&self, slot_idx: i32, value: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_Config_kF(self.get_handle(), slot_idx, value, timeout_ms) }
    }
    fn config_integral_zone(&self, slot_idx: i32, izone: i32, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_Config_IntegralZone(
                self.get_handle(),
                slot_idx,
                izone as f64, // idek both C++ and Java do this too
                timeout_ms,
            )
        }
    }
    fn config_allowable_closedloop_error(
        &self,
        slot_idx: i32,
        allowable_closed_loop_error: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigAllowableClosedloopError(
                self.get_handle(),
                slot_idx,
                allowable_closed_loop_error,
                timeout_ms,
            )
        }
    }
    fn config_max_integral_accumulator(
        &self,
        slot_idx: i32,
        iaccum: f64,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigMaxIntegralAccumulator(
                self.get_handle(),
                slot_idx,
                iaccum,
                timeout_ms,
            )
        }
    }
    fn config_closed_loop_peak_output(
        &self,
        slot_idx: i32,
        percent_out: f64,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigClosedLoopPeakOutput(
                self.get_handle(),
                slot_idx,
                percent_out,
                timeout_ms,
            )
        }
    }
    fn config_closed_loop_period(
        &self,
        slot_idx: i32,
        loop_time_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigClosedLoopPeriod(
                self.get_handle(),
                slot_idx,
                loop_time_ms,
                timeout_ms,
            )
        }
    }
    fn config_aux_pid_polarity(&self, invert: bool, timeout_ms: i32) -> ErrorCode {
        self.config_set_parameter(
            ParamEnum::PIDLoopPolarity,
            invert as i8 as f64,
            0,
            1,
            timeout_ms,
        )
    }
    fn set_integral_accumulator(&self, iaccum: f64, pid_idx: i32, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_SetIntegralAccumulator(self.get_handle(), iaccum, pid_idx, timeout_ms)
        }
    }
    fn get_closed_loop_error(&self, pid_idx: i32) -> Result<i32> {
        cci_get_call!(c_MotController_GetClosedLoopError(self.get_handle(), _: i32, pid_idx))
    }
    fn get_integral_accumulator(&self, pid_idx: i32) -> Result<f64> {
        cci_get_call!(c_MotController_GetIntegralAccumulator(self.get_handle(), _: f64, pid_idx))
    }
    /// Gets the derivative of the closed-loop error.
    fn get_error_derivative(&self, pid_idx: i32) -> Result<f64> {
        cci_get_call!(c_MotController_GetErrorDerivative(self.get_handle(), _: f64, pid_idx))
    }
    /// Selects which profile slot to use for closed-loop control.
    fn select_profile_slot(&self, slot_idx: i32, pid_idx: i32) -> ErrorCode {
        unsafe { c_MotController_SelectProfileSlot(self.get_handle(), slot_idx, pid_idx) }
    }
    fn get_closed_loop_target(&self, pid_idx: i32) -> Result<i32> {
        cci_get_call!(c_MotController_GetClosedLoopTarget(self.get_handle(), _: i32, pid_idx))
    }

    /// Gets the active trajectory target position using MotionMagic/MotionProfile control modes.
    fn get_active_trajectory_position(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetActiveTrajectoryPosition(self.get_handle(), _: i32))
    }
    /// Gets the active trajectory target velocity using MotionMagic/MotionProfile control modes.
    fn get_active_trajectory_velocity(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetActiveTrajectoryVelocity(self.get_handle(), _: i32))
    }
    /// Gets the active trajectory target heading using MotionMagic/MotionProfile control modes.
    fn get_active_trajectory_heading(&self) -> Result<f64> {
        cci_get_call!(c_MotController_GetActiveTrajectoryHeading(self.get_handle(), _: f64))
    }

    /// Sets the Motion Magic Cruise Velocity.
    /// This is the peak target velocity that the motion magic curve generator can use.
    fn config_motion_cruise_velocity(
        &self,
        sensor_units_per_100ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigMotionCruiseVelocity(
                self.get_handle(),
                sensor_units_per_100ms,
                timeout_ms,
            )
        }
    }
    /// Sets the Motion Magic Acceleration.
    /// This is the target acceleration that the motion magic curve generator can use.
    fn config_motion_acceleration(
        &self,
        sensor_units_per_100ms_per_sec: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigMotionAcceleration(
                self.get_handle(),
                sensor_units_per_100ms_per_sec,
                timeout_ms,
            )
        }
    }

    /// Clear the buffered motion profile in both motor controller's RAM (bottom),
    /// and in the API (top).
    fn clear_motion_profile_trajectories(&self) -> ErrorCode {
        unsafe { c_MotController_ClearMotionProfileTrajectories(self.get_handle()) }
    }
    /**
     * Retrieve just the buffer count for the api-level (top) buffer.
     * This routine performs no CAN or data structure lookups, so its fast and ideal
     * if caller needs to quickly poll the progress of trajectory points being
     * emptied into motor controller's RAM. Otherwise just use [`get_motion_profile_status`].
     */
    fn get_motion_profile_top_level_buffer_count(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetMotionProfileTopLevelBufferCount(
            self.get_handle(),
            _: i32,
        ))
    }
    /// Push another trajectory point into the top level buffer (which is emptied
    /// into the motor controller's bottom buffer as room allows).
    fn push_motion_profile_trajectory(&self, traj_pt: &TrajectoryPoint) -> ErrorCode {
        unsafe {
            c_MotController_PushMotionProfileTrajectory_2(
                self.get_handle(),
                traj_pt.position,
                traj_pt.velocity,
                traj_pt.auxiliary_pos,
                traj_pt.profile_slot_select_0 as _, // wtf CTRE???
                traj_pt.profile_slot_select_1 as _,
                traj_pt.is_last_point,
                traj_pt.zero_pos,
                traj_pt.time_dur as _,
            )
        }
    }
    /**
     * Retrieve just the buffer full for the api-level (top) buffer.
     * This routine performs no CAN or data structure lookups, so its fast and ideal
     * if caller needs to quickly poll. Otherwise just use [`get_motion_profile_status`].
     */
    fn is_motion_profile_top_level_buffer_full(&self) -> Result<bool> {
        cci_get_call!(c_MotController_IsMotionProfileTopLevelBufferFull(
            self.get_handle(),
            _: bool,
        ))
    }
    /**
     * This must be called periodically to funnel the trajectory points from the
     * API's top level buffer to the controller's bottom level buffer.  Recommendation
     * is to call this twice as fast as the execution rate of the motion profile.
     * So if MP is running with 20ms trajectory points, try calling this routine
     * every 10ms.  All motion profile functions are thread-safe through the use of
     * a mutex, so there is no harm in having the caller utilize threading.
     */
    fn process_motion_profile_buffer(&self) {
        unsafe { c_MotController_ProcessMotionProfileBuffer(self.get_handle()) };
    }
    /**
     * Retrieve all status information.
     * For best performance, Caller can snapshot all status information regarding the
     * motion profile executer.
     */
    fn get_motion_profile_status(&self, status_to_fill: &mut MotionProfileStatus) -> ErrorCode {
        // let mut status_to_fill: MotionProfileStatus = Default::default();
        let retval = unsafe {
            c_MotController_GetMotionProfileStatus_2(
                self.get_handle(),
                &mut status_to_fill.top_buffer_rem,
                &mut status_to_fill.top_buffer_cnt,
                &mut status_to_fill.btm_buffer_cnt,
                &mut status_to_fill.has_underrun,
                &mut status_to_fill.is_underrun,
                &mut status_to_fill.active_point_valid,
                &mut status_to_fill.is_last,
                &mut status_to_fill.profile_slot_select_0,
                &mut status_to_fill.output_enable,
                &mut status_to_fill.time_dur_ms,
                &mut status_to_fill.profile_slot_select_1,
            )
        };
        // if retval == ErrorCode::OK { Ok(status_to_fill) } else { Err(retval) }
        retval
    }
    /// Clear the "Has Underrun" flag.
    /// Typically this is called after application has confirmed an underrun had occured.
    fn clear_motion_profile_has_underrun(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ClearMotionProfileHasUnderrun(self.get_handle(), timeout_ms) }
    }
    /**
     * Calling application can opt to speed up the handshaking between the robot API
     * and the controller to increase the download rate of the controller's Motion Profile.
     * Ideally the period should be no more than half the period of a trajectory
     * point.
     */
    fn change_motion_control_frame_period(&self, period_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ChangeMotionControlFramePeriod(self.get_handle(), period_ms) }
    }
    /**
     * When trajectory points are processed in the motion profile executer, the MPE determines
     * how long to apply the active trajectory point by summing baseTrajDurationMs with the
     * timeDur of the trajectory point (see TrajectoryPoint).
     *
     * This allows general selection of the execution rate of the points with 1ms resolution,
     * while allowing some degree of change from point to point.
     *
     * * `base_traj_duration_ms` - The base duration time of every trajectory point.
     * 	 This is summed with the trajectory points unique timeDur.
     * * `timeoutMs` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    fn config_motion_profile_trajectory_period(
        &self,
        base_traj_duration_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigMotionProfileTrajectoryPeriod(
                self.get_handle(),
                base_traj_duration_ms,
                timeout_ms,
            )
        }
    }

    /**
     * Gets the last error generated by this object.
     * Not all functions return an error code but can potentially report errors.
     * This function can be used to retrieve those error codes.
     */
    fn get_last_error(&self) -> ErrorCode {
        unsafe { c_MotController_GetLastError(self.get_handle()) }
    }

    fn get_faults(&self) -> Result<Faults> {
        Ok(Faults(
            cci_get_call!(c_MotController_GetFaults(self.get_handle(), _: i32))?,
        ))
    }
    fn get_sticky_faults(&self) -> Result<StickyFaults> {
        Ok(StickyFaults(
            cci_get_call!(c_MotController_GetStickyFaults(self.get_handle(), _: i32))?,
        ))
    }
    fn clear_sticky_faults(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ClearStickyFaults(self.get_handle(), timeout_ms) }
    }

    /**
     * Gets the firmware version of the device.
     *
     * @return Firmware version of device.  For example: version 1-dot-2 is 0x0102.
     */
    fn get_firmware_version(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetFirmwareVersion(self.get_handle(), _: i32))
    }
    /// Returns true if the device has reset since last call.
    fn has_reset_occurred(&self) -> Result<bool> {
        cci_get_call!(c_MotController_HasResetOccurred(self.get_handle(), _: bool))
    }

    /**
     * Sets the value of a custom parameter. This is for arbitrary use.
     *
     * Sometimes it is necessary to save calibration/limit/target
     * information in the device. Particularly if the
     * device is part of a subsystem that can be replaced.
     *
     * @param newValue
     *            Value for custom parameter.
     * @param paramIndex
     *            Index of custom parameter [0,1]
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     */
    fn config_set_custom_param(
        &self,
        new_value: i32,
        param_index: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSetCustomParam(
                self.get_handle(),
                new_value,
                param_index,
                timeout_ms,
            )
        }
    }
    /**
     * Gets the value of a custom parameter.
     *
     * @param paramIndex
     *            Index of custom parameter [0,1].
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     */
    fn config_get_custom_param(&self, param_index: i32, timout_ms: i32) -> Result<i32> {
        cci_get_call!(c_MotController_ConfigGetCustomParam(
            self.get_handle(),
            _: i32,
            param_index,
            timout_ms,
        ))
    }

    /**
     * Sets a parameter. Generally this is not used.
     * This can be utilized in
     * - Using new features without updating API installation.
     * - Errata workarounds to circumvent API implementation.
     * - Allows for rapid testing / unit testing of firmware.
     */
    fn config_set_parameter(
        &self,
        param: ParamEnum,
        value: f64,
        sub_value: u8,
        ordinal: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSetParameter(
                self.get_handle(),
                param as _,
                value,
                sub_value as _,
                ordinal,
                timeout_ms,
            )
        }
    }
    fn config_get_parameter(&self, param: ParamEnum, ordinal: i32, timeout_ms: i32) -> Result<f64> {
        cci_get_call!(c_MotController_ConfigGetParameter(
            self.get_handle(),
            param as _,
            _: f64,
            ordinal,
            timeout_ms,
        ))
    }

    /**
     * Set the control mode and output value so that this motor controller will
     * follow another motor controller.
     * Currently supports following Victor SPX and Talon SRX.
     *
     * * `master_to_follow` - Motor Controller object to follow.
     * * `follower_type` - Type of following control.
     *   Use AuxOutput1 to follow the master device's auxiliary output 1.
     *   Use PercentOutput for standard follower mode.
     */
    fn follow<T: BaseMotorController>(&self, master_to_follow: &T, follower_type: FollowerType) {
        let base_id = master_to_follow.get_base_id();
        let id24: i32 = ((base_id >> 0x10) << 8) | (base_id & 0xFF);

        match follower_type {
            FollowerType::PercentOutput => {
                self.set(ControlMode::Follower, id24 as f64, DemandType::Neutral, 0.0)
            }
            FollowerType::AuxOutput1 => {
                self.set(ControlMode::Follower, id24 as f64, DemandType::AuxPID, 0.0)
            }
        };
    }
}

/// An interface for getting and setting raw sensor values.
pub trait SensorCollection: BaseMotorController {
    fn get_analog_in(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetAnalogIn(self.get_handle(), _: i32))
    }
    fn set_analog_position(&self, new_position: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_SetAnalogPosition(self.get_handle(), new_position, timeout_ms) }
    }
    fn get_analog_in_raw(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetAnalogInRaw(self.get_handle(), _: i32))
    }
    fn get_analog_in_vel(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetAnalogInVel(self.get_handle(), _: i32))
    }
    fn get_quadrature_position(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetQuadraturePosition(self.get_handle(), _: i32))
    }
    fn set_quadrature_position(&self, new_position: i32, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_SetQuadraturePosition(self.get_handle(), new_position, timeout_ms)
        }
    }
    fn get_quadrature_velocity(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetQuadratureVelocity(self.get_handle(), _: i32))
    }
    fn get_pulse_width_position(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPulseWidthPosition(self.get_handle(), _: i32))
    }
    fn set_pulse_width_position(&self, new_position: i32, timeout_ms: i32) -> ErrorCode {
        unsafe {
            c_MotController_SetPulseWidthPosition(self.get_handle(), new_position, timeout_ms)
        }
    }
    fn get_pulse_width_velocity(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPulseWidthVelocity(self.get_handle(), _: i32))
    }
    fn get_pulse_width_rise_to_fall_us(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPulseWidthRiseToFallUs(self.get_handle(), _: i32))
    }
    fn get_pulse_width_rise_to_rise_us(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPulseWidthRiseToRiseUs(self.get_handle(), _: i32))
    }
    fn get_pin_state_quad_a(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPinStateQuadA(self.get_handle(), _: i32))
    }
    fn get_pin_state_quad_b(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPinStateQuadB(self.get_handle(), _: i32))
    }
    fn get_pin_state_quad_idx(&self) -> Result<i32> {
        cci_get_call!(c_MotController_GetPinStateQuadIdx(self.get_handle(), _: i32))
    }
    fn is_fwd_limit_switch_closed(&self) -> Result<i32> {
        cci_get_call!(c_MotController_IsFwdLimitSwitchClosed(self.get_handle(), _: i32))
    }
    fn is_rev_limit_switch_closed(&self) -> Result<i32> {
        cci_get_call!(c_MotController_IsRevLimitSwitchClosed(self.get_handle(), _: i32))
    }
}

pub struct TalonSRX {
    handle: Handle,
    arb_id: i32,
}

impl BaseMotorController for TalonSRX {
    /**
     * Constructor
     * @param deviceNumber [0,62]
     */
    fn new(device_number: i32) -> TalonSRX {
        let arb_id = device_number | 0x02040000;
        let handle = unsafe { c_MotController_Create1(arb_id) };
        TalonSRX { handle, arb_id }
    }

    fn get_handle(&self) -> Handle {
        self.handle
    }
    fn get_base_id(&self) -> i32 {
        self.arb_id
    }
}

impl TalonSRX {
    pub fn config_selected_feedback_sensor(
        &self,
        feedback_device: FeedbackDevice,
        pid_idx: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigSelectedFeedbackSensor(
                self.handle,
                feedback_device as _,
                pid_idx,
                timeout_ms,
            )
        }
    }

    /*
    pub fn set_control_frame_period(
        &self,
        frame: ControlFrameEnhanced,
        period_ms: i32,
    ) -> ErrorCode {
        unsafe { c_MotController_SetControlFramePeriod(self.handle, frame as _, period_ms) }
    }
    */
    pub fn set_status_frame_period(
        &self,
        frame: StatusFrameEnhanced,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_SetStatusFramePeriod(self.handle, frame as _, period_ms, timeout_ms)
        }
    }
    pub fn get_status_frame_period(
        &self,
        frame: StatusFrameEnhanced,
        timeout_ms: i32,
    ) -> Result<i32> {
        cci_get_call!(c_MotController_GetStatusFramePeriod(
            self.handle,
            frame as _,
            _: i32,
            timeout_ms,
        ))
    }

    /**
     * Configures the period of each velocity sample.
     * Every 1ms a position value is sampled, and the delta between that sample
     * and the position sampled kPeriod ms ago is inserted into a filter.
     * kPeriod is configured with this function.
     */
    pub fn config_velocity_measurement_period(
        &self,
        period: VelocityMeasPeriod,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigVelocityMeasurementPeriod(self.handle, period as _, timeout_ms)
        }
    }
    /// Sets the number of velocity samples used in the rolling average velocity measurement.
    pub fn config_velocity_measurement_window(
        &self,
        window_size: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigVelocityMeasurementWindow(self.handle, window_size, timeout_ms)
        }
    }

    pub fn config_forward_limit_switch_source(
        &self,
        type_: LimitSwitchSource,
        normal_open_or_close: LimitSwitchNormal,
        device_id: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigForwardLimitSwitchSource(
                self.handle,
                type_ as _,
                normal_open_or_close as _,
                device_id,
                timeout_ms,
            )
        }
    }
    pub fn config_reverse_limit_switch_source(
        &self,
        type_: LimitSwitchSource,
        normal_open_or_close: LimitSwitchNormal,
        device_id: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_MotController_ConfigReverseLimitSwitchSource(
                self.handle,
                type_ as _,
                normal_open_or_close as _,
                device_id,
                timeout_ms,
            )
        }
    }

    pub fn config_peak_current_limit(&self, amps: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ConfigPeakCurrentLimit(self.handle, amps, timeout_ms) }
    }
    pub fn config_peak_current_duration(&self, milliseconds: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ConfigPeakCurrentLimit(self.handle, milliseconds, timeout_ms) }
    }
    pub fn config_continuous_current_limit(&self, amps: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_MotController_ConfigContinuousCurrentLimit(self.handle, amps, timeout_ms) }
    }
    pub fn enable_current_limit(&self, enable: bool) {
        unsafe { c_MotController_EnableCurrentLimit(self.handle, enable) };
    }
}

impl SensorCollection for TalonSRX {}

pub struct VictorSPX {
    handle: Handle,
    arb_id: i32,
}

impl BaseMotorController for VictorSPX {
    /**
     * Constructor
     * @param deviceNumber [0,62]
     */
    fn new(device_number: i32) -> VictorSPX {
        let arb_id = device_number | 0x01040000;
        let handle = unsafe { c_MotController_Create1(arb_id) };
        VictorSPX { handle, arb_id }
    }

    fn get_handle(&self) -> Handle {
        self.handle
    }
    fn get_base_id(&self) -> i32 {
        self.arb_id
    }
}
