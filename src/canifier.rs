//! CANifier

use ctre_sys::canifier::*;
pub use ctre_sys::canifier::{
    CANifierControlFrame as ControlFrame, CANifierStatusFrame as StatusFrame,
    CANifierVelocityMeasPeriod as VelocityMeasPeriod, GeneralPin,
};
#[cfg(feature = "usage-reporting")]
use wpilib_sys::usage::report_usage;

use super::{ErrorCode, ParamEnum, Result};

#[repr(u32)]
/// Enum for the LED Output Channels
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum LEDChannel {
    A = 0,
    B = 1,
    C = 2,
}

#[repr(u32)]
/// Enum for the PWM Input Channels
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PWMChannel {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
}
pub const PWM_CHANNEL_COUNT: usize = 4;

#[allow(non_snake_case)]
/// Structure to hold the pin values.
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
pub struct PinValues {
    pub QUAD_IDX: bool,
    pub QUAD_B: bool,
    pub QUAD_A: bool,
    pub LIMR: bool,
    pub LIMF: bool,
    pub SDA: bool,
    pub SCL: bool,
    pub SPI_CS_PWM3: bool,
    pub SPI_MISO_PWM2: bool,
    pub SPI_MOSI_PWM1: bool,
    pub SPI_CLK_PWM0: bool,
}

#[derive(Debug, Copy, Clone)]
pub struct Faults(i32);
impl Faults {
    pub fn has_any_fault(self) -> bool {
        self.0 != 0
    }
}
#[derive(Debug, Copy, Clone)]
pub struct StickyFaults(i32);
impl StickyFaults {
    pub fn has_any_fault(self) -> bool {
        self.0 != 0
    }
}

/**
 * CTRE CANifier
 *
 * Device for interfacing common devices to the CAN bus.
 */
#[derive(Debug)]
pub struct CANifier {
    handle: Handle,
}
impl CANifier {
    /// Constructor.
    /// * `device_number` - The CAN Device ID of the CANifier.
    pub fn new(device_number: i32) -> CANifier {
        let handle = unsafe { c_CANifier_Create1(device_number) };
        // kResourceType_CANifier
        #[cfg(feature = "usage-reporting")]
        report_usage(63, device_number as u32 + 1);
        CANifier { handle }
    }

    pub fn _set_led_output(&self, duty_cycle: u32, led_channel: LEDChannel) -> ErrorCode {
        unsafe { c_CANifier_SetLEDOutput(self.handle, duty_cycle, led_channel as u32) }
    }
    pub fn set_led_output(&self, percent_output: f64, led_channel: LEDChannel) -> ErrorCode {
        // convert float to integral fixed pt
        let duty_cycle = 1023. * percent_output.min(1.).max(0.);
        self._set_led_output(duty_cycle as u32, led_channel)
    }
    /**
     * Sets the output of all General Pins
     * * `output_bits` - A bit mask of all the output states.
     *   LSB->MSB is in the order of the GeneralPin enum.
     * * `is_output_bits` - A boolean bit mask that sets the pins to be outputs or inputs.
     *   A bit of 1 enables output.
     */
    pub fn set_general_outputs(&self, outputs_bits: u32, is_output_bits: u32) -> ErrorCode {
        unsafe { c_CANifier_SetGeneralOutputs(self.handle, outputs_bits, is_output_bits) }
    }
    pub fn set_general_output(
        &self,
        output_pin: GeneralPin,
        output_value: bool,
        output_enable: bool,
    ) -> ErrorCode {
        unsafe {
            c_CANifier_SetGeneralOutput(self.handle, output_pin as u32, output_value, output_enable)
        }
    }

    pub fn _set_pwm_output(&self, pwm_channel: u32, duty_cycle: u32) -> ErrorCode {
        unsafe { c_CANifier_SetPWMOutput(self.handle, pwm_channel, duty_cycle) }
    }
    /**
     * Sets the PWM Output
     * Currently supports PWM 0, PWM 1, and PWM 2
     * * `pwm_channel` - Index of the PWM channel to output.
     * * `duty_cycle` - Duty Cycle (0 to 1) to output.
     *   Default period of the signal is 4.2 ms.
     */
    pub fn set_pwm_output(&self, pwm_channel: PWMChannel, duty_cycle: f64) -> ErrorCode {
        let duty_cyc_10bit = 1023. * duty_cycle.max(0.).min(1.);
        self._set_pwm_output(pwm_channel as u32, duty_cyc_10bit as u32)
    }
    pub fn _enable_pwm_output(&self, pwm_channel: u32, b_enable: bool) -> ErrorCode {
        unsafe { c_CANifier_EnablePWMOutput(self.handle, pwm_channel, b_enable) }
    }
    /**
     * Enables PWM Outputs
     * Currently supports PWM 0, PWM 1, and PWM 2
     */
    pub fn enable_pwm_output(&self, pwm_channel: PWMChannel, enable: bool) -> ErrorCode {
        self._enable_pwm_output(pwm_channel as u32, enable)
    }

    pub fn _get_general_inputs(&self, all_pins: &mut [bool]) -> ErrorCode {
        unsafe {
            c_CANifier_GetGeneralInputs(self.handle, all_pins.as_mut_ptr(), all_pins.len() as _)
        }
    }
    /// Gets the state of all General Pins
    pub fn get_general_inputs(&self) -> Result<PinValues> {
        let mut temp_pins = [false; 11];
        let err = self._get_general_inputs(&mut temp_pins);
        match err {
            ErrorCode::OK => Ok(PinValues {
                LIMF: temp_pins[GeneralPin::LIMF as usize],
                LIMR: temp_pins[GeneralPin::LIMR as usize],
                QUAD_A: temp_pins[GeneralPin::QUAD_A as usize],
                QUAD_B: temp_pins[GeneralPin::QUAD_B as usize],
                QUAD_IDX: temp_pins[GeneralPin::QUAD_IDX as usize],
                SCL: temp_pins[GeneralPin::SCL as usize],
                SDA: temp_pins[GeneralPin::SDA as usize],
                SPI_CLK_PWM0: temp_pins[GeneralPin::SPI_CLK_PWM0P as usize],
                SPI_MOSI_PWM1: temp_pins[GeneralPin::SPI_MOSI_PWM1P as usize],
                SPI_MISO_PWM2: temp_pins[GeneralPin::SPI_MISO_PWM2P as usize],
                SPI_CS_PWM3: temp_pins[GeneralPin::SPI_CS as usize],
            }),
            _ => Err(err),
        }
    }
    /// Gets the state of the specified pin
    pub fn get_general_input(&self, input_pin: GeneralPin) -> Result<bool> {
        cci_get_call!(c_CANifier_GetGeneralInput(self.handle, input_pin as u32, _: bool))
    }

    /// Gets the PWM Input.
    /// Returns a 2-array holding the Pulse Width (microseconds) [0] and Period (microseconds) [1].
    pub fn get_pwm_input(&self, pwm_channel: PWMChannel) -> Result<[f64; 2]> {
        cci_get_call_array!(c_CANifier_GetPWMInput(self.handle, pwm_channel as u32, _: [f64; 2]))
    }

    pub fn get_last_error(&self) -> ErrorCode {
        unsafe { c_CANifier_GetLastError(self.handle) }
    }
    pub fn get_bus_voltage(&self) -> Result<f64> {
        cci_get_call!(c_CANifier_GetBusVoltage(self.handle, _: f64))
    }

    pub fn get_quadrature_position(&self) -> Result<i32> {
        cci_get_call!(c_CANifier_GetQuadraturePosition(self.handle, _: i32))
    }
    pub fn set_quadrature_position(&self, pos: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_SetQuadraturePosition(self.handle, pos, timeout_ms) }
    }
    pub fn get_quadrature_velocity(&self) -> Result<i32> {
        cci_get_call!(c_CANifier_GetQuadratureVelocity(self.handle, _: i32))
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
        unsafe { c_CANifier_ConfigVelocityMeasurementPeriod(self.handle, period as _, timeout_ms) }
    }
    pub fn config_velocity_measurement_window(&self, window: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_ConfigVelocityMeasurementWindow(self.handle, window, timeout_ms) }
    }

    /**
     * Sets a parameter. Generally this is not used.
     * This can be utilized in
     * - Using new features without updating API installation.
     * - Errata workarounds to circumvent API implementation.
     * - Allows for rapid testing / unit testing of firmware.
     */
    pub fn config_set_parameter(
        &self,
        param: ParamEnum,
        value: f64,
        sub_value: i32,
        ordinal: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_CANifier_ConfigSetParameter(
                self.handle,
                param as _,
                value,
                sub_value,
                ordinal,
                timeout_ms,
            )
        }
    }
    pub fn config_get_parameter(
        &self,
        param: ParamEnum,
        ordinal: i32,
        timeout_ms: i32,
    ) -> Result<f64> {
        cci_get_call!(c_CANifier_ConfigGetParameter(
            self.handle,
            param as _,
            _: f64,
            ordinal,
            timeout_ms,
        ))
    }
    /**
     * Sets the value of a custom parameter. This is for arbitrary use.
     *
     * Sometimes it is necessary to save calibration/duty cycle/output
     * information in the device. Particularly if the
     * device is part of a subsystem that can be replaced.
     *
     * * `new_value` - Value for custom parameter.
     * * `param_index` - Index of custom parameter [0,1]
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn config_set_custom_param(
        &self,
        new_value: i32,
        param_index: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_CANifier_ConfigSetCustomParam(self.handle, new_value, param_index, timeout_ms) }
    }
    /**
     * Gets the value of a custom parameter. This is for arbitrary use.
     *
     * * `param_index` - Index of custom parameter [0,1].
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn config_get_custom_param(&self, param_index: i32, timout_ms: i32) -> Result<i32> {
        cci_get_call!(c_CANifier_ConfigGetCustomParam(self.handle, _: i32, param_index, timout_ms))
    }

    pub fn get_faults(&self) -> Result<Faults> {
        Ok(Faults(
            cci_get_call!(c_CANifier_GetFaults(self.handle, _: i32))?,
        ))
    }
    pub fn get_sticky_faults(&self) -> Result<StickyFaults> {
        Ok(StickyFaults(
            cci_get_call!(c_CANifier_GetStickyFaults(self.handle, _: i32))?,
        ))
    }
    pub fn clear_sticky_faults(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_ClearStickyFaults(self.handle, timeout_ms) }
    }

    pub fn get_firmware_version(&self) -> Result<i32> {
        cci_get_call!(c_CANifier_GetFirmwareVersion(self.handle, _: i32))
    }
    pub fn has_reset_occurred(&self) -> Result<bool> {
        cci_get_call!(c_CANifier_HasResetOccurred(self.handle, _: bool))
    }

    pub fn set_status_frame_period(
        &self,
        frame: StatusFrame,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_CANifier_SetStatusFramePeriod(self.handle, frame as _, period_ms, timeout_ms) }
    }
    pub fn get_status_frame_period(&self, frame: StatusFrame, timeout_ms: i32) -> Result<i32> {
        cci_get_call!(c_CANifier_GetStatusFramePeriod(self.handle, frame as _, _: i32, timeout_ms))
    }
    pub fn set_control_frame_period(&self, frame: ControlFrame, period_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_SetControlFramePeriod(self.handle, frame as _, period_ms) }
    }
}
