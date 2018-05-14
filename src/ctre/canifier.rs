use ctre::{ErrorCode, ParamEnum, Result};
use ctre_sys::canifier::*;
pub use ctre_sys::canifier::{CANifierControlFrame as ControlFrame,
                             CANifierStatusFrame as StatusFrame,
                             CANifierVelocityMeasPeriod as VelocityMeasPeriod, GeneralPin};

#[repr(u32)]
/// Enum for the LED Output Channels
pub enum LEDChannel {
    A = 0,
    B = 1,
    C = 2,
}

#[repr(u32)]
/// Enum for the PWM Input Channels
pub enum PWMChannel {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
}
pub const PWM_CHANNEL_COUNT: usize = 4;

pub struct Faults(i32);
impl Faults {
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}
pub struct StickyFaults(i32);
impl StickyFaults {
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}

pub struct CANifier {
    handle: Handle,
}
impl CANifier {
    pub fn new(device_number: i32) -> CANifier {
        let handle = unsafe { c_CANifier_Create1(device_number) };
        CANifier { handle }
    }

    pub fn set_led_output(&self, duty_cycle: u32, led_channel: LEDChannel) -> ErrorCode {
        unsafe { c_CANifier_SetLEDOutput(self.handle, duty_cycle, led_channel as u32) }
    }
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

    pub fn set_pwm_output(&self, pwm_channel: PWMChannel, duty_cycle: u32) -> ErrorCode {
        unsafe { c_CANifier_SetPWMOutput(self.handle, pwm_channel as u32, duty_cycle) }
    }
    pub fn enable_pwm_output(&self, pwm_channel: PWMChannel, b_enable: bool) -> ErrorCode {
        unsafe { c_CANifier_EnablePWMOutput(self.handle, pwm_channel as u32, b_enable) }
    }

    pub fn _get_general_inputs(&self, all_pins: &mut [bool]) -> ErrorCode {
        unsafe {
            c_CANifier_GetGeneralInputs(self.handle, all_pins.as_mut_ptr(), all_pins.len() as _)
        }
    }
    pub fn get_general_input(&self, input_pin: u32) -> Result<bool> {
        cci_get_call!(c_CANifier_GetGeneralInput(self.handle, input_pin, _: bool))
    }

    pub fn get_pwm_input(&self, pwm_channel: u32) -> Result<[f64; 2]> {
        cci_get_call_array!(c_CANifier_GetPWMInput(
            self.handle,
            pwm_channel,
            _: [f64; 2],
        ))
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

    pub fn config_velocity_measurement_period(&self, period: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_ConfigVelocityMeasurementPeriod(self.handle, period, timeout_ms) }
    }
    pub fn config_velocity_measurement_window(&self, window: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_CANifier_ConfigVelocityMeasurementWindow(self.handle, window, timeout_ms) }
    }

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
    pub fn config_set_custom_param(
        &self,
        new_value: i32,
        param_index: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_CANifier_ConfigSetCustomParam(self.handle, new_value, param_index, timeout_ms) }
    }
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
