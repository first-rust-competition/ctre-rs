use ctre::{ErrorCode, ParamEnum, Result};
use ctre::motor_control::{BaseMotorController, TalonSRX};
use ctre_sys::pigeon::*;
pub use ctre_sys::pigeon::{PigeonIMU_ControlFrame, PigeonIMU_StatusFrame};

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

pub struct PigeonIMU {
    handle: Handle,
}
impl PigeonIMU {
    pub fn new(device_number: i32) -> PigeonIMU {
        let handle = unsafe { c_PigeonIMU_Create1(device_number) };
        PigeonIMU { handle }
    }
    pub fn from_talon(talon_srx: &TalonSRX) -> Result<PigeonIMU> {
        let talon_device_id = talon_srx.get_device_id()?;
        let handle = unsafe { c_PigeonIMU_Create2(talon_device_id) };
        Ok(PigeonIMU { handle })
    }

    pub fn set_yaw(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYaw(self.handle, angle_deg, timeout_ms) }
    }
    pub fn add_yaw(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddYaw(self.handle, angle_deg, timeout_ms) }
    }
    pub fn set_yaw_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYawToCompass(self.handle, timeout_ms) }
    }

    pub fn set_fused_heading(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeading(self.handle, angle_deg, timeout_ms) }
    }
    pub fn add_fused_heading(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddFusedHeading(self.handle, angle_deg, timeout_ms) }
    }
    pub fn set_fused_heading_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeadingToCompass(self.handle, timeout_ms) }
    }
    pub fn set_accum_z_angle(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetAccumZAngle(self.handle, angle_deg, timeout_ms) }
    }

    pub fn config_temperature_compensation_enable(
        &self,
        b_temp_comp_enable: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_PigeonIMU_ConfigTemperatureCompensationEnable(
                self.handle,
                b_temp_comp_enable,
                timeout_ms,
            )
        }
    }

    pub fn set_compass_declination(&self, angle_deg_offset: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassDeclination(self.handle, angle_deg_offset, timeout_ms) }
    }
    pub fn set_compass_angle(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassAngle(self.handle, angle_deg, timeout_ms) }
    }

    pub fn enter_calibration_mode(&self, cal_mode: i32, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_EnterCalibrationMode(self.handle, cal_mode, timeout_ms) }
    }

    /*
    pub fn get_general_status(&self) -> Result<(i32, i32, i32, i32, f64, i32, i32, i32, i32)> {
        cci_get_call!(c_PigeonIMU_GetGeneralStatus(
            self.handle,
            _: i32, // state
            _: i32, // current_mode
            _: i32, // calibration_error
            _: i32, // b_cal_is_booting
            _: f64, // temp_c
            _: i32, // up_time_sec
            _: i32, // no_motion_bias_count
            _: i32, // temp_compensation_count
            _: i32, // last_error
        ))
    }
    */

    pub fn get_last_error(&self) -> ErrorCode {
        unsafe { c_PigeonIMU_GetLastError(self.handle) }
    }

    pub fn get6d_quaternion(&self) -> Result<[f64; 4]> {
        cci_get_call_array!(c_PigeonIMU_Get6dQuaternion(
            self.handle,
            _: [f64; 4],
        ))
    }
    pub fn get_yaw_pitch_roll(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetYawPitchRoll(
            self.handle,
            _: [f64; 3],
        ))
    }
    pub fn get_accum_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetAccumGyro(
            self.handle,
            _: [f64; 3],
        ))
    }
    pub fn get_absolute_compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetAbsoluteCompassHeading(self.handle, _: f64))
    }
    pub fn get_compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassHeading(self.handle, _: f64))
    }
    pub fn get_compass_field_strength(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassFieldStrength(self.handle, _: f64))
    }
    pub fn get_temp(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetTemp(self.handle, _: f64))
    }
    pub fn get_state(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetState(self.handle, _: i32))
    }
    pub fn get_up_time(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetUpTime(self.handle, _: i32))
    }

    pub fn get_raw_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetRawMagnetometer(
            self.handle,
            _: [i16; 3],
        ))
    }
    pub fn get_biased_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetBiasedMagnetometer(
            self.handle,
            _: [i16; 3],
        ))
    }
    pub fn get_biased_accelerometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetBiasedAccelerometer(
            self.handle,
            _: [i16; 3],
        ))
    }
    pub fn get_raw_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetRawGyro(
            self.handle,
            _: [f64; 3],
        ))
    }
    pub fn get_accelerometer_angles(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetAccelerometerAngles(
            self.handle,
            _: [f64; 3],
        ))
    }

    /*
    pub fn get_fused_heading2(&self) -> Result<(i32, i32, f64, i32)> {
        cci_get_call!(c_PigeonIMU_GetFusedHeading2(
            self.handle,
            _: i32, // b_is_fusing
            _: i32, // b_is_valid
            _: f64, // value
            _: i32, // last_error
        ))
    }
    */
    pub fn get_fused_heading1(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetFusedHeading1(self.handle, _: f64))
    }

    pub fn get_reset_count(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetResetCount(self.handle, _: i32))
    }
    pub fn get_reset_flags(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetResetFlags(self.handle, _: i32))
    }
    pub fn get_firmware_version(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetFirmwareVersion(self.handle, _: i32))
    }
    pub fn has_reset_occurred(&self) -> Result<bool> {
        cci_get_call!(c_PigeonIMU_HasResetOccurred(self.handle, _: bool))
    }

    pub fn config_set_custom_param(
        &self,
        new_value: i32,
        param_index: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_PigeonIMU_ConfigSetCustomParam(self.handle, new_value, param_index, timeout_ms) }
    }
    pub fn config_get_custom_param(&self, param_index: i32, timout_ms: i32) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_ConfigGetCustomParam(
            self.handle,
            _: i32,
            param_index,
            timout_ms,
        ))
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
            c_PigeonIMU_ConfigSetParameter(
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
        cci_get_call!(c_PigeonIMU_ConfigGetParameter(
            self.handle,
            param as _,
            _: f64,
            ordinal,
            timeout_ms,
        ))
    }

    pub fn get_faults(&self) -> Result<Faults> {
        Ok(Faults(
            cci_get_call!(c_PigeonIMU_GetFaults(self.handle, _: i32))?,
        ))
    }
    pub fn get_sticky_faults(&self) -> Result<StickyFaults> {
        Ok(StickyFaults(
            cci_get_call!(c_PigeonIMU_GetStickyFaults(self.handle, _: i32))?,
        ))
    }
    pub fn clear_sticky_faults(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_ClearStickyFaults(self.handle, timeout_ms) }
    }

    pub fn set_status_frame_period(
        &self,
        frame: PigeonIMU_StatusFrame,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_PigeonIMU_SetStatusFramePeriod(self.handle, frame as _, period_ms, timeout_ms) }
    }
    pub fn get_status_frame_period(
        &self,
        frame: PigeonIMU_StatusFrame,
        timeout_ms: i32,
    ) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetStatusFramePeriod(self.handle, frame as _, _: i32, timeout_ms))
    }
    pub fn set_control_frame_period(
        &self,
        frame: PigeonIMU_ControlFrame,
        period_ms: i32,
    ) -> ErrorCode {
        unsafe { c_PigeonIMU_SetControlFramePeriod(self.handle, frame as _, period_ms) }
    }
}
