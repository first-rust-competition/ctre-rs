use std::fmt;
use ctre::{ErrorCode, ParamEnum, Result};
use ctre::motor_control::{BaseMotorController, TalonSRX};
use ctre_sys::pigeon::*;
pub use ctre_sys::pigeon::{PigeonIMU_ControlFrame as ControlFrame,
                           PigeonIMU_StatusFrame as StatusFrame};

#[derive(Default)]
pub struct FusionStatus {
    pub is_fusing: bool, // int
    pub is_valid: bool,  // int
    pub heading: f64,
    /// Same as getLastError()
    last_error: i32,
}
impl fmt::Debug for FusionStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            if self.last_error != 0 {
                "Could not receive status frame.  Check wiring and web-config."
            } else if !self.is_valid {
                "Fused Heading is not valid."
            } else if !self.is_fusing {
                "Fused Heading is valid."
            } else {
                "Fused Heading is valid and is fusing compass."
            }
        )
    }
}

#[repr(i32)]
/// Various calibration modes supported by Pigeon.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum CalibrationMode {
    BootTareGyroAccel = 0,
    Temperature = 1,
    Magnetometer12Pt = 2,
    Magnetometer360 = 3,
    Accelerometer = 5,
    Unknown = -1,
}
impl Default for CalibrationMode {
    fn default() -> CalibrationMode {
        CalibrationMode::Unknown
    }
}
impl From<i32> for CalibrationMode {
    fn from(v: i32) -> CalibrationMode {
        match v {
            0 => CalibrationMode::BootTareGyroAccel,
            1 => CalibrationMode::Temperature,
            2 => CalibrationMode::Magnetometer12Pt,
            3 => CalibrationMode::Magnetometer360,
            5 => CalibrationMode::Accelerometer,
            _ => CalibrationMode::Unknown,
        }
    }
}

#[repr(i32)]
/// Overall state of the Pigeon.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum PigeonState {
    NoComm = 0,
    Initializing = 1,
    Ready = 2,
    UserCalibration = 3,
    Unknown = -1,
}
impl Default for PigeonState {
    fn default() -> PigeonState {
        PigeonState::Unknown
    }
}
impl From<i32> for PigeonState {
    fn from(v: i32) -> PigeonState {
        match v {
            0 => PigeonState::NoComm,
            1 => PigeonState::Initializing,
            2 => PigeonState::Ready,
            3 => PigeonState::UserCalibration,
            _ => PigeonState::Unknown,
        }
    }
}

#[derive(Default)]
pub struct GeneralStatus {
    pub state: PigeonState,
    pub current_mode: CalibrationMode,
    pub calibration_error: i32,
    pub cal_is_booting: bool, // int
    pub temp_c: f64,
    pub up_time_sec: i32,
    pub no_motion_bias_count: i32,
    pub temp_compensation_count: i32,
    /// Same as getLastError()
    last_error: i32,
}
impl fmt::Debug for GeneralStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "{}",
            if self.last_error != 0 {
                // same as NoComm
                "Status frame was not received, check wired connections and web-based config."
            } else if self.cal_is_booting {
                "Pigeon is boot-caling to properly bias accel and gyro.  Do not move Pigeon.  When finished biasing, calibration mode will start."
            } else {
                match self.state {
                    PigeonState::UserCalibration => match self.current_mode {
                        CalibrationMode::BootTareGyroAccel => "",
                        CalibrationMode::Temperature => "",
                        CalibrationMode::Magnetometer12Pt => "",
                        CalibrationMode::Magnetometer360 => "",
                        CalibrationMode::Accelerometer => "",
                        _ => "",
                    },
                    PigeonState::Ready => "",
                    PigeonState::Initializing => "",
                    _ => "Not enough data to determine status.",
                }
            }
        )
    }
}

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

    pub fn get_general_status(&self) -> Result<GeneralStatus> {
        let mut status: GeneralStatus = Default::default();
        let mut state = 0;
        let mut current_mode = 0;
        let mut b_cal_is_booting = 0;
        let err = unsafe {
            c_PigeonIMU_GetGeneralStatus(
                self.handle,
                &mut state,
                &mut current_mode,
                &mut status.calibration_error,
                &mut b_cal_is_booting,
                &mut status.temp_c,
                &mut status.up_time_sec,
                &mut status.no_motion_bias_count,
                &mut status.temp_compensation_count,
                &mut status.last_error,
            )
        };
        if err == ErrorCode::OK {
            status.state = PigeonState::from(state);
            status.current_mode = CalibrationMode::from(current_mode);
            status.cal_is_booting = b_cal_is_booting != 0;
            Ok(status)
        } else {
            Err(err)
        }
    }

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

    pub fn get_fused_heading2(&self) -> Result<FusionStatus> {
        let mut status: FusionStatus = Default::default();
        let mut b_is_fusing = 0;
        let mut b_is_valid = 0;
        let err = unsafe {
            c_PigeonIMU_GetFusedHeading2(
                self.handle,
                &mut b_is_fusing,
                &mut b_is_valid,
                &mut status.heading,
                &mut status.last_error,
            )
        };
        if err == ErrorCode::OK {
            status.is_fusing = b_is_fusing != 0;
            status.is_valid = b_is_valid != 0;
            Ok(status)
        } else {
            Err(err)
        }
    }
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
        frame: StatusFrame,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_PigeonIMU_SetStatusFramePeriod(self.handle, frame as _, period_ms, timeout_ms) }
    }
    pub fn get_status_frame_period(&self, frame: StatusFrame, timeout_ms: i32) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetStatusFramePeriod(self.handle, frame as _, _: i32, timeout_ms))
    }
    pub fn set_control_frame_period(&self, frame: ControlFrame, period_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetControlFramePeriod(self.handle, frame as _, period_ms) }
    }
}
