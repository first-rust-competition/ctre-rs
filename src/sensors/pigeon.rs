//! Pigeon IMU

use ctre_sys::pigeon::*;
pub use ctre_sys::pigeon::{
    PigeonIMU_ControlFrame as ControlFrame, PigeonIMU_StatusFrame as StatusFrame,
};
use motor_control::{BaseMotorController, TalonSRX};
use std::fmt;
use {ErrorCode, ParamEnum, Result};

/// Data object for holding fusion information.
#[derive(Default, Debug)]
pub struct FusionStatus {
    pub is_fusing: bool, // int
    pub is_valid: bool,  // int
    pub heading: f64,
    /// Same as getLastError()
    last_error: i32,
}
impl fmt::Display for FusionStatus {
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
    #[inline]
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
    #[inline]
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

/**
 * Data object for status on current calibration and general status.
 *
 * Pigeon has many calibration modes supported for a variety of uses. The
 * modes generally collects and saves persistently information that makes
 * the Pigeon signals more accurate. This includes collecting temperature,
 * gyro, accelerometer, and compass information.
 *
 * For FRC use-cases, typically compass and temperature calibration is not required.
 *
 * Additionally when motion driver software in the Pigeon boots, it will
 * perform a fast boot calibration to initially bias gyro and setup accelerometer.
 *
 * These modes can be enabled with the EnterCalibration mode.
 *
 * When a calibration mode is entered, caller can expect...
 *
 * - PigeonState to reset to Initializing and bCalIsBooting is set to true.
 * Pigeon LEDs will blink the boot pattern. This is similar to the normal
 * boot cal, however it can an additional ~30 seconds since calibration
 * generally requires more information. currentMode will reflect the user's
 * selected calibration mode.
 *
 * - PigeonState will eventually settle to UserCalibration and Pigeon LEDs
 * will show cal specific blink patterns. bCalIsBooting is now false.
 *
 * - Follow the instructions in the Pigeon User Manual to meet the
 * calibration specific requirements. When finished calibrationError will
 * update with the result. Pigeon will solid-fill LEDs with red (for
 * failure) or green (for success) for ~5 seconds. Pigeon then perform
 * boot-cal to cleanly apply the newly saved calibration data.
 */
#[derive(Default, Debug)]
pub struct GeneralStatus {
    /**
     * The current state of the motion driver.  This reflects if the sensor signals are accurate.
     * Most calibration modes will force Pigeon to reinit the motion driver.
     */
    pub state: PigeonState,
    /**
     * The currently applied calibration mode if state is in UserCalibration
     * or if `cal_is_booting` is true. Otherwise it holds the last selected
     * calibration mode (when calibrationError was updated).
     */
    pub current_mode: CalibrationMode,
    /**
     * The error code for the last calibration mode.
     * Zero represents a successful cal (with solid green LEDs at end of cal)
     * and nonzero is a failed calibration (with solid red LEDs at end of cal).
     * Different calibration
     */
    pub calibration_error: i32,
    /**
     * After caller requests a calibration mode, pigeon will perform a boot-cal before
     * entering the requested mode.  During this period, this flag is set to true.
     */
    pub cal_is_booting: bool, // int
    /// Temperature in Celsius
    pub temp_c: f64,
    /**
     * Number of seconds Pigeon has been up (since boot).
     * This register is reset on power boot or processor reset.
     * Register is capped at 255 seconds with no wrap around.
     */
    pub up_time_sec: i32,
    /**
     * Number of times the Pigeon has automatically rebiased the gyro.
     * This counter overflows from 15 -> 0 with no cap.
     */
    pub no_motion_bias_count: i32,
    /**
     * Number of times the Pigeon has temperature compensated the various signals.
     * This counter overflows from 15 -> 0 with no cap.
     */
    pub temp_compensation_count: i32,
    /// Same as getLastError()
    last_error: i32,
}
impl fmt::Display for GeneralStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.state == PigeonState::Ready {
            return write!(
                f,
                "Pigeon is running normally.  Last CAL error code was {}.",
                self.calibration_error,
            );
        }
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
                        CalibrationMode::BootTareGyroAccel =>
                            "Boot-Calibration: Gyro and Accelerometer are being biased.",
                        CalibrationMode::Temperature =>
                            "Temperature-Calibration: Pigeon is collecting temp data and will finish when temp range is reached. Do not move Pigeon.",
                        CalibrationMode::Magnetometer12Pt =>
                            "Magnetometer Level 1 calibration: Orient the Pigeon PCB in the 12 positions documented in the User's Manual.",
                        CalibrationMode::Magnetometer360 =>
                            "Magnetometer Level 2 calibration: Spin robot slowly in 360' fashion.",
                        CalibrationMode::Accelerometer =>
                            "Accelerometer Calibration: Pigeon PCB must be placed on a level source.  Follow User's Guide for how to level surfacee.",
                        _ => "Unknown status",
                    },
                    PigeonState::Initializing =>
                        "Pigeon is boot-caling to properly bias accel and gyro.  Do not move Pigeon.",
                    _ => "Not enough data to determine status.",
                }
            }
        )
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct Faults(i32);
impl Faults {
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}
#[derive(Default, Debug, Copy, Clone)]
pub struct StickyFaults(i32);
impl StickyFaults {
    pub fn has_any_fault(&self) -> bool {
        self.0 != 0
    }
}

/**
 * Pigeon IMU Class.
 * Class supports communicating over CANbus and over ribbon-cable (CAN Talon SRX).
 */
pub struct PigeonIMU {
    handle: Handle,
}
impl PigeonIMU {
    /// Create a Pigeon object that communicates with Pigeon on CAN Bus.
    /// * `device_number` - CAN Device Id of Pigeon [0,62]
    pub fn new(device_number: i32) -> PigeonIMU {
        let handle = unsafe { c_PigeonIMU_Create1(device_number) };
        PigeonIMU { handle }
    }

    /**
     * Sets the Yaw register to the specified value.
     *
     * * `angle_deg` - Degree of Yaw [+/- 23040 degrees]
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn set_yaw(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYaw(self.handle, angle_deg, timeout_ms) }
    }
    /// Atomically add to the Yaw register.
    pub fn add_yaw(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddYaw(self.handle, angle_deg, timeout_ms) }
    }
    /// Sets the Yaw register to match the current compass value.
    pub fn set_yaw_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYawToCompass(self.handle, timeout_ms) }
    }

    /**
     * Sets the Fused Heading to the specified value.
     *
     * * `angle_deg` - Degree of heading [+/- 23040 degrees]
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn set_fused_heading(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeading(self.handle, angle_deg, timeout_ms) }
    }
    /// Atomically add to the Fused Heading register.
    pub fn add_fused_heading(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddFusedHeading(self.handle, angle_deg, timeout_ms) }
    }
    /// Sets the Fused Heading register to match the current compass value.
    pub fn set_fused_heading_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeadingToCompass(self.handle, timeout_ms) }
    }
    pub fn set_accum_z_angle(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetAccumZAngle(self.handle, angle_deg, timeout_ms) }
    }

    /// Enable/Disable Temp compensation. Pigeon defaults with this on at boot.
    pub fn config_temperature_compensation_enable(
        &self,
        enable: bool,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe {
            c_PigeonIMU_ConfigTemperatureCompensationEnable(self.handle, enable as _, timeout_ms)
        }
    }

    /// Set the declination for compass. Declination is the difference between
    /// Earth Magnetic north, and the geographic "True North".
    pub fn set_compass_declination(&self, angle_deg_offset: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassDeclination(self.handle, angle_deg_offset, timeout_ms) }
    }
    /// Sets the compass angle. Although compass is absolute [0,360) degrees, the
    /// continuous compass register holds the wrap-arounds.
    pub fn set_compass_angle(&self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassAngle(self.handle, angle_deg, timeout_ms) }
    }

    pub fn enter_calibration_mode(&self, cal_mode: CalibrationMode, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_EnterCalibrationMode(self.handle, cal_mode as _, timeout_ms) }
    }

    /// Get the status of the current (or previousley complete) calibration.
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
            status.state = state.into();
            status.current_mode = current_mode.into();
            status.cal_is_booting = b_cal_is_booting != 0;
            Ok(status)
        } else {
            Err(err)
        }
    }

    pub fn get_last_error(&self) -> ErrorCode {
        unsafe { c_PigeonIMU_GetLastError(self.handle) }
    }

    /// Get 6d Quaternion data.
    /// Returns an array of the wxyz quaternion data.
    pub fn get6d_quaternion(&self) -> Result<[f64; 4]> {
        cci_get_call_array!(c_PigeonIMU_Get6dQuaternion(self.handle, _: [f64; 4]))
    }
    /// Get Yaw, Pitch, and Roll data.
    /// Returns an array with yaw, pitch, and roll, in that order.
    pub fn get_yaw_pitch_roll(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetYawPitchRoll(self.handle, _: [f64; 3]))
    }
    /**
     * Get AccumGyro data.
     * AccumGyro is the integrated gyro value on each axis.
     *
     * Returns an array `xyz_deg`.
     */
    pub fn get_accum_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetAccumGyro(self.handle, _: [f64; 3]))
    }
    /// Get the absolute compass heading, in the interval [0, 360) degrees.
    pub fn get_absolute_compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetAbsoluteCompassHeading(self.handle, _: f64))
    }
    /// Get the continuous compass heading, in the interval [-23040, 23040) degrees.
    pub fn get_compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassHeading(self.handle, _: f64))
    }
    /// Get the compass' measured magnetic field strength in microteslas (uT).
    pub fn get_compass_field_strength(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassFieldStrength(self.handle, _: f64))
    }
    /// Get the temperature of the pigeon, in degrees Celsius.
    pub fn get_temp(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetTemp(self.handle, _: f64))
    }
    pub fn get_state(&self) -> Result<PigeonState> {
        Ok(cci_get_call!(c_PigeonIMU_GetState(self.handle, _: i32))?.into())
    }
    pub fn get_up_time(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetUpTime(self.handle, _: i32))
    }

    /// Get Raw Magnetometer data.
    /// Returns an array `rm_xyz`.  Number is equal to 0.6 microteslas per unit.
    pub fn get_raw_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetRawMagnetometer(self.handle, _: [i16; 3]))
    }
    /// Get Biased Magnetometer data.
    /// Returns an array `bm_xyz`.  Number is equal to 0.6 microteslas per unit.
    pub fn get_biased_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetBiasedMagnetometer(self.handle, _: [i16; 3]))
    }
    /// Get Biased Accelerometer data.
    /// Returns an array `ba_xyz`.  These are in fixed point notation Q2.14.  eg. 16384 = 1G
    pub fn get_biased_accelerometer(&self) -> Result<[i16; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetBiasedAccelerometer(self.handle, _: [i16; 3]))
    }
    /// Get Raw Gyro data.
    /// Returns an array `xyz_dps`, with data in degrees per second.
    pub fn get_raw_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetRawGyro(self.handle, _: [f64; 3]))
    }
    /// Get Accelerometer tilt angles.
    /// Returns a 3-array of x, y, z angles in degrees.
    pub fn get_accelerometer_angles(&self) -> Result<[f64; 3]> {
        cci_get_call_array!(c_PigeonIMU_GetAccelerometerAngles(self.handle, _: [f64; 3]))
    }

    /// Get the current Fusion Status (including fused heading)
    pub fn get_fusion_status(&self) -> Result<FusionStatus> {
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
    /// Gets the Fused Heading in degrees.
    pub fn get_fused_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetFusedHeading1(self.handle, _: f64))
    }

    // Use `has_reset_occurred` instead.
    /*
    pub fn get_reset_count(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetResetCount(self.handle, _: i32))
    }
    // XXX: C++ exposes GetResetCount here again, Java exposes neither of these.
    pub fn get_reset_flags(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetResetFlags(self.handle, _: i32))
    }
    */
    pub fn get_firmware_version(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetFirmwareVersion(self.handle, _: i32))
    }
    pub fn has_reset_occurred(&self) -> Result<bool> {
        cci_get_call!(c_PigeonIMU_HasResetOccurred(self.handle, _: bool))
    }

    /**
     * Sets the value of a custom parameter. This is for arbitrary use.
     *
     * Sometimes it is necessary to save calibration/declination/offset
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
        unsafe { c_PigeonIMU_ConfigSetCustomParam(self.handle, new_value, param_index, timeout_ms) }
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
        cci_get_call!(c_PigeonIMU_ConfigGetCustomParam(
            self.handle,
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
impl<'a> From<&'a TalonSRX> for PigeonIMU {
    /// Create a Pigeon object that communicates with Pigeon through the
    /// Gadgeteer ribbon cable connected to a Talon on CAN Bus.
    fn from(talon_srx: &'a TalonSRX) -> PigeonIMU {
        let talon_device_id = talon_srx.get_device_id();
        let handle = unsafe { c_PigeonIMU_Create2(talon_device_id) };
        PigeonIMU { handle }
    }
}
