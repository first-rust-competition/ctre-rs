//! Pigeon IMU

pub use ctre_data::pigeon::*;
use ctre_sys::pigeon::*;
pub use ctre_sys::pigeon::{PigeonIMU_ControlFrame, PigeonIMU_StatusFrame};
use std::mem;
#[cfg(feature = "usage-reporting")]
use wpilib_sys::usage::report_usage;

use super::super::{
    motor_control::{MotorController, TalonSRX},
    CustomParam, CustomParamConfiguration, ErrorCode, ParamEnum, Result,
};

pub type ControlFrame = PigeonIMU_ControlFrame;
pub type StatusFrame = PigeonIMU_StatusFrame;

#[derive(Debug, Copy, Clone)]
pub struct Faults(i32);
impl Faults {
    pub fn has_any_fault(self) -> bool {
        self.0 != 0
    }
}
impl_binary_fmt!(Faults);
#[derive(Debug, Copy, Clone)]
pub struct StickyFaults(i32);
impl StickyFaults {
    pub fn has_any_fault(self) -> bool {
        self.0 != 0
    }
}
impl_binary_fmt!(StickyFaults);

/**
 * Pigeon IMU Class.
 * Class supports communicating over CANbus and over ribbon-cable (CAN Talon SRX).
 */
#[derive(Debug)]
pub struct PigeonIMU {
    handle: Handle,
}

impl PigeonIMU {
    /// Create a Pigeon object that communicates with Pigeon on CAN Bus.
    /// * `device_number` - CAN Device Id of Pigeon [0,62]
    pub fn new(device_number: i32) -> PigeonIMU {
        let handle = unsafe { c_PigeonIMU_Create1(device_number) };
        // kResourceType_PigeonIMU
        #[cfg(feature = "usage-reporting")]
        report_usage(61, device_number as u32 + 1);
        PigeonIMU { handle }
    }

    /// Create a Pigeon object that communicates with Pigeon through the
    /// Gadgeteer ribbon cable connected to a Talon on CAN Bus.
    pub fn with_talon_srx(talon_srx: &TalonSRX) -> PigeonIMU {
        let talon_device_id = talon_srx.get_device_id();
        let handle = unsafe { c_PigeonIMU_Create2(talon_device_id) };
        #[cfg(feature = "usage-reporting")]
        {
            let instance_number = talon_device_id as u32 + 1;
            // kResourceType_PigeonIMU
            report_usage(61, instance_number);
            // kResourceType_CTRE_future0
            report_usage(64, instance_number); // record as Pigeon-via-Uart
        }
        PigeonIMU { handle }
    }

    /**
     * Sets the Yaw register to the specified value.
     *
     * * `angle` - Degree of Yaw [+/- 23040 degrees]
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn set_yaw(&self, angle: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYaw(self.handle, angle, timeout_ms) }
    }
    /// Atomically add to the Yaw register.
    pub fn add_yaw(&self, angle: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddYaw(self.handle, angle, timeout_ms) }
    }
    /// Sets the Yaw register to match the current compass value.
    pub fn set_yaw_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetYawToCompass(self.handle, timeout_ms) }
    }

    /**
     * Sets the Fused Heading to the specified value.
     *
     * * `angle` - Degree of heading [+/- 23040 degrees]
     * * `timeout_ms` - Timeout value in ms.
     *   If nonzero, function will wait for config success and report an error if it times out.
     *   If zero, no blocking or checking is performed.
     */
    pub fn set_fused_heading(&self, angle: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeading(self.handle, angle, timeout_ms) }
    }
    /// Atomically add to the Fused Heading register.
    pub fn add_fused_heading(&self, angle: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_AddFusedHeading(self.handle, angle, timeout_ms) }
    }
    /// Sets the Fused Heading register to match the current compass value.
    pub fn set_fused_heading_to_compass(&self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetFusedHeadingToCompass(self.handle, timeout_ms) }
    }
    pub fn set_accum_z_angle(&self, angle: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetAccumZAngle(self.handle, angle, timeout_ms) }
    }

    /// Enable/Disable Temp compensation. Pigeon defaults with this on at boot.
    #[deprecated(
        since = "0.7.0",
        note = "Use `set_temperature_compensation_disable` instead"
    )]
    pub fn config_temperature_compensation_enable(
        &mut self,
        enable: bool,
        timeout_ms: i32,
    ) -> ErrorCode {
        self.set_temperature_compensation_disable(!enable, timeout_ms)
    }
    /// Disable/Enable Temp compensation. Pigeon has this on/False at boot.
    pub fn set_temperature_compensation_disable(
        &mut self,
        disable: bool,
        timeout_ms: i32,
    ) -> ErrorCode {
        unimplemented!()
    }

    /// Set the declination for compass. Declination is the difference between
    /// Earth Magnetic north, and the geographic "True North".
    pub fn set_compass_declination(&mut self, angle_deg_offset: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassDeclination(self.handle, angle_deg_offset, timeout_ms) }
    }
    /// Sets the compass angle. Although compass is absolute [0,360) degrees, the
    /// continuous compass register holds the wrap-arounds.
    pub fn set_compass_angle(&mut self, angle_deg: f64, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetCompassAngle(self.handle, angle_deg, timeout_ms) }
    }

    pub fn enter_calibration_mode(&self, cal_mode: CalibrationMode, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_EnterCalibrationMode(self.handle, cal_mode as _, timeout_ms) }
    }

    /// Get the status of the current (or previousley complete) calibration.
    pub fn general_status(&self) -> Result<GeneralStatus> {
        let mut status: GeneralStatus = unsafe { mem::uninitialized() };
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

    pub fn last_error(&self) -> ErrorCode {
        unsafe { c_PigeonIMU_GetLastError(self.handle) }
    }

    /// Get 6d Quaternion data.
    /// Returns an array of the wxyz quaternion data.
    pub fn get_6d_quaternion(&self) -> Result<[f64; 4]> {
        cci_get_call!(c_PigeonIMU_Get6dQuaternion(self.handle, _: [f64; 4]))
    }
    /// Get Yaw, Pitch, and Roll data.
    /// Returns an array with yaw, pitch, and roll, in that order.
    pub fn yaw_pitch_roll(&self) -> Result<[f64; 3]> {
        cci_get_call!(c_PigeonIMU_GetYawPitchRoll(self.handle, _: [f64; 3]))
    }
    /**
     * Get AccumGyro data.
     * AccumGyro is the integrated gyro value on each axis.
     *
     * Returns an array `xyz_deg`.
     */
    pub fn accum_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call!(c_PigeonIMU_GetAccumGyro(self.handle, _: [f64; 3]))
    }
    /// Get the absolute compass heading, in the interval [0, 360) degrees.
    pub fn absolute_compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetAbsoluteCompassHeading(self.handle, _: f64))
    }
    /// Get the continuous compass heading, in the interval [-23040, 23040) degrees.
    pub fn compass_heading(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassHeading(self.handle, _: f64))
    }
    /// Get the compass' measured magnetic field strength in microteslas (uT).
    pub fn compass_field_strength(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetCompassFieldStrength(self.handle, _: f64))
    }
    /// Get the temperature of the pigeon, in degrees Celsius.
    pub fn temp(&self) -> Result<f64> {
        cci_get_call!(c_PigeonIMU_GetTemp(self.handle, _: f64))
    }
    pub fn state(&self) -> Result<PigeonState> {
        Ok(cci_get_call!(c_PigeonIMU_GetState(self.handle, _: i32))?.into())
    }
    pub fn uptime(&self) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetUpTime(self.handle, _: i32))
    }

    /// Get Raw Magnetometer data.
    /// Returns an array `rm_xyz`.  Number is equal to 0.6 microteslas per unit.
    pub fn raw_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call!(c_PigeonIMU_GetRawMagnetometer(self.handle, _: [i16; 3]))
    }
    /// Get Biased Magnetometer data.
    /// Returns an array `bm_xyz`.  Number is equal to 0.6 microteslas per unit.
    pub fn biased_magnetometer(&self) -> Result<[i16; 3]> {
        cci_get_call!(c_PigeonIMU_GetBiasedMagnetometer(self.handle, _: [i16; 3]))
    }
    /// Get Biased Accelerometer data.
    /// Returns an array `ba_xyz`.  These are in fixed point notation Q2.14.  eg. 16384 = 1G
    pub fn biased_accelerometer(&self) -> Result<[i16; 3]> {
        cci_get_call!(c_PigeonIMU_GetBiasedAccelerometer(self.handle, _: [i16; 3]))
    }
    /// Get Raw Gyro data.
    /// Returns an array `xyz_dps`, with data in degrees per second.
    pub fn raw_gyro(&self) -> Result<[f64; 3]> {
        cci_get_call!(c_PigeonIMU_GetRawGyro(self.handle, _: [f64; 3]))
    }
    /// Get Accelerometer tilt angles.
    /// Returns a 3-array of x, y, z angles in degrees.
    pub fn accelerometer_angles(&self) -> Result<[f64; 3]> {
        cci_get_call!(c_PigeonIMU_GetAccelerometerAngles(self.handle, _: [f64; 3]))
    }

    /// Get the current Fusion Status (including fused heading)
    pub fn fusion_status(&self) -> Result<FusionStatus> {
        let mut b_is_fusing = 0;
        let mut b_is_valid = 0;
        let mut heading = 0.0;
        let mut last_error = 0;
        let err = unsafe {
            c_PigeonIMU_GetFusedHeading2(
                self.handle,
                &mut b_is_fusing,
                &mut b_is_valid,
                &mut heading,
                &mut last_error,
            )
        };
        match err {
            ErrorCode::OK => Ok(FusionStatus {
                is_fusing: b_is_fusing != 0,
                is_valid: b_is_valid != 0,
                heading,
                last_error,
            }),
            _ => Err(err),
        }
    }
    /// Gets the Fused Heading in degrees.
    pub fn fused_heading(&self) -> Result<f64> {
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
    pub fn firmware_version(&self) -> Result<i32> {
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
        &mut self,
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
        cci_get_call!(
            c_PigeonIMU_ConfigGetCustomParam(self.handle, _: i32, param_index, timout_ms)
        )
    }
    /**
     * Sets a parameter. Generally this is not used.
     * This can be utilized in
     * - Using new features without updating API installation.
     * - Errata workarounds to circumvent API implementation.
     * - Allows for rapid testing / unit testing of firmware.
     */
    pub fn config_set_parameter(
        &mut self,
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
        cci_get_call!(
            c_PigeonIMU_ConfigGetParameter(self.handle, param as _, _: f64, ordinal, timeout_ms)
        )
    }

    pub fn faults(&self) -> Result<Faults> {
        Ok(Faults(
            cci_get_call!(c_PigeonIMU_GetFaults(self.handle, _: i32))?,
        ))
    }
    pub fn sticky_faults(&self) -> Result<StickyFaults> {
        Ok(StickyFaults(
            cci_get_call!(c_PigeonIMU_GetStickyFaults(self.handle, _: i32))?,
        ))
    }
    pub fn clear_sticky_faults(&mut self, timeout_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_ClearStickyFaults(self.handle, timeout_ms) }
    }

    pub fn set_status_frame_period(
        &mut self,
        frame: StatusFrame,
        period_ms: i32,
        timeout_ms: i32,
    ) -> ErrorCode {
        unsafe { c_PigeonIMU_SetStatusFramePeriod(self.handle, frame as _, period_ms, timeout_ms) }
    }
    pub fn get_status_frame_period(&self, frame: StatusFrame, timeout_ms: i32) -> Result<i32> {
        cci_get_call!(c_PigeonIMU_GetStatusFramePeriod(self.handle, frame as _, _: i32, timeout_ms))
    }
    pub fn set_control_frame_period(&mut self, frame: ControlFrame, period_ms: i32) -> ErrorCode {
        unsafe { c_PigeonIMU_SetControlFramePeriod(self.handle, frame as _, period_ms) }
    }
}

#[deprecated(since = "0.7.0", note = "Use `PigeonIMU::with_talon_srx` instead.")]
impl<'a> From<&'a TalonSRX> for PigeonIMU {
    /// Soft-deprecated. Use [`PigeonIMU::with_talon_srx`] instead.
    ///
    /// [`PigeonIMU::with_talon_srx`]: #method.with_talon_srx
    fn from(talon_srx: &'a TalonSRX) -> PigeonIMU {
        PigeonIMU::with_talon_srx(talon_srx)
    }
}
