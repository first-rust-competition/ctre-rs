//! Enums and structs for the Pigeon IMU defined by the Phoenix API.

use core::fmt;

/// Data object for holding fusion information.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FusionStatus {
    pub is_fusing: bool, // int
    pub is_valid: bool,  // int
    pub heading: f64,
    /// Same as getLastError()
    pub last_error: i32,
}
impl fmt::Display for FusionStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(if self.last_error != 0 {
            "Could not receive status frame.  Check wiring and web-config."
        } else if !self.is_valid {
            "Fused Heading is not valid."
        } else if !self.is_fusing {
            "Fused Heading is valid."
        } else {
            "Fused Heading is valid and is fusing compass."
        })
    }
}

#[repr(i32)]
/// Various calibration modes supported by Pigeon.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum CalibrationMode {
    BootTareGyroAccel = 0,
    Temperature = 1,
    Magnetometer12Pt = 2,
    Magnetometer360 = 3,
    Accelerometer = 5,
    Unknown = -1,
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
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum PigeonState {
    NoComm = 0,
    Initializing = 1,
    Ready = 2,
    UserCalibration = 3,
    Unknown = -1,
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
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    pub last_error: i32,
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
        f.write_str(
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
