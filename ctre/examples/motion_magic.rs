/*!
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 *
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */

extern crate ctre;
extern crate wpilib;

use ctre::motor_control::{prelude::*, FeedbackDevice, PIDLoop, PIDSlot, StatusFrameEnhanced};
use wpilib::ds;

/// Which PID slot to pull gains from.
pub const SLOT_IDX: PIDSlot = PIDSlot::S0;
/// Talon SRX/Victor SPX supports multiple (auxiliary) PID loops.
/// For now we just want the primary one.
pub const PID_LOOP_IDX: PIDLoop = PIDLoop::Primary;
/// Set to zero to skip waiting for confirmation.
/// Set to non-zero to wait and report to DS if action fails.
pub const TIMEOUT_MS: i32 = 10;

struct Joystick<'a> {
    ds: &'a ds::DriverStation<'a>,
    port: ds::JoystickPort,
}

impl Joystick<'_> {
    fn y(&self) -> f32 {
        self.ds
            .stick_axis(self.port, ds::JoystickAxis::new(1).unwrap())
            .unwrap_or(0.0)
    }

    fn raw_button(&self, button: u8) -> bool {
        self.ds.stick_button(self.port, button).unwrap_or(false)
    }
}

struct Robot<'a> {
    joy: Joystick<'a>,
    talon: TalonSRX,
    loops: u32,
    times_in_motion_magic: u32,
}

pub fn main() {
    let base = wpilib::RobotBase::new().expect("Could not initialise HAL");
    let ds = base.make_ds();
    let mut talon = TalonSRX::new(3);
    let joy = Joystick {
        ds: &ds,
        port: ds::JoystickPort::new(0).unwrap(),
    };

    // first choose the sensor
    talon.config_selected_feedback_sensor(
        FeedbackDevice::CTRE_MagEncoder_Relative,
        PID_LOOP_IDX,
        TIMEOUT_MS,
    );
    talon.set_sensor_phase(true);
    talon.set_inverted(false);

    // set relevant frame periods to be as fast as periodic rate
    talon.set_status_frame_period(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, TIMEOUT_MS);
    talon.set_status_frame_period(StatusFrameEnhanced::Status_10_Targets, 10, TIMEOUT_MS);

    // set the peak and nominal outputs
    talon.config_nominal_output_forward(0.0, TIMEOUT_MS);
    talon.config_nominal_output_reverse(0.0, TIMEOUT_MS);
    talon.config_peak_output_forward(1.0, TIMEOUT_MS);
    talon.config_peak_output_reverse(-1.0, TIMEOUT_MS);

    // set closed loop gains in slot0 - see documentation
    talon.select_profile_slot(SLOT_IDX, PID_LOOP_IDX);
    talon.config_kf(SLOT_IDX, 0.2, TIMEOUT_MS);
    talon.config_kp(SLOT_IDX, 0.2, TIMEOUT_MS);
    talon.config_ki(SLOT_IDX, 0.0, TIMEOUT_MS);
    talon.config_kd(SLOT_IDX, 0.0, TIMEOUT_MS);
    // set acceleration and vcruise velocity - see documentation
    talon.config_motion_cruise_velocity(15000, TIMEOUT_MS);
    talon.config_motion_acceleration(6000, TIMEOUT_MS);
    // zero the sensor
    talon.set_selected_sensor_position(0, PID_LOOP_IDX, TIMEOUT_MS);

    let mut robot = Robot {
        joy,
        talon,
        loops: 0,
        times_in_motion_magic: 0,
    };

    wpilib::start_timed(&mut robot, &ds)
}

impl wpilib::IterativeRobot for Robot<'_> {
    fn teleop_periodic(&mut self) {
        let left_y_stick = -f64::from(self.joy.y());
        // let motor_output = self.talon.motor_output_percent();

        if self.joy.raw_button(1) {
            // Motion Magic - 4096 ticks/rev * 10 Rotations in either direction
            let target_pos = left_y_stick * 4096.0 * 10.0;
            self.talon
                .set(ControlMode::MotionMagic, target_pos, Demand::Neutral);
        } else {
            self.talon
                .set(ControlMode::PercentOutput, left_y_stick, Demand::Neutral);
        }
    }
}
