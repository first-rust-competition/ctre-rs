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

use ctre::motor_control::*;
use std::time::Duration;
use wpilib::ds::RobotState;

/// Which PID slot to pull gains from.
pub const SLOT_IDX: PIDSlot = PIDSlot::S0;
/// Talon SRX/Victor SPX supports multiple (auxiliary) PID loops.
/// For now we just want the primary one.
pub const PID_LOOP_IDX: PIDLoop = PIDLoop::Primary;
/// Set to zero to skip waiting for confirmation.
/// Set to non-zero to wait and report to DS if action fails.
pub const TIMEOUT_MS: i32 = 10;

pub fn main() {
    let robot = wpilib::RobotBase::new().expect("Could not initialise HAL");
    let ds = robot.get_ds_instance();
    let talon = TalonSRX::new(3);
    let joy = wpilib::Joystick::new(&robot, 0);

    // let mut loops = 0;
    // let mut timesInMotionMagic = 0;

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
    talon.config_nominal_output_forward(0, TIMEOUT_MS);
    talon.config_nominal_output_reverse(0, TIMEOUT_MS);
    talon.config_peak_output_forward(1, TIMEOUT_MS);
    talon.config_peak_output_reverse(-1, TIMEOUT_MS);

    // set closed loop gains in slot0 - see documentation
    talon.select_profile_slot(SLOT_IDX, PID_LOOP_IDX);
    talon.config_kf(0, 0.2, TIMEOUT_MS);
    talon.config_kp(0, 0.2, TIMEOUT_MS);
    talon.config_ki(0, 0, TIMEOUT_MS);
    talon.config_kd(0, 0, TIMEOUT_MS);
    // set acceleration and vcruise velocity - see documentation
    talon.config_motion_cruise_velocity(15000, TIMEOUT_MS);
    talon.config_motion_acceleration(6000, TIMEOUT_MS);
    // zero the sensor
    talon.set_selected_sensor_position(0, PID_LOOP_IDX, TIMEOUT_MS);

    loop {
        ds.wait_for_data(Duration::from_millis(20));
        match ds.get_state() {
            RobotState::Teleop => {
                let left_y_stick = -1.0 * joy.get_y();
                let motor_output = talon.get_motor_output_percent();

                if joy.get_raw_button(1) {
                    // Motion Magic - 4096 ticks/rev * 10 Rotations in either direction
                    let target_pos = left_y_stick * 4096 * 10.0;
                    talon.set(ControlMode::MotionMagic, target_pos);
                } else {
                    talon.set(ControlMode::PercentOutput, left_y_stick);
                }
            }
            _ => {}
        }
    }
}
