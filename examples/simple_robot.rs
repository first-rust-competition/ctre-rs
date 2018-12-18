//! Don't actually do this.  Ideally you would use a WPILib port with a nice Robot abstraction.

extern crate ctre;
use ctre::motor_control::{ControlMode, DemandType, LimitSwitchNormal, LimitSwitchSource};
use ctre::motor_control::{MotorController, TalonSRX};
use std::{thread, time};

fn main() -> ctre::Result<()> {
    let mut talon = TalonSRX::new(0);
    let delay = time::Duration::from_millis(20);
    talon
        .config_forward_limit_switch_source(
            LimitSwitchSource::FeedbackConnector,
            LimitSwitchNormal::NormallyOpen,
            10,
        ).into_res()?;
    loop {
        talon.set(ControlMode::PercentOutput, 0.5, DemandType::Neutral, 0.0);
        thread::sleep(delay);
        talon.neutral_output();
        thread::sleep(delay);
    }
}
