extern crate ctre;
use ctre::motor_control::*;
use std::{thread, time};

/// Don't actually do this.  Ideally you would use a WPILib port with a nice Robot abstraction.
fn main() {
    let talon = TalonSRX::new(0);
    let ten_ms = time::Duration::from_millis(10);
    loop {
        talon.set(ControlMode::PercentOutput, 0.5, DemandType::Neutral, 0.0);
        thread::sleep(ten_ms);
        talon.neutral_output();
        thread::sleep(ten_ms);
    }
}
