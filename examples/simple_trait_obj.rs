//! This is the `simple_robot` example, but passes a TalonSRX object
//! as a MotorController trait object to another function.
//!
//! This example only exists to test that MotorController can be type erased.
//! Please don't actually use this.

extern crate ctre;
use ctre::motor_control::{ControlMode, DemandType, LimitSwitchNormal, LimitSwitchSource};
use ctre::motor_control::{MotorController, TalonSRX};
use std::{thread, time};

// Seriously, please don't actually do this.
fn main() {
    let mut talon = TalonSRX::new(0);
    foo(&mut talon);
}

fn foo(mot: &mut MotorController) {
    let delay = time::Duration::from_millis(20);
    loop {
        mot.set(ControlMode::PercentOutput, 0.5, DemandType::Neutral, 0.0);
        thread::sleep(delay);
        mot.neutral_output();
        thread::sleep(delay);
    }
}
