//! This is the `simple_robot` example, but passes a TalonSRX object
//! as a MotorController trait object to another function.
//!
//! This example only exists to test that MotorController can be type erased.
//! Please don't actually use this.

extern crate ctre;
use ctre::motor_control::{ControlMode, Demand, MotorController, TalonSRX};
use std::{thread, time};

// Seriously, please don't actually do this.
fn main() {
    let mut talon = TalonSRX::new(0);
    foo(&mut talon);
}

fn foo(mot: &mut dyn MotorController) {
    let delay = time::Duration::from_millis(20);
    loop {
        mot.set(ControlMode::PercentOutput, 0.5, Demand::Neutral);
        thread::sleep(delay);
        mot.neutral_output();
        thread::sleep(delay);
    }
}
