//! Functions for simulation as a platform

use std::os::raw;

#[repr(C)]
pub enum DeviceType {
    TalonSRXType,
    VictorSPXType,
    CANifierType,
    PigeonIMUType,
}

extern "C" {
    /// Create Device
    pub fn c_SimCreate(type_: DeviceType, id: raw::c_int) -> i32;
    /// Destroy Device
    pub fn c_SimDestroy(type_: DeviceType, id: raw::c_int) -> i32;
    /// Destroy all devices in sim
    pub fn c_SimDestroyAll() -> i32;

    /// Get configs of simulated device
    pub fn c_SimConfigGet(
        type_: DeviceType,
        param: u32,
        valueToSend: u32,
        outValueReceived: &mut u32,
        outSubvalue: &mut u32,
        ordinal: u32,
        id: raw::c_int,
    ) -> i32;

    /// Sets configs of simulated device
    pub fn c_SimConfigSet(
        type_: DeviceType,
        param: u32,
        value: u32,
        subValue: u32,
        ordinal: u32,
        id: raw::c_int,
    ) -> i32;
}
