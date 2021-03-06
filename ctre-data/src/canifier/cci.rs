//! CANifier enums defined in the Phoenix CCI headers.
#![allow(non_camel_case_types)]

enum_defaults! {
    /// Returns the default measurement period of 100 ms.
    CANifierVelocityMeasPeriod::Period_100Ms;
}

// This would say this was autogenerated by rust-bindgen, but that'd be a lie.

enum_number! {
    #[repr(i32)]
    #[cfg_attr(feature = "num", derive(FromPrimitive))]
    pub enum CANifierVelocityMeasPeriod {
        Period_1Ms = 1,
        Period_2Ms = 2,
        Period_5Ms = 5,
        Period_10Ms = 10,
        Period_20Ms = 20,
        Period_25Ms = 25,
        Period_50Ms = 50,
        Period_100Ms = 100,
    }
}
