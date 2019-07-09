//! Various enums and structs in CTRE Phoenix CCI and API.
//!
//! Anything with serde support or that are required for configuration will go here.

#![no_std]

#[cfg(feature = "num")]
#[macro_use]
extern crate num_derive;
#[cfg(feature = "num")]
extern crate num_traits;

#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;

#[macro_use]
extern crate smart_default;

#[macro_use]
mod macros;

pub mod canifier;
pub mod mot;
pub mod motion;
pub mod pigeon;

#[derive(Default, Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CustomParamConfiguration(pub i32, pub i32);
