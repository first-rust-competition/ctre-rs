//! Various enums and structs in CTRE Phoenix CCI and API.
//!
//! Anything with serde support or that are required for configuration will go here.
//!
//! This crate has a number of optional features, none of which are enabled by default:
//!
//! - `num` - Implements `FromPrimitive` for various numeric enums.
//! - `serde` - Provides serde support for all configuration enums and structs.
//! - `serde-flatten` - Flattens motor controller configuration structs
//!   when serialising.  Implies `serde`.

#![no_std]

#[cfg(feature = "num")]
#[macro_use]
extern crate num_derive;

#[macro_use]
extern crate smart_default;

#[macro_use]
mod macros;

pub mod canifier;
pub mod mot;
pub mod motion;
pub mod pigeon;

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CustomParamConfiguration(pub i32, pub i32);
