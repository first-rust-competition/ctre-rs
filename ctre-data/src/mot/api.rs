//! Serializable enums for CTRE motor controllers defined for the API level.

// These enums are defined by ctre-rs to convey information with type safety.
// These are required for configuration or motion profiling.

enum_defaults! {
    /// Returns the factory default of standard polarity.
    AuxPIDPolarity::Standard;
    /// Returns a default of slot 0.
    PIDSlot::S0;
}

/**
 * The polarity of the auxiliary PID loop (PID1).
 *
 * Standard Polarity:
 *  - Primary Output = PID0 + PID1
 *  - Auxiliary Output = PID0 - PID1
 *
 * Inverted Polarity:
 *  - Primary Output = PID0 - PID1
 *  - Auxiliary Output = PID0 + PID1
 *
 * See `MotorController::config_aux_pid_polarity`.
 */
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum AuxPIDPolarity {
    Standard = 0,
    Inverted = 1,
}

enum_number! {
    /// PIDF profile slots.
    #[cfg_attr(feature = "num", derive(FromPrimitive))]
    #[repr(i32)]
    pub enum PIDSlot {
        S0 = 0,
        S1 = 1,
        S2 = 2,
        S3 = 3,
    }
}

unsafe impl bytemuck::Contiguous for PIDSlot {
    type Int = i32;
    const MIN_VALUE: Self::Int = 0;
    const MAX_VALUE: Self::Int = 3;
}
