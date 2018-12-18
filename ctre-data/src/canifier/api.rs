//! Structs for the CANifier defined by the Phoenix API.

#[allow(non_snake_case)]
/// Structure to hold the pin values.
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PinValues {
    pub QUAD_IDX: bool,
    pub QUAD_B: bool,
    pub QUAD_A: bool,
    pub LIMR: bool,
    pub LIMF: bool,
    pub SDA: bool,
    pub SCL: bool,
    pub SPI_CS_PWM3: bool,
    pub SPI_MISO_PWM2: bool,
    pub SPI_MOSI_PWM1: bool,
    pub SPI_CLK_PWM0: bool,
}
