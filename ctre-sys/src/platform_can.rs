//! Configures the can interface for API

use std::os::raw;

use super::ErrorCode;

extern "C" {
    /// Set the CAN interface to use, for example on Linux you may select "can0".
    pub fn c_SetCANInterface(canBusInterface: *const raw::c_char) -> i32;
    /// Destroys all items in interface
    pub fn c_DestroyAll() -> ErrorCode;
    /// Starts all items in interface
    pub fn c_StartAll() -> ErrorCode;
    // pub fn c_Autocache_SetAutocacheLevel(state: AutocacheState);
}
