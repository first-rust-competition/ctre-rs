use std::os::raw;

use super::ErrorCode;

extern "C" {
    pub fn c_SetCANInterface(canBusInterface: *const raw::c_char) -> i32;
    pub fn c_DestroyAll() -> ErrorCode;
    pub fn c_StartAll() -> ErrorCode;
    // pub fn c_Autocache_SetAutocacheLevel(state: AutocacheState);
}
