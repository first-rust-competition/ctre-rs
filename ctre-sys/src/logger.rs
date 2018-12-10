use std::os::raw;

use super::ErrorCode;

extern "C" {
    pub fn c_Logger_Close();
    pub fn c_Logger_Open(language: raw::c_int, logDriverStation: bool);
    pub fn c_Logger_Log(
        code: ErrorCode,
        origin: *const raw::c_char,
        hierarchy: raw::c_int,
        stacktrace: *const raw::c_char,
    ) -> ErrorCode;
}
