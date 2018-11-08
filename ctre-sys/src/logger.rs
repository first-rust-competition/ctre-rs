use std::os::raw::{c_char, c_int};

use super::ErrorCode;

extern "C" {
    pub fn c_Logger_Close();
    pub fn c_Logger_Open(language: c_int, logDriverStation: bool);
    pub fn c_Logger_Log(
        code: ErrorCode,
        origin: *const c_char,
        hierarchy: c_int,
        stacktrace: *const c_char,
    ) -> ErrorCode;
}
