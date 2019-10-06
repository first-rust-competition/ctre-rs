use std::ffi;

use super::ErrorCode;

#[repr(C)]
pub struct Stream {
    _private: [u8; 0],
}

extern "C" {
    pub fn c_BuffTrajPointStream_Create1() -> *mut Stream;
    pub fn c_BuffTrajPointStream_DestroyAll();
    pub fn c_BuffTrajPointStream_Destroy(handle: *mut Stream) -> ErrorCode;

    pub fn c_BuffTrajPointStream_Clear(handle: *mut Stream) -> ErrorCode;

    pub fn c_BuffTrajPointStream_Write(
        handle: *mut Stream,
        position: f64,
        velocity: f64,
        arbFeedFwd: f64,
        auxiliaryPos: f64,
        auxiliaryVel: f64,
        auxiliaryArbFeedFwd: f64,
        profileSlotSelect0: u32,
        profileSlotSelect1: u32,
        isLastPoint: bool,
        zeroPos: bool,
        timeDur: u32,
        useAuxPID: bool,
    ) -> ErrorCode;

    pub fn c_BuffTrajPointStream_Lookup(
        handle: *mut Stream,
        outObject: *mut *mut ffi::c_void,
    ) -> ErrorCode;
}
