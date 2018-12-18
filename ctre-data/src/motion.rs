//! Motion profiling enums and structs defined in the Phoenix CCI headers.

use super::mot::api::PIDSlot;

defaults! {
    /// Returns a default duration of 100 ms (not recommended).
    TrajectoryDuration => TrajectoryDuration::T100ms,
}

impl From<i32> for SetValueMotionProfile {
    fn from(value: i32) -> SetValueMotionProfile {
        match value {
            0 => SetValueMotionProfile::Disable,
            1 => SetValueMotionProfile::Enable,
            2 => SetValueMotionProfile::Hold,
            _ => SetValueMotionProfile::Invalid,
        }
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SetValueMotionProfile {
    Invalid = -1,
    Disable = 0,
    Enable = 1,
    Hold = 2,
}

enum_number! {
    #[repr(i32)]
    /// Duration to apply to a particular trajectory pt.
    /// This time unit is ADDED to the existing base time set by
    /// ConfigMotionProfileTrajectoryPeriod().
    pub enum TrajectoryDuration {
        T0ms = 0,
        T5ms = 5,
        T10ms = 10,
        T20ms = 20,
        T30ms = 30,
        T40ms = 40,
        T50ms = 50,
        T100ms = 100,
    }
}

/// Motion Profile Trajectory Point
/// This is simply a data transfer object.
#[repr(C)]
#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct TrajectoryPoint {
    /// The position to servo to.
    pub position: f64,
    /// The velocity to feed-forward.
    pub velocity: f64,
    /// Not used.  Use auxiliary_pos instead.
    heading_deg: f64,
    /// The position for auxiliary PID to target.
    pub auxiliary_pos: f64,
    /// Which slot to get PIDF gains.
    /// PID is used for position servo.
    /// F is used as the Kv constant for velocity feed-forward.
    /// Typically this is hard-coded
    /// to a particular slot, but you are free to gain schedule if need be.
    /// gain schedule if need be.
    /// Choose from [0,3].
    pub profile_slot_select_0: PIDSlot,
    /// Which slot to get PIDF gains for auxiliary PID.
    /// This only has impact during MotionProfileArc Control mode.
    /// Choose from [0,1].
    pub profile_slot_select_1: PIDSlot,
    /// Set to true to signal Talon that this is the final point, so do not
    /// attempt to pop another trajectory point from out of the Talon buffer.
    /// Instead continue processing this way point.  Typically the velocity
    /// member variable should be zero so that the motor doesn't spin indefinitely.
    pub is_last_point: bool,
    /// Set to true to signal Talon to zero the selected sensor.
    /// When generating MPs, one simple method is to make the first target position zero,
    /// and the final target position the target distance from the current position.
    /// Then when you fire the MP, the current position gets set to zero.
    /// If this is the intent, you can set zero_pos on the first trajectory point.
    ///
    /// Otherwise you can leave this false for all points, and offset the positions
    /// of all trajectory points so they are correct.
    pub zero_pos: bool,
    /// Duration to apply this trajectory pt.
    /// This time unit is ADDED to the existing base time set by
    /// ConfigMotionProfileTrajectoryPeriod().
    pub time_dur: TrajectoryDuration,
}
#[test]
fn bindgen_test_layout_TrajectoryPoint() {
    assert_eq!(
        ::core::mem::size_of::<TrajectoryPoint>(),
        48usize,
        concat!("Size of: ", stringify!(TrajectoryPoint))
    );
    assert_eq!(
        ::core::mem::align_of::<TrajectoryPoint>(),
        8usize,
        concat!("Alignment of ", stringify!(TrajectoryPoint))
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).position as *const _ as usize },
        0usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(position)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).velocity as *const _ as usize },
        8usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(velocity)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).heading_deg as *const _ as usize },
        16usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(heading_deg)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).auxiliary_pos as *const _ as usize },
        24usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(auxiliary_pos)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<TrajectoryPoint>())).profile_slot_select_0 as *const _ as usize
        },
        32usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(profile_slot_select_0)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<TrajectoryPoint>())).profile_slot_select_1 as *const _ as usize
        },
        36usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(profile_slot_select1)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).is_last_point as *const _ as usize },
        40usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(is_last_point)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).zero_pos as *const _ as usize },
        41usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(zero_pos)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<TrajectoryPoint>())).time_dur as *const _ as usize },
        44usize,
        concat!(
            "Offset of field: ",
            stringify!(TrajectoryPoint),
            "::",
            stringify!(time_dur)
        )
    );
}

// NOTE: MotionProfileStatus uses c_int upstream.
// Assuming that c_int is i32 on the relevant platforms.

/// Motion Profile Status
/// This is simply a data transer object.
#[repr(C)]
#[derive(Debug, PartialEq, Eq, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MotionProfileStatus {
    /// The available empty slots in the trajectory buffer.
    ///
    /// The robot API holds a "top buffer" of trajectory points, so your applicaion
    /// can dump several points at once.  The API will then stream them into the Talon's
    /// low-level buffer, allowing the Talon to act on them.
    pub top_buffer_rem: i32,
    /// The number of points in the top trajectory buffer.
    pub top_buffer_cnt: i32,
    /// The number of points in the low level Talon buffer.
    pub btm_buffer_cnt: i32,
    /// Set if `is_underrun` ever gets set.
    /// Only is cleared by clearMotionProfileHasUnderrun() to ensure
    /// robot logic can react or instrument it.
    /// @see clearMotionProfileHasUnderrun()
    pub has_underrun: bool,
    /// This is set if Talon needs to shift a point from its buffer into
    /// the active trajectory point however the buffer is empty. This gets cleared
    /// automatically when is resolved.
    pub is_underrun: bool,
    /// True if the active trajectory point has not empty, false otherwise.
    /// The members in activePoint are only valid if this signal is set.
    pub active_point_valid: bool,
    pub is_last: bool,
    /// Selected slot for PID Loop 0
    pub profile_slot_select_0: i32,
    /// Selected slot for PID Loop 0
    pub profile_slot_select_1: i32,
    /// The current output mode of the motion profile executer (disabled, enabled, or hold).
    /// When changing the set() value in MP mode, it's important to check this signal to
    /// confirm the change takes effect before interacting with the top buffer.
    pub output_enable: SetValueMotionProfile,
    /// The applied duration of the active trajectory point
    pub time_dur_ms: i32,
}
#[test]
fn bindgen_test_layout_MotionProfileStatus() {
    assert_eq!(
        ::core::mem::size_of::<MotionProfileStatus>(),
        32usize,
        concat!("Size of: ", stringify!(MotionProfileStatus))
    );
    assert_eq!(
        ::core::mem::align_of::<MotionProfileStatus>(),
        4usize,
        concat!("Alignment of ", stringify!(MotionProfileStatus))
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).top_buffer_rem as *const _ as usize
        },
        0usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(top_buffer_rem)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).top_buffer_cnt as *const _ as usize
        },
        4usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(top_buffer_cnt)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).btm_buffer_cnt as *const _ as usize
        },
        8usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(btm_buffer_cnt)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).has_underrun as *const _ as usize
        },
        12usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(has_underrun)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).is_underrun as *const _ as usize
        },
        13usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(is_underrun)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).active_point_valid as *const _ as usize
        },
        14usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(active_point_valid)
        )
    );
    assert_eq!(
        unsafe { &(*(::core::ptr::null::<MotionProfileStatus>())).is_last as *const _ as usize },
        15usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(is_last)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).profile_slot_select_0 as *const _
                as usize
        },
        16usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(profile_slot_select_0)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).profile_slot_select_1 as *const _
                as usize
        },
        20usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(profile_slot_select_1)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).output_enable as *const _ as usize
        },
        24usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(output_enable)
        )
    );
    assert_eq!(
        unsafe {
            &(*(::core::ptr::null::<MotionProfileStatus>())).time_dur_ms as *const _ as usize
        },
        28usize,
        concat!(
            "Offset of field: ",
            stringify!(MotionProfileStatus),
            "::",
            stringify!(time_dur_ms)
        )
    );
}
