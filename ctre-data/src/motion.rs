//! Motion profiling enums and structs defined in the Phoenix CCI headers.

use super::mot::api::PIDSlot;

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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SetValueMotionProfile {
    Invalid = -1,
    Disable = 0,
    Enable = 1,
    Hold = 2,
}

/// Motion Profile Trajectory Point
/// This is simply a data transfer object.
#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct TrajectoryPoint {
    /// The position to servo to (in sensor units).
    pub position: f64,
    /// The velocity to feed-forward (in sensor units per 100ms).
    pub velocity: f64,
    /// Added to the output of PID[0], should be within [-1,+1] where 0.01 = 1%.
    pub arb_feed_fwd: f64,
    /// The position for auxiliary PID[1] to target (in sensor units).
    pub auxiliary_pos: f64,
    /// The velocity for auxiliary PID[1] to target (in sensor units per 100ms).
    pub auxiliary_vel: f64,
    /// Added to the output of PID[1], should be within [-1,+1] where 0.01 = 1%.
    pub auxiliary_arb_feed_fwd: f64,
    /// Which slot to get PIDF gains.
    /// PID is used for position servo.
    /// F is used as the Kv constant for velocity feed-forward.
    /// Typically this is hard-coded
    /// to a particular slot, but you are free to gain schedule if need be.
    /// gain schedule if need be.
    pub profile_slot_select_0: PIDSlot,
    /// Which slot to get PIDF gains for auxiliary PID.
    /// This only has impact during MotionProfileArc Control mode.
    pub profile_slot_select_1: PIDSlot,
    /// Set to true to signal Talon that this is the final point, so do not
    /// attempt to pop another trajectory point from out of the Talon buffer.
    /// Instead continue processing this way point.  Typically the velocity
    /// member variable should be zero so that the motor doesn't spin indefinitely.
    pub is_last_point: bool,
    /**
     * Set to true to signal Talon to zero the selected sensor.
     * When generating MPs, one simple method is to make the first target position zero,
     * and the final target position the target distance from the current position.
     * Then when you fire the MP, the current position gets set to zero.
     * If this is the intent, you can set `zero_pos` on the first trajectory point.
     *
     * Otherwise you can leave this false for all points, and offset the positions
     * of all trajectory points so they are correct.
     *
     * If using multiple sensor sources (Arc modes) we recommend you manually set sensor positions
     * before arming MP.
     */
    pub zero_pos: bool,
    /// Duration (ms) to apply this trajectory pt.
    /// This time unit is ADDED to the existing base time set by
    /// ConfigMotionProfileTrajectoryPeriod().
    pub time_dur: u32,
    /// If using MotionProfileArc, this flag must be true on all points.
    /// If using MotionProfile, this flag must be false on all points.
    pub use_aux_pid: bool,
}

// NOTE: MotionProfileStatus uses c_int upstream.
// Assuming that c_int is i32 on the relevant platforms.

/// Motion Profile Status
/// This is simply a data transer object.
#[derive(Debug, PartialEq, Eq, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MotionProfileStatus {
    /// The available empty slots in the trajectory buffer.
    ///
    /// The robot API holds a "top buffer" of trajectory points, so your applicaion
    /// can dump several points at once.  The API will then stream them into the Talon's
    /// low-level buffer, allowing the Talon to act on them.
    pub top_buffer_rem: usize,
    /// The number of points in the top trajectory buffer.
    pub top_buffer_cnt: usize,
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
    /// The selected PID[0] profile slot of current profile
    pub profile_slot_select_0: i32,
    /// The selected auxiliary PID[1] profile slot of current profile
    pub profile_slot_select_1: i32,
    /// The current output mode of the motion profile executer (disabled, enabled, or hold).
    /// When changing the set() value in MP mode, it's important to check this signal to
    /// confirm the change takes effect before interacting with the top buffer.
    pub output_enable: SetValueMotionProfile,
    /// The applied duration of the active trajectory point
    pub time_dur_ms: i32,
}
