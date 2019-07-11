//! Motion profiling structs and enums.

pub use ctre_data::motion::*;
use ctre_sys::buff_traj::*;

use super::ErrorCode;

#[derive(Debug)]
/// Stream of trajectory points for Talon/Victor motion profiling.
pub struct BuffTrajPointStream {
    pub(crate) handle: *mut Stream,
}

impl BuffTrajPointStream {
    pub fn new() -> Self {
        let handle = unsafe { c_BuffTrajPointStream_Create1() };
        Self { handle }
    }

    /// Clear all trajectory points.
    pub fn clear(&mut self) -> ErrorCode {
        unsafe { c_BuffTrajPointStream_Clear(self.handle) }
    }

    /// Write a single trajectory point into the buffer.
    pub fn write(&mut self, traj_pt: &TrajectoryPoint) -> ErrorCode {
        unsafe {
            c_BuffTrajPointStream_Write(
                self.handle,
                traj_pt.position,
                traj_pt.velocity,
                traj_pt.arb_feed_fwd,
                traj_pt.auxiliary_pos,
                traj_pt.auxiliary_vel,
                traj_pt.auxiliary_arb_feed_fwd,
                traj_pt.profile_slot_select_0 as u32,
                traj_pt.profile_slot_select_1 as u32,
                traj_pt.is_last_point,
                traj_pt.zero_pos,
                traj_pt.time_dur,
                traj_pt.use_aux_pid,
            )
        }
    }

    /// Writes an iterable of trajectory points into the buffer.
    pub fn write_all(
        &mut self,
        traj_pts: impl IntoIterator<Item = impl AsRef<TrajectoryPoint>>,
    ) -> ErrorCode {
        let mut rv = ErrorCode::OK;
        for traj_pt in traj_pts {
            let er = self.write(traj_pt.as_ref());
            // save first nonzero error code
            rv = rv.or(er);
        }
        rv
    }
}

impl Drop for BuffTrajPointStream {
    fn drop(&mut self) {
        unsafe { c_BuffTrajPointStream_Destroy(self.handle) };
    }
}
