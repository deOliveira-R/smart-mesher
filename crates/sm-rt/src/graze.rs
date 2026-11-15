//! The one bound on grazing traps shared by every walk loop (trace, fire, delta flights, jumps).
//!
//! A trap is a run of more than [`ZERO_RUN_MAX`] consecutive carried steps of at most `tol`
//! (instances or cells handing the ray back and forth). The loop then nudges the ray on along its
//! direction ([`nudge`]), at most [`GRAZE_HOPS`] times; a trap that cannot be left ends in the typed
//! `LostCause::RecoveryFailed` error instead of running to `max_steps`.

/// Consecutive steps of at most `tol` before a run is treated as a grazing trap.
pub(crate) const ZERO_RUN_MAX: u32 = 8;
/// Nudge attempts (`tol * 2^k`, `k < GRAZE_TRIES`) out of one trap.
pub(crate) const GRAZE_TRIES: u32 = 6;
/// Traps one walk may leave this way before it ends in a typed error.
pub(crate) const GRAZE_HOPS: u32 = 4;

/// Per-walk state of the bound: the current run of tiny steps and the hops taken.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Guard {
    run: u32,
    hops: u32,
}

impl Guard {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Records a carried step of length `dist`; true when the run has reached a trap.
    pub(crate) fn trapped(&mut self, dist: f64, tol: f64) -> bool {
        self.run = if dist > tol { 0 } else { self.run + 1 };
        self.run > ZERO_RUN_MAX
    }

    /// Whether the walk may still hop out of a trap.
    pub(crate) fn may_hop(&self) -> bool {
        self.hops < GRAZE_HOPS
    }

    /// Records a successful hop (the run restarts).
    pub(crate) fn hopped(&mut self) {
        self.hops += 1;
        self.run = 0;
    }

    /// Hops taken so far (the `retries` of a typed error).
    pub(crate) fn hops(&self) -> u32 {
        self.hops
    }
}

/// Whether the room left is too short for even the first nudge (`tol`). A trap there is not a
/// failure: the walk keeps going (`max_steps` still bounds it) and ends at its limit as usual.
pub(crate) fn room_short(tol: f64, room: f64) -> bool {
    room.is_nan() || room < tol
}

/// The first nudge distance `tol * 2^k` (at most `room`, `k < GRAZE_TRIES`) for which `accept`
/// holds; `accept` moves the state when it takes the point.
pub(crate) fn nudge(tol: f64, room: f64, mut accept: impl FnMut(f64) -> bool) -> Option<f64> {
    for k in 0..GRAZE_TRIES {
        let nd = tol * f64::from(1u32 << k);
        if nd > room {
            break;
        }
        if accept(nd) {
            return Some(nd);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_run_of_tiny_steps_traps_after_the_bound_and_a_long_step_resets_it() {
        let mut g = Guard::new();
        for _ in 0..ZERO_RUN_MAX {
            assert!(!g.trapped(0.0, 1e-9));
        }
        assert!(g.trapped(1e-9, 1e-9), "step number ZERO_RUN_MAX + 1 of at most tol");
        g.hopped();
        for _ in 0..ZERO_RUN_MAX {
            assert!(!g.trapped(1e-12, 1e-9));
        }
        assert!(!g.trapped(2e-9, 1e-9), "a step longer than tol resets the run");
        for _ in 0..ZERO_RUN_MAX {
            assert!(!g.trapped(0.0, 1e-9));
        }
    }

    #[test]
    fn hops_are_capped() {
        let mut g = Guard::new();
        for k in 0..GRAZE_HOPS {
            assert!(g.may_hop(), "hop {k}");
            g.hopped();
        }
        assert!(!g.may_hop());
        assert_eq!(g.hops(), GRAZE_HOPS);
    }

    #[test]
    fn nudge_doubles_from_tol_and_respects_the_room() {
        let mut tried = Vec::new();
        let got = nudge(1e-9, f64::INFINITY, |nd| {
            tried.push(nd);
            tried.len() == 3
        });
        assert_eq!(got, Some(4e-9));
        assert_eq!(tried, [1e-9, 2e-9, 4e-9]);
        // Never further than the room, and never more than GRAZE_TRIES attempts.
        let mut n = 0;
        assert_eq!(
            nudge(1e-9, 3e-9, |_| {
                n += 1;
                false
            }),
            None
        );
        assert_eq!(n, 2);
        let mut n = 0;
        assert_eq!(
            nudge(1e-9, 1.0, |_| {
                n += 1;
                false
            }),
            None
        );
        assert_eq!(n, GRAZE_TRIES);
    }
}
