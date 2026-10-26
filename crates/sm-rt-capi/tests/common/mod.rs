//! Shared helpers for the integration tests: stub handles and a seeded RNG.
#![allow(dead_code)]

use sm_rt_capi::ffi::test_hooks::{STUB_SPHERE, tracker_for};
use sm_rt_capi::ffi::*;
use sm_rt_capi::{SmrtCtx, SmrtState, SmrtStep, SmrtTracker, tracker_into_raw};

/// splitmix64, seeded from a name (FNV-1a of the bytes).
pub struct Rng(pub u64);

impl Rng {
    pub fn named(name: &str) -> Self {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for b in name.bytes() {
            h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
        Rng(h)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Isotropic unit vector.
    pub fn dir(&mut self) -> [f64; 3] {
        let z = 2.0 * self.unit() - 1.0;
        let phi = 2.0 * std::f64::consts::PI * self.unit();
        let s = (1.0 - z * z).max(0.0).sqrt();
        [s * phi.cos(), s * phi.sin(), z]
    }
}

/// A stub tracker handle of a test kind.
pub fn stub(kind: i32, radius: f64) -> *mut SmrtTracker {
    tracker_into_raw(tracker_for(kind, radius).expect("stub kind"))
}

/// A sphere of radius 5.
pub fn sphere5() -> *mut SmrtTracker {
    stub(STUB_SPHERE, 5.0)
}

pub fn ctx_of(t: *const SmrtTracker) -> *mut SmrtCtx {
    let mut c = std::ptr::null_mut();
    // SAFETY: `t` is a live handle, `c` is a valid out slot.
    assert_eq!(unsafe { smrt_ctx_new(t, &mut c) }, 0);
    c
}

pub fn init(c: *mut SmrtCtx, p: [f64; 3], d: [f64; 3]) -> i32 {
    // SAFETY: `c` is a live context used by this thread.
    unsafe { smrt_init_state(c, p[0], p[1], p[2], d[0], d[1], d[2], -1) }
}

pub fn fire(c: *mut SmrtCtx) -> (i32, SmrtStep) {
    let mut s = SmrtStep::default();
    // SAFETY: as above, `s` is a valid out slot.
    let r = unsafe { smrt_fire(c, f64::INFINITY, 0, &mut s) };
    (r, s)
}

pub fn step(c: *mut SmrtCtx, max: f64) -> (i32, SmrtStep) {
    let mut s = SmrtStep::default();
    // SAFETY: as above.
    let r = unsafe { smrt_step(c, max, &mut s) };
    (r, s)
}

pub fn state(c: *const SmrtCtx) -> SmrtState {
    let mut s = SmrtState::default();
    // SAFETY: as above.
    assert_eq!(unsafe { smrt_get_state(c, &mut s) }, 0);
    s
}

pub fn err_msg() -> String {
    let mut buf = vec![0u8; 512];
    // SAFETY: `buf` is valid for 512 bytes.
    let n = unsafe { smrt_last_error_message(buf.as_mut_ptr().cast(), 512) };
    assert!(n >= 0);
    let n = (n as usize).min(511);
    String::from_utf8_lossy(&buf[..n]).into_owned()
}
