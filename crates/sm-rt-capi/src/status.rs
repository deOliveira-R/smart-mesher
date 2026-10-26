//! Status codes of the C ABI and the thread-local last-error message.
//!
//! Every `smrt_*` function returns an `i32`: `0` ok, `1` "no hit", a negative [`RtError::code`]
//! for a contract error, and the ABI-level codes below. The last error message lives in
//! thread-local storage, so there is no global lock and one thread's message never disturbs
//! another's.

use sm_rt_api::RtError;
use std::cell::RefCell;

/// Success.
pub const SMRT_OK: i32 = 0;
/// No hit / nothing within the limit / the relation does not exist (not an error).
pub const SMRT_NO_HIT: i32 = 1;
/// `RtError::Build`.
pub const SMRT_ERR_BUILD: i32 = -1;
/// `RtError::BadDirection`.
pub const SMRT_ERR_BAD_DIRECTION: i32 = -2;
/// `RtError::NonFinite`.
pub const SMRT_ERR_NON_FINITE: i32 = -3;
/// `RtError::UnknownId`.
pub const SMRT_ERR_UNKNOWN_ID: i32 = -4;
/// `RtError::Unsupported` (also `smrt_open` without a loader).
pub const SMRT_ERR_UNSUPPORTED: i32 = -5;
/// `RtError::CacheStale`.
pub const SMRT_ERR_CACHE_STALE: i32 = -6;
/// `RtError::CacheCorrupt`.
pub const SMRT_ERR_CACHE_CORRUPT: i32 = -7;
/// `RtError::Geom`.
pub const SMRT_ERR_GEOM: i32 = -8;
/// `RtError::BadAlbedo`.
pub const SMRT_ERR_BAD_ALBEDO: i32 = -9;
/// `RtError::BinCountMismatch`.
pub const SMRT_ERR_BIN_COUNT: i32 = -10;
/// `RtError::InvalidInput`.
pub const SMRT_ERR_INVALID_INPUT: i32 = -11;
/// A required pointer argument was null.
pub const SMRT_ERR_NULL: i32 = -100;
/// A panic was caught at the boundary (the process survives; see the last-error message).
pub const SMRT_ERR_PANIC: i32 = -101;
/// Bad enum value, bad size or capacity, or an argument outside its domain.
pub const SMRT_ERR_BAD_ARG: i32 = -102;
/// A DAGMC shim lost the particle (a value elsewhere, an error here because the shim has no
/// state to carry it).
pub const SMRT_ERR_LOST: i32 = -103;

/// Why an entry point failed. Converted to a status code and a thread-local message.
#[derive(Debug)]
pub enum Fail {
    /// Null pointer argument (the name of the argument).
    Null(&'static str),
    /// Bad enum, size, capacity or domain (message).
    BadArg(String),
    /// A lost particle seen by a shim.
    Lost(String),
    /// Error from the tracker contract.
    Rt(RtError),
}

impl From<RtError> for Fail {
    fn from(e: RtError) -> Self {
        Self::Rt(e)
    }
}

impl Fail {
    /// The status code.
    pub fn code(&self) -> i32 {
        match self {
            Self::Null(_) => SMRT_ERR_NULL,
            Self::BadArg(_) => SMRT_ERR_BAD_ARG,
            Self::Lost(_) => SMRT_ERR_LOST,
            Self::Rt(e) => e.code(),
        }
    }

    /// Human message for the thread-local slot.
    pub fn message(&self) -> String {
        match self {
            Self::Null(n) => format!("null pointer argument `{n}`"),
            Self::BadArg(m) => m.clone(),
            Self::Lost(m) => m.clone(),
            Self::Rt(e) => e.to_string(),
        }
    }

    /// Shorthand for [`Fail::BadArg`].
    pub fn bad(msg: impl Into<String>) -> Self {
        Self::BadArg(msg.into())
    }
}

thread_local! {
    static LAST_ERROR: RefCell<String> = const { RefCell::new(String::new()) };
}

/// Store `msg` as this thread's last error message.
pub fn set_last_error(msg: &str) {
    // `try_with`: during thread teardown the slot may be gone; losing the message is fine.
    let _ = LAST_ERROR.try_with(|s| {
        let mut s = s.borrow_mut();
        s.clear();
        s.push_str(msg);
    });
}

/// This thread's last error message (empty if none).
pub fn last_error() -> String {
    LAST_ERROR.try_with(|s| s.borrow().clone()).unwrap_or_default()
}

/// Copy this thread's message into `buf` (NUL-terminated, truncated to fit) and return its full
/// length in bytes, excluding the NUL (snprintf semantics). Pure safe code: the FFI layer hands
/// in a slice.
pub fn copy_last_error(buf: &mut [u8]) -> usize {
    LAST_ERROR
        .try_with(|s| {
            let s = s.borrow();
            let bytes = s.as_bytes();
            if let Some(room) = buf.len().checked_sub(1) {
                // Cut at a UTF-8 boundary so the C string stays valid text.
                let mut n = bytes.len().min(room);
                while n > 0 && !s.is_char_boundary(n) {
                    n -= 1;
                }
                buf[..n].copy_from_slice(&bytes[..n]);
                buf[n] = 0;
            }
            bytes.len()
        })
        .unwrap_or(0)
}

/// Message of a caught panic payload.
pub fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    let what = if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "non-string panic payload".to_string()
    };
    format!("panic caught at the C boundary: {what}")
}
