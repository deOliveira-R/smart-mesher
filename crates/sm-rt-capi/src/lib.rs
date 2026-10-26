//! C ABI over any `sm-rt-api` [`Tracker`](sm_rt_api::Tracker) for Fortran / C / C++ transport
//! codes, plus DAGMC-shaped shims. See `DESIGN.md` for the ABI table and the DAGMC differences.
//!
//! The library is built as `libsm_rt_capi` (`cdylib`, `staticlib`) and as an rlib for Rust
//! embedding. The C declarations are in `include/smrt.h`, the Fortran interfaces in
//! `fortran/smrt.f90`.
//!
//! # Safety layout
//! `unsafe` exists only in the [`ffi`] module tree (the `extern "C"` entry points and their pointer
//! helpers). Everything else (status codes, conversions, the bundle codec) is safe Rust, and a
//! test checks that no other file contains the keyword.
//!
//! # Embedding from Rust
//! ```
//! use std::sync::Arc;
//! use sm_rt_api::stub::AnalyticWorld;
//! use sm_rt_api::{ResponseKind, Tracker};
//! use sm_core::{Aabb, Point3};
//!
//! let w = AnalyticWorld::simple(vec![], Aabb { lo: Point3::new(-1.0, -1.0, -1.0), hi: Point3::new(1.0, 1.0, 1.0) }, ResponseKind::Vacuum).unwrap();
//! let t: Arc<dyn Tracker> = Arc::new(w);
//! let h = sm_rt_capi::tracker_into_raw(t);
//! assert!(!h.is_null());
//! // SAFETY: `h` came from `tracker_into_raw` and is freed once.
//! assert_eq!(unsafe { sm_rt_capi::ffi::smrt_tracker_free(h) }, 0);
//! ```

#![deny(unsafe_code)]

pub mod bundle;
pub mod convert;
#[allow(unsafe_code)]
pub mod ffi;
pub mod status;
pub mod types;

pub use ffi::{SmrtCtx, SmrtTracker, tracker_into_raw};
pub use status::*;
pub use types::*;
