//! C ABI over any `sm-rt-api` [`Tracker`](sm_rt_api::Tracker) for Fortran / C / C++ transport
//! codes, plus DAGMC-shaped shims. See `DESIGN.md` for the ABI table and the DAGMC differences.
//!
//! The library is built as `libsm_rt_capi` (`cdylib`, `staticlib`) and as an rlib for Rust
//! embedding. The C declarations are in `include/smrt.h`, the Fortran interfaces in
//! `fortran/smrt.f90`.
//!
//! # Safety layout
//! The keyword that marks unchecked code appears only in the [`ffi`] module tree (the `extern "C"`
//! entry points and their pointer helpers). Everything else (status codes, conversions, the bundle
//! codec) is plain checked Rust, and a test greps that no other file under `src/` contains the
//! keyword.
//!
//! # Embedding from Rust
//! [`tracker_into_raw`] wraps any `Arc<dyn Tracker>` in a C handle (example there).

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
