//! Production sizing for the smart-mesh driver (W2 task T3): the exact interval solver
//! [`IiaSolver`], the Cubit-style corner picker [`AnglePicker`], size fields ([`size_field`])
//! and automatic element size ([`auto_size`]). See `DESIGN.md`.
//!
//! The solver and the picker implement the `sm-scheme-api` traits and are injected into the
//! driver; this crate registers no ops or layers.

pub mod auto;
pub mod field;
pub mod iia;
pub mod picker;

pub use auto::{SizingError, auto_size};
pub use field::size_field;
pub use iia::{IiaSolver, compare_deviation_sums, deviation_key, first_violation};
pub use picker::{AnglePicker, classify_angle};
