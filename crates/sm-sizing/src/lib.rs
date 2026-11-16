//! Production sizing for the smart-mesh driver (W2 task T3): the exact interval solver
//! [`IiaSolver`], the Cubit-style corner picker [`AnglePicker`], size fields ([`size_field`])
//! automatic element size ([`auto_size`]) and the opt-in geometry-aware `curvature`/`point`/`grade` fields ([`size_field_geo`]). See `DESIGN.md`.
//!
//! The solver and the picker implement the `sm-scheme-api` traits and are injected into the
//! driver; this crate registers no ops or layers.

pub mod auto;
pub mod curvature;
pub mod field;
pub mod grade;
pub mod iia;
pub mod picker;

pub use auto::{SizingError, auto_size};
pub use curvature::{is_ungraded_curvature, size_field_geo};
pub use field::size_field;
pub use grade::Graded;
pub use iia::{IiaSolver, compare_deviation_sums, deviation_key, first_violation};
pub use picker::{AnglePicker, classify_angle};
