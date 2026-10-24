//! Robust exact predicates (orient2d/3d, incircle/insphere), intervals and numeric utilities.
//!
//! See `DESIGN.md` for the positioning and design notes.
//!
//! - [`predicates`]: adaptive-filter + exact-fallback `orient2d`, `orient3d`, `incircle`,
//!   `insphere`.
//! - [`sos`]: simulation-of-simplicity tie-breaking for degenerate predicate inputs.
//! - [`exact`]: expansion arithmetic underlying the exact stage.
//! - [`interval`]: interval arithmetic with documented conservative widening.
//! - [`roots`]: bracketed Brent and Newton–bisection root finders.
//! - [`linalg`]: small dense solves, symmetric 3x3 eigen decomposition and 3x3 SVD.
//! - [`sum`]: Kahan/Neumaier compensated summation and deterministic parallel reductions.
//!
//! Points, vectors and transforms are `sm-core`'s [`Point3`], [`Vector3`] and [`Transform`].

pub mod exact;
pub mod interval;
pub mod linalg;
pub mod predicates;
pub mod roots;
pub mod sign;
pub mod sos;
pub mod sum;

pub use predicates::{
    incircle, insphere, orient2d, orient3d, try_incircle, try_insphere, try_orient2d, try_orient3d, PredicateError,
};
pub use sign::Sign;
pub use sm_core::{Point3, Transform, Vector3};
