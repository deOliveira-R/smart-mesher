//! Reference in-memory implementors and fixtures.
//!
//! [`RefComplex`] implements the whole contract and is what downstream crates build their tests
//! on until `sm-topo` exists. [`fixtures`] builds the canonical test models.

pub mod complex;
pub mod fixtures;
pub(crate) mod ring;

pub use complex::RefComplex;
