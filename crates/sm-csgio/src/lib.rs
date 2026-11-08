#![forbid(unsafe_code)]
//! CSG model import: reads constructive-solid-geometry decks into a deterministic IR and flattens
//! it into a model that tracks through sm-rt's analytic tracker.
//!
//! Only OpenMC is a target (ruling C6). The pieces:
//! * [`ir`]: the format-neutral deck ([`CsgIr`]); format readers (today the [`openmc`] stub) fill it.
//! * [`dnf`]: region expressions to unions of intersections of half-spaces.
//! * [`ImportedCsg`]: universes and lattices instantiated into one flat leaf in canonical order,
//!   region names per ruling C9 ([`resolve_region_names`]), boundary flags as `BoundarySpec`
//!   rules, and a sampled overlap check that rejects overlapping decks and warns about gaps.
//! * [`cli`]: the `smcsg` tool as a library function (the binary only supplies the loader).

pub mod cli;
pub mod dnf;
mod error;
mod instantiate;
pub mod ir;
mod names;
pub mod openmc;
mod world;

pub use dnf::MAX_TERMS;
pub use error::{CellSource, ImportError, OverlapFinding, PathStep};
pub use ir::CsgIr;
pub use names::{VOID_REGION, resolve_region_names};
pub use world::{CheckOptions, CheckOutcome, ImportOptions, ImportReport, ImportedCsg, MAX_RAYS, MAX_SAMPLES, MIN_RAYS, MIN_SAMPLES, Warning};
