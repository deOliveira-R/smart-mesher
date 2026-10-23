//! Model-semantics and decomposition layers, designed as **attribute tables over topology**.
//!
//! Candidate "tables" (Salome groups / Simmetrix attribute cases): everything is a typed table
//! keyed by [`sm_core::EntityId`]. Topology is reached only through [`topology::TopologyView`].
//!
//! | Module | Tables / content |
//! |---|---|
//! | [`color`] | colors, volume coloring (exact partition), face coloring, named groups |
//! | [`boundary`] | boundary records (deck / response / interface), face membership and pairing |
//! | [`symmetry`] | deck generators + fundamental domain, cut-and-keep-chamber, unfold |
//! | [`detect`] | symmetry detection from colored invariants, and verification |
//! | [`instancing`] | prototypes, instances, lattices, overrides, flatten with paths |
//! | [`decomposition`] | pieces (`PartitionCell`), virtual cuts, piece/volume/face queries |
//! | [`action`] | shared group-action machinery (transform lists, placed copies, paths) |
//! | [`validate`] | validator for semantic invariants |
//! | [`mock`] | in-memory [`topology::TopologyView`] for tests |
//!
//! Out of scope by design: material properties and response values. This layer colors; the
//! consuming program resolves colors.

pub mod action;
pub mod boundary;
pub mod color;
pub mod decomposition;
pub mod detect;
pub mod error;
pub mod instancing;
pub mod mock;
pub mod model;
pub mod symmetry;
pub mod topology;
pub mod validate;

pub use error::{ModelError, Result};
pub use model::{IdAlloc, Lineage, Model};
pub use topology::{FaceSide, Moments, Sense, Tolerance, TopologyView};
