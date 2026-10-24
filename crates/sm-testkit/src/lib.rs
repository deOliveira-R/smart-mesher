//! Test support for smart-mesh implementation crates.
//!
//! - [`geom`]: analytic curves and surfaces (the `sm-geom-api` types) with closed-form expectations.
//! - [`loops2d`]: discretized 2D regions with known areas and submap vertex types.
//! - [`meshes`]: small 3D meshes with known volumes, built through `sm-mesh-api`.
//! - [`topo`]: synthetic B-rep volumes and model fixtures.
//! - [`topo_conformance`]: the generic topology conformance suite for `TopologyEdit` implementations.
//! - [`topo_differential`]: lock-step comparison and a seeded random driver for two implementations.
//! - [`check`]: property checkers (mesh validity, watertightness, classification, Euler,
//!   conformity, hashes).
//! - [`broken`]: deliberately broken inputs for negative tests.
//! - [`determinism`]: the "same result for 1 and N threads, run twice" harness.
//! - [`strategies`]: `proptest` strategies.
//!
//! See `DESIGN.md`.

pub mod broken;
pub mod check;
pub mod determinism;
pub mod geom;
pub mod loops2d;
pub mod meshes;
pub mod strategies;
pub mod topo;
pub mod topo_conformance;
pub mod topo_differential;
