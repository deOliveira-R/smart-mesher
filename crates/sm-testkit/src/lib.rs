//! Test support for smart-mesh implementation crates.
//!
//! - [`geom`]: analytic curves and surfaces (the `sm-geom-api` types) with closed-form expectations.
//! - [`loops2d`]: discretized 2D regions with known areas and submap vertex types.
//! - [`meshes`]: small 3D meshes with known volumes, built through `sm-mesh-api`.
//! - [`surf3`]: closed triangulated surfaces (cube, icosphere, torus, cavity, shared interface,
//!   Schonhardt, noisy sphere) with exact enclosed volumes.
//! - [`lattice`]: classified hex/tet/quad/tube lattices on a real topology, with a constrained jitter.
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

pub mod boolean;
pub mod broken;
pub mod check;
pub mod determinism;
pub mod geom;
pub mod lattice;
pub mod loops2d;
pub mod meshes;
pub mod step;
pub mod strategies;
pub mod surf3;
pub mod topo;
pub mod topo_conformance;
pub mod topo_differential;
pub mod w3a;
