//! Exodus II writer in pure Rust, with its own classic-netCDF encoder.
//!
//! [`write_exodus`] turns any [`sm_mesh_api::MeshRead`] mesh into an Exodus II file (CDF-1, CDF-2 or
//! CDF-5 container, no libnetcdf or HDF5). The output is a pure function of the mesh and the
//! [`ExodusOptions`]: no clock, and the same bytes for any thread count.
//!
//! # Mapping (all deterministic)
//! - Nodes are the mesh vertices in index order; `num_dim` is 3, or 2 for a dimension-2 mesh in the
//!   z = 0 plane. Only active cells are written. High-order cells are `Unsupported`.
//! - Element blocks come from the labelled cell sets (label = block id, name = set name). A set
//!   with several shapes splits by shape; cells in no set go to `unassigned_<ELEMTYPE>` blocks.
//!   Blocks are ordered by id, elements numbered block by block; `elem_num_map[e] = cell + 1`.
//! - Template cells (`TET4 HEX8 WEDGE6 PYRAMID5 TRI3 QUAD4`) write `cell_corners + 1` unchanged; the
//!   template corner order already is the Exodus order.
//! - Side sets come from `SetMembers::Sides` (and, optionally, deck `Faces` sets on the left cell);
//!   the side is found by comparing the face loop with the cell's template faces ([`tables`]).
//! - Node sets come from `SetMembers::Vertices` / `Points`.
//! - Polygons become `NSIDED` blocks, polyhedra `NFACED` blocks over a face block with one outward
//!   face copy per (cell, face). The internal names `POLYGON` / `POLYHEDRON` never reach the file.
//!   NFACED names are written from memory of libexodus and are **experimental**
//!   ([`ExodusOptions::polyhedra`]).
//!
//! [`netcdf`] is the codec on its own; [`read`] is an internal decoder for round-trip tests, not a
//! general Exodus reader. See `DESIGN.md` for the positioning, the side tables and the variable list.

#![forbid(unsafe_code)]

pub mod error;
pub mod netcdf;
pub mod options;
pub mod plan;
pub mod read;
pub mod tables;
pub mod write;

pub use error::ExodusError;
pub use options::{BlockSelection, ExodusOptions};
pub use plan::{BlockPlan, ExodusPlan, NodeSetPlan, SideSetPlan, plan};
pub use read::{ExodusData, read_exodus};
pub use tables::{exodus_element_type, exodus_side, local_face};
pub use write::{BlockSummary, ExodusReport, SetSummary, exodus_bytes, exodus_nc, write_exodus};
