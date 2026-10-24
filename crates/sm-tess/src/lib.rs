//! `sm-tess`: B-rep tessellation for display (W2 task T2).
//!
//! [`tessellate`] turns faces of a [`TopologyView`](sm_topo_api::TopologyView) into one
//! indexed triangle mesh (`f32` positions, `u32` indices) with per-face index ranges and
//! per-edge polylines. [`TessPlugin`] exposes it as the read-only query `view.tessellate`.
//!
//! # Sampling rules
//!
//! - **Edges** are sampled once and shared by every face that uses them. Lines get one segment
//!   (more only if `max_edge` demands it); circles get a closed-form count
//!   `n = ceil(span / min(2 acos(1 - chord_tol / r), angle_tol))`, so the inscribed polygon's
//!   sagitta is at most `chord_tol`; any other curve is bisected until the chord deviation, the
//!   turning angle and `max_edge` are met.
//! - **Grid faces** (a curved surface whose parameter-space loop is a rectangle: cylinder,
//!   cone, sphere, torus, revolved or ruled patches) get interior rows evaluated on the surface
//!   and joined by a merge triangulation. Each row's point count comes from its own iso-`v`
//!   curve (and is at least the blend of the bottom and top counts). On doubly curved surfaces
//!   (everything but plane, cylinder, cone) a triangle sags about `sag_u + sag_v`, so the edges
//!   next to such a face and its rows each use `chord_tol / 2`: the mesh stays within
//!   `chord_tol` of the surface everywhere (measured at most 0.99 `chord_tol` on sphere and
//!   torus; cylinder and cone are developable and meet it with one direction's budget). The `v`
//!   resolution of a grid face is that of its side edges, so a freeform patch whose side edges
//!   are straight lines is not refined across `v`.
//! - **All other faces** are ear-clipped from their boundary points only (planar faces are exact
//!   this way). A curved face that is not a parameter rectangle is therefore resolved at its
//!   boundary only: its interior chord error is not controlled. Such faces are flagged with
//!   [`FaceRange::boundary_only`] (and `boundary_only` in the query reply). See `DESIGN.md`.
//!
//! # Determinism
//!
//! Vertex numbering is fixed (vertices, edge interiors in id order, then per-face points in face
//! id order) and every parallel stage collects in input order, so the output is byte-identical
//! for any thread count.

mod earcut;
mod edges;
mod face;
mod options;
mod query;
mod sample;
mod tess;

pub use options::{TessError, TessOptions};
pub use query::{DEFAULT_RELATIVE_CHORD_TOL, EdgeRangeOut, FaceRangeOut, TessellateQuery, TessellateReply, base64_decode};
pub use tess::{EdgeRange, FaceRange, Tessellation, tessellate, tessellate_entities};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The tessellation plugin: registers the query `view.tessellate`
/// (`{entities, chord_tol?, angle_tol?, max_edge?}` to base64 buffers and ranges). It registers
/// no ops and no layers and never mutates state; it reads the `model` layer, which
/// `sm_layer_api::CoreLayers` registers.
#[derive(Clone, Copy, Debug, Default)]
pub struct TessPlugin;

impl Plugin for TessPlugin {
    fn id(&self) -> &str {
        "tess"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_query::<TessellateQuery>()?;
        Ok(())
    }
}
