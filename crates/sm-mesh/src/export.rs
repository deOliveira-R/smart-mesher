//! Zero-copy views of the primitive columns, for writers (mmap, `.smsh`, VTU, Exodus).
//!
//! The columns of a [`Mesh`] are flat `Vec`s with fixed-width little-endian-native elements
//! (`f64`, `u64`, `u32`). [`RawColumns`] borrows them as typed slices and [`as_bytes`] turns any of
//! them into a byte slice without copying.
//!
//! The index columns whose element type lives in `sm-mesh-api` (`face_vertices`, `face_left`,
//! `cell_faces`, `corners`) are `repr(transparent)` over `u32` and implement `bytemuck::Pod`
//! (the `bytemuck` feature of `sm-mesh-api`, always on here), so [`RawColumns`] lends them as
//! `&[u32]` with no copy; `cell_faces` holds the packed `SignedFace::to_raw` words.

use crate::Mesh;
use bytemuck::Pod;

/// Borrowed primitive columns of a mesh (see the module docs). Offsets are `u64`, the on-disk
/// width; `NO_CELL` (`u32::MAX`) marks "none" in `face_right` and `cell_parent`.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct RawColumns<'a> {
    /// x coordinates of vertices followed by high-order extra nodes.
    pub x: &'a [f64],
    /// y coordinates.
    pub y: &'a [f64],
    /// z coordinates.
    pub z: &'a [f64],
    /// CSR offsets into the face-vertex loops (`n_faces + 1`).
    pub face_vertex_offsets: &'a [u64],
    /// Concatenated face loops (`VertexIx` words).
    pub face_vertices: &'a [u32],
    /// Left cell of every face (`CellIx` words).
    pub face_left: &'a [u32],
    /// Right cell of every face, `NO_CELL` if none.
    pub face_right: &'a [u32],
    /// CSR offsets into the signed cell->face lists (`n_cells + 1`).
    pub cell_face_offsets: &'a [u64],
    /// Concatenated signed cell->face lists (`SignedFace::to_raw` words).
    pub cell_faces: &'a [u32],
    /// CSR offsets into the template corner lists (`n_cells + 1`; equal neighbours = no corners).
    pub corner_offsets: &'a [u64],
    /// Concatenated template corner lists (`VertexIx` words), indexed by `corner_offsets`.
    pub corners: &'a [u32],
    /// Refinement parent of every cell, `NO_CELL` for roots.
    pub cell_parent: &'a [u32],
}

/// View a slice of plain-old-data as bytes (no copy).
pub fn as_bytes<T: Pod>(s: &[T]) -> &[u8] {
    bytemuck::cast_slice(s)
}

impl Mesh {
    /// The primitive columns, borrowed. See [`RawColumns`].
    pub fn raw_columns(&self) -> RawColumns<'_> {
        RawColumns {
            x: &self.x,
            y: &self.y,
            z: &self.z,
            face_vertex_offsets: &self.face_off,
            face_vertices: bytemuck::cast_slice(&self.face_verts),
            face_left: bytemuck::cast_slice(&self.face_left),
            face_right: &self.face_right,
            cell_face_offsets: &self.cell_off,
            cell_faces: bytemuck::cast_slice(&self.cell_faces),
            corner_offsets: &self.corner_off,
            corners: bytemuck::cast_slice(&self.corners),
            cell_parent: &self.cell_parent,
        }
    }
}
