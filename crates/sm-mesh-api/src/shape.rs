//! Shape tags (derived from topology) and the fast-path vertex templates.
//!
//! Polyhedra are the general case. A shape tag is a *derived* label checked by the validator;
//! it never carries information the faces do not. Every template-shaped cell also has an ordered
//! corner list (see [`crate::MeshRead::cell_corners`]) in the templates below; `finish` derives it
//! when a mesher built the cell from face loops.
//!
//! Template conventions (faces are listed with OUTWARD loops, right-hand rule):
//! - Tet: 0,1,2 is the base, 3 the apex; with 3 above, 0,1,2 is counter-clockwise from +z.
//! - Hex: 0..3 bottom (counter-clockwise from +z), 4..7 the top ring above 0..3.
//! - Wedge: 0..2 bottom triangle (counter-clockwise from +z), 3..5 the top above them.
//! - Pyramid: 0..3 base (counter-clockwise from +z), 4 apex.

use crate::index::VertexIx;
use smallvec::SmallVec;

/// Shape tag of a 3D cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CellShape {
    /// 4 triangles, 4 vertices.
    Tet,
    /// 6 quads, 8 vertices.
    Hex,
    /// 2 triangles + 3 quads, 6 vertices.
    Wedge,
    /// 1 quad + 4 triangles, 5 vertices.
    Pyramid,
    /// Anything else (general closed polyhedron).
    Polyhedron,
}

/// Shape tag of a face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FaceShape {
    /// Triangle.
    Tri,
    /// Quadrilateral.
    Quad,
    /// General polygon with five or more vertices.
    Polygon,
}

impl FaceShape {
    /// Tag for a loop of `n` vertices (`n >= 3`).
    pub const fn from_vertex_count(n: usize) -> Self {
        match n {
            3 => FaceShape::Tri,
            4 => FaceShape::Quad,
            _ => FaceShape::Polygon,
        }
    }
}

const TET_FACES: &[&[u8]] = &[&[0, 2, 1], &[0, 1, 3], &[1, 2, 3], &[2, 0, 3]];
const HEX_FACES: &[&[u8]] = &[
    &[0, 3, 2, 1],
    &[4, 5, 6, 7],
    &[0, 1, 5, 4],
    &[1, 2, 6, 5],
    &[2, 3, 7, 6],
    &[3, 0, 4, 7],
];
const WEDGE_FACES: &[&[u8]] = &[
    &[0, 2, 1],
    &[3, 4, 5],
    &[0, 1, 4, 3],
    &[1, 2, 5, 4],
    &[2, 0, 3, 5],
];
const PYR_FACES: &[&[u8]] = &[
    &[0, 3, 2, 1],
    &[0, 1, 4],
    &[1, 2, 4],
    &[2, 3, 4],
    &[3, 0, 4],
];

impl CellShape {
    /// Number of vertices of a fast-path shape; `None` for [`CellShape::Polyhedron`].
    pub const fn vertex_count(self) -> Option<usize> {
        match self {
            CellShape::Tet => Some(4),
            CellShape::Hex => Some(8),
            CellShape::Wedge => Some(6),
            CellShape::Pyramid => Some(5),
            CellShape::Polyhedron => None,
        }
    }

    /// Local outward face loops (indices into the ordered vertex list). Empty for polyhedra.
    pub const fn local_faces(self) -> &'static [&'static [u8]] {
        match self {
            CellShape::Tet => TET_FACES,
            CellShape::Hex => HEX_FACES,
            CellShape::Wedge => WEDGE_FACES,
            CellShape::Pyramid => PYR_FACES,
            CellShape::Polyhedron => &[],
        }
    }

    /// Permutation `new[k] = old[perm[k]]` that restores a right-handed ordering after an
    /// improper transform. Empty for polyhedra.
    pub const fn reflect_permutation(self) -> &'static [u8] {
        match self {
            CellShape::Tet => &[0, 2, 1, 3],
            CellShape::Hex => &[0, 3, 2, 1, 4, 7, 6, 5],
            CellShape::Wedge => &[0, 2, 1, 3, 5, 4],
            CellShape::Pyramid => &[0, 3, 2, 1, 4],
            CellShape::Polyhedron => &[],
        }
    }

    /// Classify a cell from its distinct-vertex count and face sizes.
    pub fn detect(n_vertices: usize, face_sizes: &[usize]) -> Self {
        let (mut tris, mut quads, mut other) = (0usize, 0usize, 0usize);
        for &s in face_sizes {
            match s {
                3 => tris += 1,
                4 => quads += 1,
                _ => other += 1,
            }
        }
        match (tris, quads, other, n_vertices) {
            (4, 0, 0, 4) => CellShape::Tet,
            (0, 6, 0, 8) => CellShape::Hex,
            (2, 3, 0, 6) => CellShape::Wedge,
            (4, 1, 0, 5) => CellShape::Pyramid,
            _ => CellShape::Polyhedron,
        }
    }

    /// Shape implied by an ordered vertex count (template shapes only; 5 means pyramid).
    pub const fn from_corner_count(n: usize) -> Option<Self> {
        match n {
            4 => Some(CellShape::Tet),
            5 => Some(CellShape::Pyramid),
            6 => Some(CellShape::Wedge),
            8 => Some(CellShape::Hex),
            _ => None,
        }
    }

    /// Local edges `(a, b)` of the template in the `smartmesh.nodes.v1` (Gmsh) order.
    pub const fn local_edges(self) -> &'static [[u8; 2]] {
        match self {
            CellShape::Tet => &[[0, 1], [1, 2], [2, 0], [3, 0], [3, 2], [3, 1]],
            CellShape::Hex => &[
                [0, 1],
                [0, 3],
                [0, 4],
                [1, 2],
                [1, 5],
                [2, 3],
                [2, 6],
                [3, 7],
                [4, 5],
                [4, 7],
                [5, 6],
                [6, 7],
            ],
            CellShape::Wedge => &[
                [0, 1],
                [0, 2],
                [0, 3],
                [1, 2],
                [1, 4],
                [2, 5],
                [3, 4],
                [3, 5],
                [4, 5],
            ],
            CellShape::Pyramid => &[
                [0, 1],
                [0, 3],
                [0, 4],
                [1, 2],
                [1, 4],
                [2, 3],
                [2, 4],
                [3, 4],
            ],
            CellShape::Polyhedron => &[],
        }
    }

    /// Local quadrilateral faces of the template, in the `smartmesh.nodes.v1` (Gmsh) order of face
    /// nodes. Triangular faces carry no node at degree 2.
    pub const fn local_quads(self) -> &'static [[u8; 4]] {
        match self {
            CellShape::Hex => &[
                [0, 3, 2, 1],
                [0, 1, 5, 4],
                [0, 4, 7, 3],
                [1, 2, 6, 5],
                [2, 3, 7, 6],
                [4, 5, 6, 7],
            ],
            CellShape::Wedge => &[[0, 1, 4, 3], [0, 2, 5, 3], [1, 2, 5, 4]],
            CellShape::Pyramid => &[[0, 3, 2, 1]],
            CellShape::Tet | CellShape::Polyhedron => &[],
        }
    }
}

/// Derive the template corner order of a cell from its outward face loops.
///
/// Returns `None` if the loops do not realise the template of `shape`. Cells whose template
/// has symmetries (a hex has 24) get the first match in a fixed search order (template face 0 against
/// the cell's faces in the given order, rotations ascending), so the result is deterministic for
/// a given face numbering but is only one of several equivalent orderings.
pub fn derive_corners(shape: CellShape, loops: &[&[VertexIx]]) -> Option<SmallVec<[VertexIx; 8]>> {
    let n = shape.vertex_count()?;
    let tf = shape.local_faces();
    let t0 = tf[0];
    for g in loops.iter().filter(|g| g.len() == t0.len()) {
        for r in 0..g.len() {
            let mut map: [Option<VertexIx>; 8] = [None; 8];
            for (i, &l) in t0.iter().enumerate() {
                map[l as usize] = Some(g[(i + r) % g.len()]);
            }
            if propagate(&mut map, tf, loops)
                && map[..n].iter().all(Option::is_some)
                && verify(&map, tf, loops)
            {
                return Some(map[..n].iter().map(|v| v.unwrap_or_default()).collect());
            }
        }
    }
    None
}

fn propagate(map: &mut [Option<VertexIx>; 8], tf: &[&[u8]], loops: &[&[VertexIx]]) -> bool {
    loop {
        let mut changed = false;
        for t in tf {
            let k = t.len();
            if t.iter().all(|&l| map[l as usize].is_some()) {
                continue;
            }
            for ia in 0..k {
                let (a, b) = (t[ia] as usize, t[(ia + 1) % k] as usize);
                let (Some(ma), Some(mb)) = (map[a], map[b]) else {
                    continue;
                };
                // The neighbour face of the cell traverses (ma -> mb) in its outward loop.
                let hit = loops.iter().find_map(|g| {
                    (g.len() == k)
                        .then(|| {
                            (0..k)
                                .find(|&p| g[p] == ma && g[(p + 1) % k] == mb)
                                .map(|p| (*g, p))
                        })
                        .flatten()
                });
                let Some((g, p)) = hit else { continue };
                for i in 0..k {
                    let v = g[(p + k - ia + i) % k];
                    match map[t[i] as usize] {
                        Some(old) if old != v => return false,
                        Some(_) => {}
                        None => {
                            map[t[i] as usize] = Some(v);
                            changed = true;
                        }
                    }
                }
                break;
            }
        }
        if !changed {
            return true;
        }
    }
}

fn verify(map: &[Option<VertexIx>; 8], tf: &[&[u8]], loops: &[&[VertexIx]]) -> bool {
    let mut seen: SmallVec<[VertexIx; 8]> = map.iter().flatten().copied().collect();
    seen.sort_unstable();
    if seen.windows(2).any(|w| w[0] == w[1]) {
        return false;
    }
    tf.iter().all(|t| {
        let want: SmallVec<[VertexIx; 4]> = t
            .iter()
            .map(|&l| map[l as usize].unwrap_or_default())
            .collect();
        loops
            .iter()
            .any(|g| loop_relation(&want, g) == Some(LoopRelation::Same))
    })
}

/// How one closed vertex loop relates to another with the same vertex set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopRelation {
    /// Same cyclic order (same normal).
    Same,
    /// Opposite cyclic order (opposite normal).
    Reversed,
}

/// Compare two loops as cycles. `None` if they are not the same cycle in either direction.
pub fn loop_relation(a: &[VertexIx], b: &[VertexIx]) -> Option<LoopRelation> {
    let n = a.len();
    if n != b.len() || n == 0 {
        return None;
    }
    let s = b.iter().position(|&v| v == a[0])?;
    if (0..n).all(|i| a[i] == b[(s + i) % n]) {
        return Some(LoopRelation::Same);
    }
    if (0..n).all(|i| a[i] == b[(s + n - i) % n]) {
        return Some(LoopRelation::Reversed);
    }
    None
}
