//! The mesh skin: the visible boundary of the kept cells as an indexed triangle soup.
//!
//! See `DESIGN.md` for the rules (clip, interfaces, polyhedral fan, vertex and triangle order).

use rayon::prelude::*;
use sm_core::EntityId;
use sm_mesh_api::{CellIx, FaceIx, MeshDim, MeshRead, cell_volume_centroid};

use crate::error::{TOO_LARGE_HINT, ViewError};

/// `Skin::tri_face` value for triangles that come from a polygon cell (surface meshes).
pub const NO_FACE: u32 = u32::MAX;

/// A clip plane. Cells whose centroid has `dot(c - origin, normal) > 0` are removed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clip {
    /// A point on the plane.
    pub origin: [f64; 3],
    /// Plane normal (need not be unit; must be nonzero and finite). The side it points to is cut away.
    pub normal: [f64; 3],
}

/// Options of [`skin`].
#[derive(Clone, Debug, PartialEq)]
pub struct SkinOptions {
    /// Keep only cells classified on one of these entities (`None`: all cells).
    pub volumes: Option<Vec<EntityId>>,
    /// Clip plane, if any.
    pub clip: Option<Clip>,
    /// Also produce the polygon-edge line list.
    pub outline: bool,
    /// Refuse (never decimate) above this many triangles.
    pub max_tris: u32,
}

impl Default for SkinOptions {
    fn default() -> Self {
        Self { volumes: None, clip: None, outline: true, max_tris: 2_000_000 }
    }
}

/// The skin of a mesh, as flat columns.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct Skin {
    /// `x, y, z` per vertex (f64; the wire reply casts to f32): mesh nodes in first-use order, then polygon-centroid vertices.
    pub positions: Vec<f64>,
    /// Triangle vertex indices (3 per triangle).
    pub indices: Vec<u32>,
    /// Dense index of the visible cell of each triangle.
    pub tri_cell: Vec<u32>,
    /// Per triangle: 0 for none, else 1 + index into [`owners`](Self::owners).
    pub tri_owner: Vec<u32>,
    /// Topo entities that own triangles, ascending, deduplicated.
    pub owners: Vec<EntityId>,
    /// Mesh facet index of each triangle ([`NO_FACE`] for surface-mesh polygons).
    pub tri_face: Vec<u32>,
    /// Line list (two vertex indices per segment) of the polygon edges, sorted and unique.
    pub outline_indices: Vec<u32>,
    /// Bounding box of the emitted vertices (zeros when empty).
    pub bbox: [[f64; 3]; 2],
    /// Cells that passed the volume filter but were removed by the clip plane.
    pub clipped_cells: u32,
    /// Cells that remain after the volume filter and the clip.
    pub kept_cells: u32,
    /// Number of vertices that are mesh nodes; vertices from this index on are polygon centroids.
    pub node_vertices: u32,
    /// For each node vertex, the mesh point index it came from.
    pub vertex_node: Vec<u32>,
}

impl Skin {
    /// Number of triangles.
    pub fn tri_count(&self) -> usize {
        self.indices.len() / 3
    }
    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len() / 3
    }
}

struct Piece {
    face: u32,
    cell: u32,
    flip: bool,
    owner: Option<EntityId>,
}

const fn tris_of(n: usize) -> usize {
    match n {
        0..=2 => 0,
        3 => 1,
        4 => 2,
        n => n,
    }
}

/// Compute the skin of `mesh`.
///
/// Errors: [`ViewError::BadClip`] for an unusable plane, [`ViewError::TooLarge`] when the result
/// would have more than `o.max_tris` triangles.
pub fn skin(mesh: &dyn MeshRead, o: &SkinOptions) -> Result<Skin, ViewError> {
    if let Some(c) = &o.clip {
        if c.origin.iter().chain(&c.normal).any(|x| !x.is_finite()) {
            return Err(ViewError::BadClip("origin and normal must be finite".into()));
        }
        if c.normal.iter().all(|x| *x == 0.0) {
            return Err(ViewError::BadClip("normal must be nonzero".into()));
        }
    }
    let nc = mesh.cell_count();
    let mut vols = o.volumes.clone();
    if let Some(v) = &mut vols {
        v.sort();
        v.dedup();
    }
    let cell_owner = |c: usize| mesh.cell_class(CellIx::new(c)).entity;
    let eligible: Vec<bool> = (0..nc)
        .map(|c| mesh.cell_active(CellIx::new(c)) && vols.as_ref().is_none_or(|v| v.binary_search(&cell_owner(c)).is_ok()))
        .collect();
    let removed: Vec<bool> = match &o.clip {
        None => vec![false; nc],
        Some(cl) => (0..nc)
            .into_par_iter()
            .map(|c| {
                let p = cell_volume_centroid(mesh, CellIx::new(c)).1;
                let d = (p.x - cl.origin[0]) * cl.normal[0] + (p.y - cl.origin[1]) * cl.normal[1] + (p.z - cl.origin[2]) * cl.normal[2];
                d > 0.0
            })
            .collect(),
    };
    let kept: Vec<bool> = (0..nc).map(|c| eligible[c] && !removed[c]).collect();
    let clipped_cells = (0..nc).filter(|&c| eligible[c] && removed[c]).count() as u32;
    let kept_cells = kept.iter().filter(|k| **k).count() as u32;

    // Pieces: one per visible polygon side, in facet order.
    let mut pieces: Vec<Piece> = Vec::new();
    if mesh.dim() == MeshDim::Two {
        for c in (0..nc).filter(|&c| kept[c]) {
            pieces.push(Piece { face: NO_FACE, cell: c as u32, flip: false, owner: Some(cell_owner(c)) });
        }
    } else {
        for f in 0..mesh.face_count() {
            let fi = FaceIx::new(f);
            let l = mesh.face_left(fi).get();
            let r = mesh.face_right(fi).map(|r| r.get());
            let model_face = || {
                let k = mesh.face_class(fi);
                (k.dim() == Some(2)).then_some(k.entity)
            };
            let kl = kept[l];
            let kr = r.is_some_and(|r| kept[r]);
            match (kl, kr) {
                (true, true) => {
                    let r = r.unwrap_or(l);
                    if cell_owner(l) != cell_owner(r) {
                        let owner = model_face().or(Some(cell_owner(l)));
                        pieces.push(Piece { face: f as u32, cell: l as u32, flip: false, owner });
                        let owner = model_face().or(Some(cell_owner(r)));
                        pieces.push(Piece { face: f as u32, cell: r as u32, flip: true, owner });
                    }
                }
                (true, false) => {
                    let owner = if r.is_none() { model_face().or(Some(cell_owner(l))) } else { Some(cell_owner(l)) };
                    pieces.push(Piece { face: f as u32, cell: l as u32, flip: false, owner });
                }
                (false, true) => {
                    let r = r.unwrap_or(l);
                    pieces.push(Piece { face: f as u32, cell: r as u32, flip: true, owner: Some(cell_owner(r)) });
                }
                (false, false) => {}
            }
        }
    }
    let loop_of = |p: &Piece| -> Vec<u32> {
        let l: &[sm_mesh_api::VertexIx] = if p.face == NO_FACE {
            mesh.cell_corners(CellIx::new(p.cell as usize)).unwrap_or(&[])
        } else {
            mesh.face_vertices(FaceIx::new(p.face as usize))
        };
        let mut v: Vec<u32> = l.iter().map(|x| x.0).collect();
        if p.flip {
            v.reverse();
        }
        v
    };
    let loops: Vec<Vec<u32>> = pieces.iter().map(loop_of).collect();
    let tris: u64 = loops.iter().map(|l| tris_of(l.len()) as u64).sum();
    if tris > u64::from(o.max_tris) {
        return Err(ViewError::TooLarge { tris, max: o.max_tris, hint: TOO_LARGE_HINT });
    }

    // Node vertices in first-use order.
    let co = mesh.coords();
    let mut remap = vec![u32::MAX; co.len()];
    let mut vertex_node: Vec<u32> = Vec::new();
    for l in &loops {
        if tris_of(l.len()) == 0 {
            continue;
        }
        for &v in l {
            if remap[v as usize] == u32::MAX {
                remap[v as usize] = vertex_node.len() as u32;
                vertex_node.push(v);
            }
        }
    }
    let node_vertices = vertex_node.len() as u32;
    let mut positions: Vec<f64> = Vec::with_capacity((vertex_node.len() + 8) * 3);
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let mut note = |p: [f64; 3], positions: &mut Vec<f64>| {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
            positions.push(p[k]);
        }
    };
    for &v in &vertex_node {
        note([co.x[v as usize], co.y[v as usize], co.z[v as usize]], &mut positions);
    }

    let mut owners: Vec<EntityId> = pieces.iter().filter_map(|p| p.owner).collect();
    owners.sort();
    owners.dedup();

    let mut indices = Vec::with_capacity(tris as usize * 3);
    let mut tri_cell = Vec::with_capacity(tris as usize);
    let mut tri_owner = Vec::with_capacity(tris as usize);
    let mut tri_face = Vec::with_capacity(tris as usize);
    let mut edges: Vec<(u32, u32)> = Vec::new();
    let mut next_centroid = node_vertices;
    for (p, l) in pieces.iter().zip(&loops) {
        let n = l.len();
        let k = tris_of(n);
        if k == 0 {
            continue;
        }
        let ids: Vec<u32> = l.iter().map(|&v| remap[v as usize]).collect();
        match n {
            3 => indices.extend_from_slice(&[ids[0], ids[1], ids[2]]),
            4 => indices.extend_from_slice(&[ids[0], ids[1], ids[2], ids[0], ids[2], ids[3]]),
            _ => {
                let inv = 1.0 / n as f64;
                let mut c = [0.0f64; 3];
                for &v in l {
                    c[0] += co.x[v as usize] * inv;
                    c[1] += co.y[v as usize] * inv;
                    c[2] += co.z[v as usize] * inv;
                }
                note(c, &mut positions);
                let cv = next_centroid;
                next_centroid += 1;
                for i in 0..n {
                    indices.extend_from_slice(&[cv, ids[i], ids[(i + 1) % n]]);
                }
            }
        }
        let owner = p.owner.map_or(0, |e| 1 + owners.binary_search(&e).map_or(0, |i| i as u32));
        for _ in 0..k {
            tri_cell.push(p.cell);
            tri_owner.push(owner);
            tri_face.push(p.face);
        }
        if o.outline {
            for i in 0..n {
                let (a, b) = (ids[i], ids[(i + 1) % n]);
                edges.push((a.min(b), a.max(b)));
            }
        }
    }
    edges.sort_unstable();
    edges.dedup();
    let outline_indices = edges.into_iter().flat_map(|(a, b)| [a, b]).collect();
    let bbox = if positions.is_empty() { [[0.0; 3]; 2] } else { [lo, hi] };
    Ok(Skin {
        positions,
        indices,
        tri_cell,
        tri_owner,
        owners,
        tri_face,
        outline_indices,
        bbox,
        clipped_cells,
        kept_cells,
        node_vertices,
        vertex_node,
    })
}
