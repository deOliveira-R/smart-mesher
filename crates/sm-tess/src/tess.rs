//! The tessellation driver: shared edge samples, parallel face jobs, deterministic assembly.

use std::collections::BTreeSet;

use rayon::prelude::*;
use sm_core::EntityId;
use sm_geom_api::{CurveDef, SurfaceDef};
use sm_topo_api::{Carrier, CellFlags, Dim, TopologyView, TopologyViewExt};

use crate::edges::{self, EdgeSamples};
use crate::face::{self, FaceOut, Shared};
use crate::options::{TessError, TessOptions};
use crate::sample::{self, P3};

/// The triangles of one face inside [`Tessellation::indices`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceRange {
    /// The face.
    pub face: EntityId,
    /// Offset into [`Tessellation::indices`] (a multiple of 3).
    pub first_index: u32,
    /// Number of indices (a multiple of 3).
    pub index_count: u32,
    /// True for a curved face that was triangulated from its boundary points only (a hole in a
    /// cylindrical wall, a trimmed patch, a multi-loop curved face): its interior deviation from
    /// the surface is not controlled by the tolerances.
    pub boundary_only: bool,
}

/// The polyline of one edge inside [`Tessellation::edge_indices`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EdgeRange {
    /// The edge.
    pub edge: EntityId,
    /// Offset into [`Tessellation::edge_indices`].
    pub first: u32,
    /// Number of vertices of the polyline (at least 2).
    pub count: u32,
}

/// A display mesh of part of a B-rep.
///
/// One vertex table serves faces and edges. Topological vertices come first (ascending id),
/// then the interior samples of each edge (ascending edge id, forward order), then the
/// face-interior points (ascending face id). Adjacent faces reference the *same* vertex
/// indices along a shared edge, so the mesh is watertight by construction, and the numbering
/// does not depend on the thread count.
#[derive(Clone, Debug, PartialEq)]
pub struct Tessellation {
    /// `x, y, z` per vertex.
    pub positions: Vec<f32>,
    /// Three vertex indices per triangle, counter-clockwise about the face normal.
    pub indices: Vec<u32>,
    /// Triangle ranges, ascending by face id.
    pub faces: Vec<FaceRange>,
    /// Edge polyline vertices (edge-forward order), concatenated.
    pub edge_indices: Vec<u32>,
    /// Polyline ranges, ascending by edge id.
    pub edges: Vec<EdgeRange>,
}

impl Tessellation {
    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len() / 3
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Position of vertex `i`. Panics if out of range.
    pub fn position(&self, i: u32) -> [f32; 3] {
        let k = 3 * i as usize;
        [self.positions[k], self.positions[k + 1], self.positions[k + 2]]
    }

    /// The triangle indices of `face`, if it was tessellated.
    pub fn face_indices(&self, face: EntityId) -> Option<&[u32]> {
        let r = self.faces.binary_search_by_key(&face, |r| r.face).ok().map(|i| self.faces[i])?;
        Some(&self.indices[r.first_index as usize..(r.first_index + r.index_count) as usize])
    }

    /// The polyline of `edge` (vertex indices, edge-forward order), if it was sampled.
    pub fn edge_polyline(&self, edge: EntityId) -> Option<&[u32]> {
        let r = self.edges.binary_search_by_key(&edge, |r| r.edge).ok().map(|i| self.edges[i])?;
        Some(&self.edge_indices[r.first as usize..(r.first + r.count) as usize])
    }
}

/// Tessellate `faces` (ids of live faces; duplicates and order are irrelevant).
///
/// The result holds the triangles of each face and the polyline of every edge those faces use,
/// shared between neighbours. See [`Tessellation`] for the layout and the crate docs for the
/// sampling rules. Fails as a whole on the first error in ascending id order.
pub fn tessellate(view: &dyn TopologyView, faces: &[EntityId], opts: &TessOptions) -> Result<Tessellation, TessError> {
    tessellate_parts(view, faces, &[], opts)
}

/// Like [`tessellate`], from arbitrary entities: faces as is, volumes and bodies expanded to
/// their faces, and edges contributing their polylines only.
pub fn tessellate_entities(view: &dyn TopologyView, entities: &[EntityId], opts: &TessOptions) -> Result<Tessellation, TessError> {
    let (faces, edges) = expand(view, entities)?;
    tessellate_parts(view, &faces, &edges, opts)
}

/// Split `entities` into faces and extra edges (sorted, deduplicated).
pub(crate) fn expand(view: &dyn TopologyView, entities: &[EntityId]) -> Result<(Vec<EntityId>, Vec<EntityId>), TessError> {
    let (mut faces, mut edges) = (BTreeSet::new(), BTreeSet::new());
    for &e in entities {
        if !view.is_alive(e) {
            return Err(TessError::BadEntity(e));
        }
        match view.dim_of(e) {
            Some(Dim::Face) => {
                faces.insert(e);
            }
            Some(Dim::Edge) => {
                edges.insert(e);
            }
            Some(Dim::Volume) => faces.extend(view.volume_faces(e).iter().copied()),
            Some(Dim::Body) => {
                for inc in view.boundary(e) {
                    match view.dim_of(inc.cell) {
                        Some(Dim::Volume) => faces.extend(view.volume_faces(inc.cell).iter().copied()),
                        Some(Dim::Face) => {
                            faces.insert(inc.cell);
                        }
                        Some(Dim::Edge) => {
                            edges.insert(inc.cell);
                        }
                        _ => {}
                    }
                }
            }
            _ => return Err(TessError::BadEntity(e)),
        }
    }
    Ok((faces.into_iter().collect(), edges.into_iter().collect()))
}

/// Diagonal of the bounding box of the vertices and a coarse sampling of the edges of
/// `faces` and `extra_edges` (0 if there is nothing to measure).
pub(crate) fn scene_diagonal(view: &dyn TopologyView, faces: &[EntityId], extra_edges: &[EntityId]) -> f64 {
    let mut edges: BTreeSet<EntityId> = extra_edges.iter().copied().collect();
    for &f in faces {
        edges.extend(view.face_edges(f).iter().copied());
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let mut add = |p: P3| {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    };
    for &e in &edges {
        if let Some((a, b)) = view.edge_ends(e) {
            for v in [a, b] {
                if let Carrier::Point(p) = view.carrier(v) {
                    add([p.x, p.y, p.z]);
                }
            }
        }
        if let Carrier::Curve(cc) = view.carrier(e)
            && let Some(c) = view.shapes().curve(cc.curve)
        {
            for i in 0..=8 {
                let t = cc.range.lo + cc.range.len() * (i as f64 / 8.0);
                let p = c.eval(t);
                if p.x.is_finite() && p.y.is_finite() && p.z.is_finite() {
                    add([p.x, p.y, p.z]);
                }
            }
        }
    }
    if lo[0] > hi[0] {
        return 0.0;
    }
    (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt()
}

/// Sampling rule of one edge: the options (halved chord if any adjacent face is on a doubly
/// curved surface) and, for a circle next to a cone, the largest ring radius of that cone.
///
/// A cone is sampled well only if all its rings have the same point count: then every triangle
/// of the grid is a planar quad half spanned by two generators through the apex, and its sag is
/// the circle rule at the largest ring. With different counts a triangle edge joins two rings at
/// different angles, is not a generator and cuts across the whole slant. So each circle next to a
/// cone is sampled as if it had the radius of the largest circle on that cone.
fn edge_rule(view: &dyn TopologyView, edge: EntityId, o: &TessOptions) -> (TessOptions, Option<f64>) {
    let shapes = view.shapes();
    let mut opts = *o;
    let mut ring_radius: Option<f64> = None;
    let mut halved = false;
    for inc in view.coboundary(edge) {
        let Carrier::Surface(sc) = view.carrier(inc.cell) else { continue };
        let Some(def) = shapes.surface_def(sc.surface) else { continue };
        if sample::is_doubly_curved(def) && !halved {
            opts = sample::grid_opts(def, o);
            halved = true;
        }
        if matches!(def, SurfaceDef::Cone(_)) {
            for e in view.face_edges(inc.cell) {
                if let Carrier::Curve(cc) = view.carrier(*e)
                    && let Some(CurveDef::Circle(c)) = shapes.curve_def(cc.curve)
                {
                    ring_radius = Some(ring_radius.map_or(c.radius, |r: f64| r.max(c.radius)));
                }
            }
        }
    }
    (opts, ring_radius)
}

fn first_error<T>(results: Vec<Result<T, TessError>>) -> Result<Vec<T>, TessError> {
    results.into_iter().collect()
}

/// First error in face order, ignoring the scheduler-dependent budget placeholder. Every other
/// error depends on its face alone. If only placeholders remain, the message names the total
/// requested by all faces that got that far (each adds its sum whether or not it trips), which
/// does not depend on the order either.
fn pick_error<T>(results: Vec<Result<T, TessError>>, budget: &std::sync::atomic::AtomicUsize) -> Result<Vec<T>, TessError> {
    let is_placeholder = |e: &TessError| matches!(e, TessError::TooLarge(m) if m == face::BUDGET_TRIPPED);
    if results.iter().any(|r| r.as_ref().is_err_and(is_placeholder)) {
        let real = results.into_iter().filter_map(Result::err).find(|e| !is_placeholder(e));
        return Err(real.unwrap_or_else(|| {
            TessError::TooLarge(format!("grid faces need about {} interior points in total; the limit is {} vertices (raise chord_tol)", budget.load(std::sync::atomic::Ordering::Relaxed), sample::MAX_VERTICES))
        }));
    }
    first_error(results)
}

pub(crate) fn tessellate_parts(view: &dyn TopologyView, faces: &[EntityId], extra_edges: &[EntityId], opts: &TessOptions) -> Result<Tessellation, TessError> {
    opts.validate()?;
    let mut faces: Vec<EntityId> = faces.to_vec();
    faces.sort_unstable();
    faces.dedup();
    for &f in &faces {
        if !(view.is_alive(f) && view.dim_of(f) == Some(Dim::Face)) {
            return Err(TessError::NotAFace(f));
        }
    }
    let mut edge_set: BTreeSet<EntityId> = BTreeSet::new();
    for &e in extra_edges {
        if !(view.is_alive(e) && view.dim_of(e) == Some(Dim::Edge)) {
            return Err(TessError::NotAnEdge(e));
        }
        edge_set.insert(e);
    }
    for &f in &faces {
        edge_set.extend(view.face_edges(f).iter().copied());
    }

    // Vertices (ascending) and the edges that carry a polyline (ascending, non-degenerate).
    let mut vertex_set: BTreeSet<EntityId> = BTreeSet::new();
    let mut edges: Vec<EntityId> = Vec::new();
    for &e in &edge_set {
        let (a, b) = view.edge_ends(e).ok_or_else(|| TessError::edge(e, "edge has no end vertices"))?;
        vertex_set.insert(a);
        vertex_set.insert(b);
        if !view.flags(e).contains(CellFlags::DEGENERATE) {
            edges.push(e);
        }
    }
    let vertices: Vec<EntityId> = vertex_set.into_iter().collect();
    let mut vertex_pos: Vec<P3> = Vec::with_capacity(vertices.len());
    for &v in &vertices {
        match view.carrier(v) {
            Carrier::Point(p) => vertex_pos.push([p.x, p.y, p.z]),
            _ => return Err(TessError::Vertex { vertex: v, reason: "vertex has no point carrier".into() }),
        }
    }

    // Stage A: sample every edge (parallel, results kept in edge order).
    let ends: Vec<(usize, usize)> = edges
        .iter()
        .map(|&e| {
            let (a, b) = view.edge_ends(e).ok_or_else(|| TessError::edge(e, "edge has no end vertices"))?;
            let ix = |v: EntityId| vertices.binary_search(&v).map_err(|_| TessError::edge(e, "end vertex missing"));
            Ok((ix(a)?, ix(b)?))
        })
        .collect::<Result<_, TessError>>()?;
    // Edges next to a doubly curved face use half the chord budget (the face's rows get the other
    // half). The rule looks at all faces of the model, not only the requested ones, so an edge's
    // polyline never depends on which faces were asked for.
    let edge_opts: Vec<(TessOptions, Option<f64>)> = edges.iter().map(|&e| edge_rule(view, e, opts)).collect();
    let sampled: Vec<Result<EdgeSamples, TessError>> = edges.par_iter().zip(ends.par_iter()).zip(edge_opts.par_iter()).map(|((&e, &(a, b)), (o, ring))| edges::sample_edge(view, e, a == b, o, *ring)).collect();
    let sampled = first_error(sampled)?;

    // Stage B (serial): number vertices, then edge interiors in edge order.
    let mut positions: Vec<f32> = Vec::with_capacity(3 * vertices.len());
    for p in &vertex_pos {
        positions.extend(p.iter().map(|&x| x as f32));
    }
    let mut edge_off = Vec::with_capacity(edges.len() + 1);
    let (mut edge_g, mut edge_pos, mut edge_t): (Vec<u32>, Vec<P3>, Vec<f64>) = (Vec::new(), Vec::new(), Vec::new());
    edge_off.push(0);
    let mut next = vertices.len();
    for (k, s) in sampled.iter().enumerate() {
        let (a, b) = ends[k];
        let (ta, tb) = if s.params.is_empty() { (f64::NAN, f64::NAN) } else { (s.params[0], s.params[s.params.len() - 1]) };
        edge_g.push(a as u32);
        edge_pos.push(vertex_pos[a]);
        edge_t.push(ta);
        for (i, p) in s.interior.iter().enumerate() {
            if next >= sample::MAX_VERTICES {
                return Err(TessError::TooLarge(format!("more than {} vertices on the edges alone (raise chord_tol)", sample::MAX_VERTICES)));
            }
            edge_g.push(next as u32);
            next += 1;
            edge_pos.push(*p);
            edge_t.push(s.params[i + 1]);
            positions.extend(p.iter().map(|&x| x as f32));
        }
        edge_g.push(b as u32);
        edge_pos.push(vertex_pos[b]);
        edge_t.push(tb);
        edge_off.push(edge_g.len());
    }
    let g_count = next as u32;

    // Stage C: triangulate faces (parallel, results kept in face order).
    let budget = std::sync::atomic::AtomicUsize::new(0);
    let sh = Shared { view, opts, edges: &edges, edge_off: &edge_off, edge_g: &edge_g, edge_pos: &edge_pos, edge_t: &edge_t, vertices: &vertices, g_count, budget: &budget };
    let outs: Vec<Result<FaceOut, TessError>> = faces.par_iter().map(|&f| face::tessellate_face(&sh, f)).collect();
    let outs = pick_error(outs, &budget)?;

    // Stage D (serial): append face-interior points and remap local indices.
    let mut indices: Vec<u32> = Vec::new();
    let mut face_ranges = Vec::with_capacity(faces.len());
    for (&f, out) in faces.iter().zip(&outs) {
        let offset = positions.len() / 3;
        if offset + out.steiner.len() > sample::MAX_VERTICES || indices.len() + 3 * out.tris.len() >= u32::MAX as usize {
            return Err(TessError::TooLarge(format!("more than {} vertices or 2^32 indices (raise chord_tol)", sample::MAX_VERTICES)));
        }
        for p in &out.steiner {
            positions.extend(p.iter().map(|&x| x as f32));
        }
        let first = indices.len() as u32;
        for t in &out.tris {
            for &i in t {
                indices.push(if i >= g_count { i - g_count + offset as u32 } else { i });
            }
        }
        face_ranges.push(FaceRange { face: f, first_index: first, index_count: indices.len() as u32 - first, boundary_only: out.boundary_only });
    }
    let edge_ranges = edges.iter().enumerate().map(|(k, &e)| EdgeRange { edge: e, first: edge_off[k] as u32, count: (edge_off[k + 1] - edge_off[k]) as u32 }).collect();
    Ok(Tessellation { positions, indices, faces: face_ranges, edge_indices: edge_g, edges: edge_ranges })
}
