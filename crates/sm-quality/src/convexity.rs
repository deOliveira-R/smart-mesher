//! Cell convexity and the ray-tracing fast-path rule.
//!
//! [`cell_convexity`] mirrors the rule `sm-rt` uses to flag a cell `C_CONVEX` (Cyrus-Beck walk):
//! every face planar within `planarity_tol * diam(face)`, every cell vertex on the inner side of
//! every face plane within `planarity_tol * diam(cell)`, at most [`CONVEX_CHECK_MAX`] faces. The
//! face plane is the `smartmesh.fan_vertex_mean.v1` plane (unit fan normal through the vertex
//! mean). The metric itself is the largest outside distance of a cell vertex over the face
//! planes, relative to `V^(1/3)`. It is not a [`crate::Metric`] and is not in any profile.

use rayon::prelude::*;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, Side, cell_volume};
use smallvec::SmallVec;

use crate::vec::{V, add, cross, dot, norm, scale, sub};

/// Cells with more faces than this are never fast-path eligible (the `sm-rt` check is quadratic).
pub const CONVEX_CHECK_MAX: usize = 64;

/// The `sm-rt` default planarity tolerance (`BuildOptions::planarity_tol`); a test pins it
/// against `sm_rt_api::BuildOptions::default()`.
pub const RT_PLANARITY_TOL: f64 = 1e-9;

/// Fixed chunk of the reductions (order independent anyway: counts and a maximum).
const CHUNK: usize = 2048;

/// Convexity facts of one cell, see [`cell_convexity`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellConvexity {
    /// Number of faces of the cell.
    pub faces: usize,
    /// Every face is planar within the tolerance (a triangle always is).
    pub planar: bool,
    /// Largest outside distance of a cell vertex over the face planes (0 if none is outside).
    /// Absolute, not relative.
    pub excess: f64,
    /// The metric: `excess / V^(1/3)`; NaN if the volume is not positive and finite.
    pub convexity: f64,
    /// Convex in the `sm-rt` sense: all faces planar and `excess <= planarity_tol * diam(cell)`.
    pub convex: bool,
    /// Fast-path eligible: `convex` and at most [`CONVEX_CHECK_MAX`] faces.
    pub eligible: bool,
}

/// Counts over a mesh, see [`convexity_report`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConvexityReport {
    /// Cells examined.
    pub cells: usize,
    /// Convex cells (any face count).
    pub convex: usize,
    /// Fast-path eligible cells (convex and at most [`CONVEX_CHECK_MAX`] faces).
    pub eligible: usize,
    /// Cells with more than [`CONVEX_CHECK_MAX`] faces.
    pub over_cap: usize,
    /// Convex cells with more than [`CONVEX_CHECK_MAX`] faces (the `CONVEX_CHECK_MAX` lever).
    pub convex_over_cap: usize,
    /// Largest `convexity` of any cell with a defined value (0 if none).
    pub max_convexity: f64,
}

fn bbox_diam(pts: &[V]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    norm(sub(hi, lo))
}

/// Convexity facts of cell `c`. `planarity_tol` is relative to a diameter, as in `sm-rt`.
pub fn cell_convexity(mesh: &dyn MeshRead, c: CellIx, planarity_tol: f64) -> CellConvexity {
    let co = mesh.coords();
    let sfs = mesh.cell_faces(c);
    // Face loops in stored order (as `sm-rt` reads them), with spans and the side sign.
    let mut pts: SmallVec<[V; 96]> = SmallVec::new();
    let mut spans: SmallVec<[(u32, u32, f64); 16]> = SmallVec::new();
    for sf in sfs {
        let l = mesh.face_vertices(sf.face());
        let start = pts.len() as u32;
        pts.extend(l.iter().map(|v| [co.x[v.get()], co.y[v.get()], co.z[v.get()]]));
        spans.push((start, l.len() as u32, if sf.side() == Side::Left { 1.0 } else { -1.0 }));
    }
    let faces = sfs.len();
    let finite = pts.iter().all(|q| q.iter().all(|x| x.is_finite()));
    let mut planar = finite && faces > 0;
    // Plane per face from the stored loop: (unit normal, offset, side sign); the outside
    // distance of a point is `sign * (n . p - d)`, the same arithmetic as `sm-rt`.
    let mut planes: SmallVec<[(V, f64, f64); 16]> = SmallVec::new();
    for &(s, n, sign) in &spans {
        let l = &pts[s as usize..(s + n) as usize];
        let apex = scale(l.iter().fold([0.0; 3], |a, &p| add(a, p)), 1.0 / l.len() as f64);
        let mut a = [0.0; 3];
        for i in 0..l.len() {
            a = add(a, scale(cross(sub(l[i], apex), sub(l[(i + 1) % l.len()], apex)), 0.5));
        }
        let len = norm(a);
        if !(len > 0.0 && len.is_finite()) {
            planar = false;
            planes.push(([0.0; 3], 0.0, sign));
            continue;
        }
        let nrm = scale(a, 1.0 / len);
        let d = dot(nrm, apex);
        let diam = bbox_diam(l);
        let dev = l.iter().map(|&p| (dot(nrm, p) - d).abs()).fold(0.0, f64::max);
        if !(l.len() == 3 || dev <= planarity_tol * diam) {
            planar = false;
        }
        planes.push((nrm, d, sign));
    }
    let mut excess = 0.0f64;
    if finite {
        for &(nrm, d, sign) in &planes {
            for &p in &pts {
                excess = excess.max(sign * (dot(nrm, p) - d));
            }
        }
    }
    let diam = bbox_diam(&pts);
    let convex = planar && excess <= planarity_tol * diam;
    let v = cell_volume(mesh, c);
    let convexity = if v > 0.0 && v.is_finite() { excess / v.cbrt() } else { f64::NAN };
    CellConvexity { faces, planar, excess, convexity, convex, eligible: convex && faces <= CONVEX_CHECK_MAX }
}

/// `fast_path_eligible`: the exact `sm-rt` rule for the Cyrus-Beck walk, see [`cell_convexity`].
pub fn fast_path_eligible(mesh: &dyn MeshRead, c: CellIx, planarity_tol: f64) -> bool {
    cell_convexity(mesh, c, planarity_tol).eligible
}

/// The `cell_convexity` metric of every cell, in cell order (bit-identical for any thread count).
pub fn convexity_column(mesh: &dyn MeshRead, planarity_tol: f64) -> Vec<f64> {
    let nc = mesh.count(MeshEntityKind::Cell);
    (0..nc).into_par_iter().map(|c| cell_convexity(mesh, CellIx::new(c), planarity_tol).convexity).collect()
}

/// Counts over all cells (integer sums and a maximum, so order independent).
pub fn convexity_report(mesh: &dyn MeshRead, planarity_tol: f64) -> ConvexityReport {
    let nc = mesh.count(MeshEntityKind::Cell);
    let parts: Vec<ConvexityReport> = (0..nc.div_ceil(CHUNK))
        .into_par_iter()
        .map(|k| {
            let mut r = ConvexityReport::default();
            for c in k * CHUNK..((k + 1) * CHUNK).min(nc) {
                let x = cell_convexity(mesh, CellIx::new(c), planarity_tol);
                r.cells += 1;
                r.convex += usize::from(x.convex);
                r.eligible += usize::from(x.eligible);
                r.over_cap += usize::from(x.faces > CONVEX_CHECK_MAX);
                r.convex_over_cap += usize::from(x.convex && x.faces > CONVEX_CHECK_MAX);
                if x.convexity.is_finite() {
                    r.max_convexity = r.max_convexity.max(x.convexity);
                }
            }
            r
        })
        .collect();
    parts.into_iter().fold(ConvexityReport::default(), |a, b| ConvexityReport {
        cells: a.cells + b.cells,
        convex: a.convex + b.convex,
        eligible: a.eligible + b.eligible,
        over_cap: a.over_cap + b.over_cap,
        convex_over_cap: a.convex_over_cap + b.convex_over_cap,
        max_convexity: a.max_convexity.max(b.max_convexity),
    })
}
