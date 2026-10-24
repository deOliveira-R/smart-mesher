//! Mesh-level drivers: metric columns, validity classification and reports.
//!
//! Every per-cell value is written to its own slot of a pre-allocated column, so the result
//! cannot depend on scheduling. Reductions use the fixed-chunk scheme of [`crate::stats`].

use crate::error::{QualityError, Result};
use crate::metric::{Context, Metric, eval};
use crate::polyhedron;
use crate::solid;
use crate::stats::{Acc, CHUNK, Summary, summarize};
use crate::vec::V;
use rayon::prelude::*;
use sm_mesh_api::{CellIx, CellShape, Coords, FaceNormals, MeshRead, MeshEntityKind};
use sm_core::Point3;
use smallvec::SmallVec;

/// Where the reference normal of 2-D cells comes from. It fixes the sign of Jacobians and
/// areas: a cell whose vertex cycle is clockwise about the normal reads as inverted.
#[derive(Clone, Copy, Default)]
#[non_exhaustive]
pub enum SurfaceNormals<'a> {
    /// The cell's own area normal: planar cells are never inverted, a reflex corner still reads
    /// negative. The default (a 2-D mesh carries no global normal in the contract).
    #[default]
    Own,
    /// One normal for all 2-D cells (e.g. `[0, 0, 1]` for a mesh in the xy-plane).
    Uniform(V),
    /// One normal per cell, indexed by cell index.
    PerCell(&'a [V]),
    /// The oriented normal of the model face each cell is classified on, evaluated at the cell's
    /// vertex mean: the shell orientation rule of `sm-mesh-api` (cells whose entity has no
    /// carrier normal fall back to [`SurfaceNormals::Own`]).
    Field(&'a (dyn FaceNormals + Sync)),
}

impl core::fmt::Debug for SurfaceNormals<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Own => f.write_str("Own"),
            Self::Uniform(n) => f.debug_tuple("Uniform").field(n).finish(),
            Self::PerCell(n) => write!(f, "PerCell({} normals)", n.len()),
            Self::Field(_) => f.write_str("Field(..)"),
        }
    }
}

/// Reference size (area/volume) used by the relative-size metrics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub enum ReferenceSize {
    /// Mean size over all counted cells of the mesh (Verdict's "average size"). Default.
    #[default]
    MeanAll,
    /// Mean size per cell shape (meaningful for mixed meshes).
    MeanPerShape,
    /// A caller-supplied size.
    Fixed(f64),
}

/// Options of the mesh-level drivers.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Options<'a> {
    /// Reference normals for 2-D cells.
    pub normals: SurfaceNormals<'a>,
    /// Reference size for relative-size metrics.
    pub reference_size: ReferenceSize,
    /// Include cells with `cell_active == false` (refined parents). Default `false`: inactive
    /// cells yield `NaN`, are skipped by validity, and are excluded from the reference size.
    pub include_inactive: bool,
    /// Relative threshold below which a scaled Jacobian counts as degenerate (default `1e-12`).
    pub degenerate_tol: f64,
}

impl Default for Options<'_> {
    fn default() -> Self {
        Self { normals: SurfaceNormals::Own, reference_size: ReferenceSize::MeanAll, include_inactive: false, degenerate_tol: 1e-12 }
    }
}

impl<'a> Options<'a> {
    /// Set the reference normals.
    pub fn with_normals(mut self, n: SurfaceNormals<'a>) -> Self {
        self.normals = n;
        self
    }
    /// Set the reference size.
    pub fn with_reference_size(mut self, r: ReferenceSize) -> Self {
        self.reference_size = r;
        self
    }
    /// Include inactive (refined parent) cells.
    pub fn with_inactive(mut self, yes: bool) -> Self {
        self.include_inactive = yes;
        self
    }
    /// Set the degeneracy threshold.
    pub fn with_degenerate_tol(mut self, tol: f64) -> Self {
        self.degenerate_tol = tol;
        self
    }
}

/// Several metric columns of one mesh, one value per cell (SoA).
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct QualityTable {
    /// The metrics, in request order.
    pub metrics: Vec<Metric>,
    /// One column per metric, indexed by cell index. `NaN` where the metric is undefined for the
    /// cell's shape, the cell is inactive, or its coordinates are not finite.
    pub columns: Vec<Vec<f64>>,
}

impl QualityTable {
    /// The column of `metric`, if it was requested.
    pub fn column(&self, metric: Metric) -> Option<&[f64]> {
        self.metrics.iter().position(|m| *m == metric).map(|i| self.columns[i].as_slice())
    }
    /// Summary of the column of `metric`.
    pub fn summary(&self, metric: Metric) -> Option<Summary> {
        self.column(metric).map(summarize)
    }
}

fn shape_key(s: CellShape) -> usize {
    match s {
        CellShape::Tet => 0,
        CellShape::Hex => 1,
        CellShape::Wedge => 2,
        CellShape::Pyramid => 3,
        CellShape::Polyhedron => 4,
        CellShape::Tri => 5,
        CellShape::Quad => 6,
        _ => 7,
    }
}

/// Resolved per-mesh state shared by all cells.
struct Prep<'a, 'o> {
    mesh: &'a dyn MeshRead,
    co: Coords<'a>,
    opts: &'o Options<'o>,
    /// Reference size per shape key (`NaN` if unavailable).
    ref_size: [f64; 8],
}

impl Prep<'_, '_> {
    fn counted(&self, c: CellIx) -> bool {
        self.opts.include_inactive || self.mesh.cell_active(c)
    }

    fn normal(&self, c: usize, pts: &[V]) -> Option<V> {
        match self.opts.normals {
            SurfaceNormals::Own => None,
            SurfaceNormals::Uniform(n) => Some(n),
            SurfaceNormals::PerCell(ns) => Some(ns[c]),
            SurfaceNormals::Field(f) => {
                let k = 1.0 / pts.len().max(1) as f64;
                let m = pts.iter().fold([0.0; 3], |s, q| [s[0] + q[0] * k, s[1] + q[1] * k, s[2] + q[2] * k]);
                let class = self.mesh.classification(MeshEntityKind::Cell, c as u32);
                f.normal(class.entity, Point3::new(m[0], m[1], m[2]))
                    .map(|v| [v.x, v.y, v.z])
                    // A zero or non-finite carrier normal is no normal: fall back to `Own`.
                    .filter(|n| n.iter().all(|x| x.is_finite()) && n.iter().any(|x| *x != 0.0))
            }
        }
    }

    /// Gather the corner coordinates of `ci` into `buf`; false if the cell has no corner list.
    fn gather(&self, ci: CellIx, buf: &mut SmallVec<[V; 8]>) -> bool {
        let Some(corners) = self.mesh.cell_corners(ci) else { return false };
        buf.clear();
        buf.extend(corners.iter().map(|v| {
            let i = v.get();
            [self.co.x[i], self.co.y[i], self.co.z[i]]
        }));
        true
    }

    /// Value of `metric` on cell `c` (`NaN` if not counted / undefined).
    fn value(&self, metric: Metric, c: usize, buf: &mut SmallVec<[V; 8]>, ref_size: Option<f64>) -> f64 {
        let ci = CellIx::new(c);
        if !self.counted(ci) {
            return f64::NAN;
        }
        let shape = self.mesh.cell_shape(ci);
        if !metric.applies(shape) {
            return f64::NAN;
        }
        if shape == CellShape::Polyhedron {
            return polyhedron::eval(self.mesh, ci, metric).unwrap_or(f64::NAN);
        }
        if !self.gather(ci, buf) {
            return f64::NAN;
        }
        // Only 2-D cells use a reference normal (skips the classification lookup for solids).
        let normal = if shape.dim() == sm_mesh_api::MeshDim::Two { self.normal(c, buf) } else { None };
        let ctx = Context { normal, ref_size: ref_size.unwrap_or(self.ref_size[shape_key(shape)]) };
        eval(metric, shape, buf, &ctx).unwrap_or(f64::NAN)
    }

    fn column(&self, metric: Metric) -> Vec<f64> {
        let n = self.mesh.cell_count();
        let mut out = vec![f64::NAN; n];
        out.par_chunks_mut(CHUNK).enumerate().for_each(|(ci, chunk)| {
            let mut buf: SmallVec<[V; 8]> = SmallVec::new();
            for (i, slot) in chunk.iter_mut().enumerate() {
                *slot = self.value(metric, ci * CHUNK + i, &mut buf, None);
            }
        });
        out
    }
}

fn needs_ref(m: Metric) -> bool {
    matches!(m, Metric::RelativeSizeSquared | Metric::ShapeAndSize | Metric::ShearAndSize)
}

/// Deterministic mean of the positive finite `volumes` per shape key.
fn reference_sizes(mesh: &dyn MeshRead, volumes: &[f64], opts: &Options<'_>) -> [f64; 8] {
    let keys: Vec<u8> = (0..volumes.len()).map(|c| shape_key(mesh.cell_shape(CellIx::new(c))) as u8).collect();
    let parts: Vec<[(Acc, u64); 8]> = volumes
        .par_chunks(CHUNK)
        .zip(keys.par_chunks(CHUNK))
        .map(|(vs, ks)| {
            let mut a = [(Acc::default(), 0u64); 8];
            for (&v, &k) in vs.iter().zip(ks) {
                if v.is_finite() && v > 0.0 {
                    a[k as usize].0.add(v);
                    a[k as usize].1 += 1;
                }
            }
            a
        })
        .collect();
    let mut tot = [(Acc::default(), 0u64); 8];
    for p in parts {
        for k in 0..8 {
            tot[k].0.merge(p[k].0);
            tot[k].1 += p[k].1;
        }
    }
    match opts.reference_size {
        ReferenceSize::Fixed(v) => [v; 8],
        ReferenceSize::MeanPerShape => {
            let mut r = [f64::NAN; 8];
            for k in 0..8 {
                if tot[k].1 > 0 {
                    r[k] = tot[k].0.value() / tot[k].1 as f64;
                }
            }
            r
        }
        ReferenceSize::MeanAll => {
            let (mut s, mut n) = (Acc::default(), 0u64);
            for t in tot {
                s.merge(t.0);
                n += t.1;
            }
            [if n > 0 { s.value() / n as f64 } else { f64::NAN }; 8]
        }
    }
}

fn prepare<'a, 'o>(mesh: &'a dyn MeshRead, metrics: &[Metric], opts: &'o Options<'o>) -> Result<Prep<'a, 'o>> {
    if let SurfaceNormals::PerCell(ns) = opts.normals
        && ns.len() != mesh.cell_count()
    {
        return Err(QualityError::BadArgument(format!("PerCell normals: {} given, mesh has {} cells", ns.len(), mesh.cell_count())));
    }
    let bad = |n: &V| !n.iter().all(|x| x.is_finite()) || n.iter().all(|x| *x == 0.0);
    match opts.normals {
        SurfaceNormals::Uniform(n) if bad(&n) => {
            return Err(QualityError::BadArgument(format!("Uniform normal {n:?} is zero or non-finite")));
        }
        SurfaceNormals::PerCell(ns) => {
            // Only 2-D cells use the normal, so a mixed mesh may carry zeros for its solids.
            for (c, n) in ns.iter().enumerate() {
                if bad(n) && mesh.cell_shape(CellIx::new(c)).dim() == sm_mesh_api::MeshDim::Two {
                    return Err(QualityError::BadArgument(format!("PerCell normal of cell {c} ({n:?}) is zero or non-finite")));
                }
            }
        }
        _ => {}
    }
    let mut p = Prep { mesh, co: mesh.coords(), opts, ref_size: [f64::NAN; 8] };
    // A fixed reference size needs no Volume column.
    if let ReferenceSize::Fixed(v) = opts.reference_size {
        p.ref_size = [v; 8];
    } else if metrics.iter().any(|m| needs_ref(*m)) {
        let vols = p.column(Metric::Volume);
        p.ref_size = reference_sizes(mesh, &vols, opts);
    }
    Ok(p)
}

/// Cell shapes present in the mesh (one sequential scan, shared by all requested metrics).
fn shapes_present(mesh: &dyn MeshRead) -> Vec<CellShape> {
    let mut seen = [None; 8];
    for c in 0..mesh.cell_count() {
        let s = mesh.cell_shape(CellIx::new(c));
        seen[shape_key(s)] = Some(s);
        if seen.iter().all(Option::is_some) {
            break;
        }
    }
    seen.iter().flatten().copied().collect()
}

/// Is `metric` defined for at least one of the `present` cell shapes?
fn supported(present: &[CellShape], metric: Metric) -> bool {
    present.iter().any(|&s| metric.applies(s))
}

/// Compute one metric for every cell (indexed by cell index; `NaN` where undefined).
///
/// Errors with [`QualityError::Unsupported`] if the metric applies to no cell of the mesh.
pub fn compute(mesh: &dyn MeshRead, metric: Metric, opts: &Options<'_>) -> Result<Vec<f64>> {
    if !supported(&shapes_present(mesh), metric) {
        return Err(QualityError::Unsupported { metric });
    }
    Ok(prepare(mesh, &[metric], opts)?.column(metric))
}

/// Compute several metrics (one parallel pass each; the reference size is computed once).
pub fn compute_many(mesh: &dyn MeshRead, metrics: &[Metric], opts: &Options<'_>) -> Result<QualityTable> {
    let present = shapes_present(mesh);
    if let Some(&metric) = metrics.iter().find(|m| !supported(&present, **m)) {
        return Err(QualityError::Unsupported { metric });
    }
    let p = prepare(mesh, metrics, opts)?;
    Ok(QualityTable { metrics: metrics.to_vec(), columns: metrics.iter().map(|m| p.column(*m)).collect() })
}

/// Compute `metrics` and summarise each: `(metric, summary)` in request order.
pub fn report(mesh: &dyn MeshRead, metrics: &[Metric], opts: &Options<'_>) -> Result<Vec<(Metric, Summary)>> {
    let t = compute_many(mesh, metrics, opts)?;
    Ok(t.metrics.iter().zip(&t.columns).map(|(m, c)| (*m, summarize(c))).collect())
}

/// Validity class of one cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum CellStatus {
    /// Positive Jacobian at every corner, positive volume.
    Valid,
    /// Zero (within tolerance) Jacobian somewhere, or non-finite coordinates: a collapsed cell.
    Degenerate,
    /// Negative Jacobian at some corner or negative volume: an inverted or tangled cell.
    Inverted,
    /// The cell is inactive and `include_inactive` is off.
    Skipped,
}

/// Result of [`validity`].
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Validity {
    /// Status per cell, indexed by cell index.
    pub status: Vec<CellStatus>,
    /// Number of valid cells.
    pub n_valid: usize,
    /// Number of degenerate cells.
    pub n_degenerate: usize,
    /// Number of inverted cells.
    pub n_inverted: usize,
    /// Number of skipped (inactive) cells.
    pub n_skipped: usize,
}

impl Validity {
    /// Cell indices with the given status, ascending.
    pub fn cells_with(&self, s: CellStatus) -> Vec<CellIx> {
        self.status.iter().enumerate().filter(|(_, x)| **x == s).map(|(i, _)| CellIx::new(i)).collect()
    }
    /// Inverted cells, ascending.
    pub fn inverted(&self) -> Vec<CellIx> {
        self.cells_with(CellStatus::Inverted)
    }
    /// True if no counted cell is inverted or degenerate.
    pub fn all_valid(&self) -> bool {
        self.n_inverted == 0 && self.n_degenerate == 0
    }
}

fn classify(x: f64, tol: f64) -> CellStatus {
    if !x.is_finite() {
        CellStatus::Degenerate
    } else if x < -tol {
        CellStatus::Inverted
    } else if x <= tol {
        CellStatus::Degenerate
    } else {
        CellStatus::Valid
    }
}

/// Classify every cell as valid, degenerate or inverted.
///
/// Template cells and polygons use the scaled Jacobian (negative at any corner = inverted) and
/// the sign of the volume; polyhedra use the volume and star-shapedness about the vertex mean.
/// For 2-D cells the sign is relative to [`Options::normals`].
pub fn validity(mesh: &dyn MeshRead, opts: &Options<'_>) -> Result<Validity> {
    let p = prepare(mesh, &[], opts)?;
    let tol = opts.degenerate_tol;
    let n = mesh.cell_count();
    let mut status = vec![CellStatus::Skipped; n];
    status.par_chunks_mut(CHUNK).enumerate().for_each(|(ci, chunk)| {
        let mut buf: SmallVec<[V; 8]> = SmallVec::new();
        for (i, slot) in chunk.iter_mut().enumerate() {
            let c = ci * CHUNK + i;
            let cell = CellIx::new(c);
            if !p.counted(cell) {
                continue;
            }
            *slot = if mesh.cell_shape(cell) == CellShape::Polyhedron {
                let (v, s, star) = polyhedron::validity_inputs(mesh, cell);
                let scale = s.powf(1.5);
                if !(v.is_finite() && star.is_finite()) {
                    CellStatus::Degenerate
                } else if v < -tol * scale || star < -tol {
                    CellStatus::Inverted
                } else if v <= tol * scale || star <= tol {
                    CellStatus::Degenerate
                } else {
                    CellStatus::Valid
                }
            } else {
                let sj = if mesh.cell_shape(cell) == CellShape::Pyramid && p.gather(cell, &mut buf) && buf.len() == 5 {
                    // The reported pyramid scaled Jacobian is floored at 0; classify on the signed one.
                    if buf.iter().any(|q| q.iter().any(|x| !x.is_finite())) { f64::NAN } else { solid::pyramid_signed_sj(&buf) }
                } else {
                    p.value(Metric::ScaledJacobian, c, &mut buf, None)
                };
                let vol = p.value(Metric::Volume, c, &mut buf, None);
                match classify(sj, tol) {
                    CellStatus::Valid if vol < 0.0 => CellStatus::Inverted,
                    s => s,
                }
            };
        }
    });
    let count = |s| status.iter().filter(|x| **x == s).count();
    Ok(Validity {
        n_valid: count(CellStatus::Valid),
        n_degenerate: count(CellStatus::Degenerate),
        n_inverted: count(CellStatus::Inverted),
        n_skipped: count(CellStatus::Skipped),
        status,
    })
}
