//! Finite-volume geometry and metrics that follow OpenFOAM `checkMesh` (and Fluent's orthogonal
//! quality), so the numbers can be diffed against those tools.
//!
//! Conventions: owner is `face_left`, neighbour is `face_right`, the area vector `Sf` points
//! from owner to neighbour. Face centres and area vectors come from
//! [`sm_mesh_api::face_geometry`] (vertex-mean estimate, area-weighted triangle centroids). Cell
//! centres and volumes use OpenFOAM's pyramid rule, see [`fv_geometry`]. Every formula is
//! written in `DESIGN.md` ("Finite-volume metrics").
//!
//! Every column is written one slot per entity over fixed chunks of [`CHUNK`]; there is no
//! cross-thread accumulation, so the output is bit-identical for any thread count.

use crate::error::{QualityError, Result};
use crate::stats::CHUNK;
use crate::vec::{DBL_MAX, V, add, cross, dot, norm, scale, sub};
use rayon::prelude::*;
use sm_mesh_api::{CellIx, Coords, FaceIx, MeshDim, MeshRead, VertexIx, face_geometry};

/// OpenFOAM's `vSmall` (double precision).
const V_SMALL: f64 = 1e-300;
/// OpenFOAM's `rootVSmall` (double precision).
const ROOT_V_SMALL: f64 = 1e-150;

/// Face and cell geometry of a mesh under the OpenFOAM conventions (SoA columns).
///
/// Inactive cells and faces (see `MeshRead::cell_active`, `MeshRead::face_active`) hold NaN.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FvGeometry {
    /// Area vector `Sf` of every face, owner to neighbour; its length is the face area.
    pub face_area: Vec<V>,
    /// Face centre `Cf`.
    pub face_centre: Vec<V>,
    /// Cell centre `C` (pyramid rule).
    pub cell_centre: Vec<V>,
    /// Cell volume (pyramid rule; differs from `sm_mesh_api::cell_volume` on warped faces).
    pub cell_volume: Vec<f64>,
}

/// Compute [`FvGeometry`]. A dimension-2 mesh gives [`QualityError::BadArgument`].
///
/// Cell rule (OpenFOAM `primitiveMeshCellCentresAndVols`): `cEst` is the mean of the cell's face
/// centres, `pyr3Vol_f = Sf_out . (Cf - cEst)`, centre = `sum(pyr3Vol_f (3/4 Cf + 1/4 cEst)) /
/// sum(pyr3Vol_f)` and volume = `sum(pyr3Vol_f) / 3`. A cell whose pyramid sum is zero gets
/// centre `cEst`. Pyramid volumes are not clipped to `vSmall`.
pub fn fv_geometry(mesh: &dyn MeshRead) -> Result<FvGeometry> {
    if mesh.dim() == MeshDim::Two {
        return Err(QualityError::BadArgument("finite-volume metrics need a dimension-3 mesh".into()));
    }
    let (nf, nc) = (mesh.face_count(), mesh.cell_count());
    let mut face_area = vec![[f64::NAN; 3]; nf];
    let mut face_centre = vec![[f64::NAN; 3]; nf];
    face_area.par_chunks_mut(CHUNK).zip(face_centre.par_chunks_mut(CHUNK)).enumerate().for_each(|(ci, (area, centre))| {
        for (i, (a, c)) in area.iter_mut().zip(centre.iter_mut()).enumerate() {
            let f = FaceIx::new(ci * CHUNK + i);
            if !mesh.face_active(f) {
                continue;
            }
            let g = face_geometry(mesh, f);
            *a = [g.area_vector.x, g.area_vector.y, g.area_vector.z];
            *c = [g.centroid.x, g.centroid.y, g.centroid.z];
        }
    });
    let mut cell_centre = vec![[f64::NAN; 3]; nc];
    let mut cell_volume = vec![f64::NAN; nc];
    let (fa, fc) = (&face_area, &face_centre);
    cell_centre.par_chunks_mut(CHUNK).zip(cell_volume.par_chunks_mut(CHUNK)).enumerate().for_each(|(ci, (centre, volume))| {
        for (i, (c, v)) in centre.iter_mut().zip(volume.iter_mut()).enumerate() {
            let cell = CellIx::new(ci * CHUNK + i);
            if !mesh.cell_active(cell) {
                continue;
            }
            let faces = mesh.cell_faces(cell);
            if faces.is_empty() {
                continue;
            }
            let mut est = [0.0; 3];
            for sf in faces {
                est = add(est, fc[sf.face().get()]);
            }
            let est = scale(est, 1.0 / faces.len() as f64);
            let (mut vol3, mut ctr) = (0.0, [0.0; 3]);
            for sf in faces {
                let f = sf.face().get();
                let out = scale(fa[f], sf.sign());
                let pyr3 = dot(out, sub(fc[f], est));
                let pc = add(scale(fc[f], 0.75), scale(est, 0.25));
                ctr = add(ctr, scale(pc, pyr3));
                vol3 += pyr3;
            }
            *c = if vol3 == 0.0 { est } else { scale(ctr, 1.0 / vol3) };
            *v = vol3 / 3.0;
        }
    });
    Ok(FvGeometry { face_area, face_centre, cell_centre, cell_volume })
}

/// Whether a finite-volume metric is indexed by face or by cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum FvEntity {
    /// One value per face.
    Face,
    /// One value per cell.
    Cell,
}

impl FvEntity {
    /// `"face"` or `"cell"`.
    pub const fn name(self) -> &'static str {
        match self {
            FvEntity::Face => "face",
            FvEntity::Cell => "cell",
        }
    }
}

/// A finite-volume metric (formulas in `DESIGN.md`, "Finite-volume metrics").
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum FvMetric {
    /// Angle in degrees between `d = C_N - C_O` and `Sf` (internal faces; checkMesh fails above 70).
    NonOrthogonality,
    /// OpenFOAM face skewness (dimensionless; internal and boundary faces).
    Skewness,
    /// Fluent orthogonal quality of a cell (1 is best).
    OrthogonalQuality,
    /// `max(V_O, V_N) / min(V_O, V_N)` of an internal face (>= 1; reciprocal of OpenFOAM's `faceVolRatio`).
    VolumeRatio,
    /// `min(w, 1 - w)` of the linear interpolation weight of an internal face (0.5 ideal).
    InterpolationWeight,
    /// `|Sf| / sum |A_i|` over the fan triangles about `Cf` (1 is flat).
    FaceFlatness,
    /// Largest angle in degrees between a fan-triangle normal and `Sf`.
    FaceWarp,
    /// Largest concave vertex angle of a face in degrees (0 if convex).
    FaceConcavity,
    /// `det(M) / (tr(M)/3)^3`, `M = sum_f Sf (x) Sf / |Sf|` (1 for a cube).
    CellDeterminant,
}

impl FvMetric {
    /// Every metric, in declaration order.
    pub const ALL: &'static [FvMetric] = &[
        FvMetric::NonOrthogonality,
        FvMetric::Skewness,
        FvMetric::OrthogonalQuality,
        FvMetric::VolumeRatio,
        FvMetric::InterpolationWeight,
        FvMetric::FaceFlatness,
        FvMetric::FaceWarp,
        FvMetric::FaceConcavity,
        FvMetric::CellDeterminant,
    ];

    /// Whether the metric has one value per face or per cell.
    pub const fn entity(self) -> FvEntity {
        match self {
            FvMetric::OrthogonalQuality | FvMetric::CellDeterminant => FvEntity::Cell,
            _ => FvEntity::Face,
        }
    }

    /// Stable snake_case name (the wire name).
    pub const fn name(self) -> &'static str {
        match self {
            FvMetric::NonOrthogonality => "non_orthogonality",
            FvMetric::Skewness => "skewness",
            FvMetric::OrthogonalQuality => "orthogonal_quality",
            FvMetric::VolumeRatio => "volume_ratio",
            FvMetric::InterpolationWeight => "interpolation_weight",
            FvMetric::FaceFlatness => "face_flatness",
            FvMetric::FaceWarp => "face_warp",
            FvMetric::FaceConcavity => "face_concavity",
            FvMetric::CellDeterminant => "cell_determinant",
        }
    }

    /// The metric named `name` (inverse of [`FvMetric::name`]).
    pub fn from_name(name: &str) -> Option<FvMetric> {
        FvMetric::ALL.iter().copied().find(|m| m.name() == name)
    }

    /// Unit of the values: `"deg"` for angles, `""` for dimensionless metrics.
    pub const fn unit(self) -> &'static str {
        match self {
            FvMetric::NonOrthogonality | FvMetric::FaceWarp | FvMetric::FaceConcavity => "deg",
            _ => "",
        }
    }

    /// True if the metric is defined for internal faces only (NaN on boundary faces).
    pub const fn internal_only(self) -> bool {
        matches!(self, FvMetric::NonOrthogonality | FvMetric::VolumeRatio | FvMetric::InterpolationWeight)
    }
}

/// One computed metric column.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FvColumn {
    /// The metric.
    pub metric: FvMetric,
    /// Whether `values` is indexed by face or by cell.
    pub entity: FvEntity,
    /// One value per entity, NaN where the metric is undefined (boundary face for an
    /// internal-only metric, inactive entity).
    pub values: Vec<f64>,
}

struct Ctx<'a> {
    mesh: &'a dyn MeshRead,
    co: Coords<'a>,
    g: &'a FvGeometry,
}

impl Ctx<'_> {
    fn point(&self, v: VertexIx) -> V {
        let i = v.get();
        [self.co.x[i], self.co.y[i], self.co.z[i]]
    }

    /// Skewness of face `f` (OpenFOAM `faceSkewness`, internal and boundary variants).
    fn skewness(&self, f: usize) -> f64 {
        let fi = FaceIx::new(f);
        let (sf, cf) = (self.g.face_area[f], self.g.face_centre[f]);
        let own = self.g.cell_centre[self.mesh.face_left(fi).get()];
        let cpf = sub(cf, own);
        let d = match self.mesh.face_right(fi) {
            Some(n) => sub(self.g.cell_centre[n.get()], own),
            None => {
                let nrm = scale(sf, 1.0 / (norm(sf) + V_SMALL));
                scale(nrm, dot(nrm, cpf))
            }
        };
        let sv = sub(cpf, scale(d, dot(sf, cpf) / (dot(sf, d) + V_SMALL)));
        let sv_hat = scale(sv, 1.0 / (norm(sv) + V_SMALL));
        let mut fd = 0.2 * norm(d) + V_SMALL;
        for &v in self.mesh.face_vertices(fi) {
            fd = fd.max(dot(sv_hat, sub(self.point(v), cf)).abs());
        }
        norm(sv) / fd
    }

    /// Fan-triangle normals about `Cf`: `(p_{i+1} - p_i) x (Cf - p_i) / 2`, and their lengths.
    fn fan(&self, f: usize, mut visit: impl FnMut(V, f64)) {
        let fi = FaceIx::new(f);
        let l = self.mesh.face_vertices(fi);
        let cf = self.g.face_centre[f];
        for i in 0..l.len() {
            let (p, q) = (self.point(l[i]), self.point(l[(i + 1) % l.len()]));
            let n = scale(cross(sub(q, p), sub(cf, p)), 0.5);
            visit(n, norm(n));
        }
    }

    fn flatness(&self, f: usize) -> f64 {
        let nv = self.mesh.face_vertices(FaceIx::new(f)).len();
        if nv <= 3 {
            return 1.0;
        }
        let mut sum_a = 0.0;
        self.fan(f, |_, len| sum_a += len);
        (norm(self.g.face_area[f]) / (sum_a + ROOT_V_SMALL)).min(1.0)
    }

    fn warp(&self, f: usize) -> f64 {
        let nv = self.mesh.face_vertices(FaceIx::new(f)).len();
        if nv <= 3 {
            return 0.0;
        }
        let sf = self.g.face_area[f];
        if norm(sf) <= ROOT_V_SMALL {
            return 90.0;
        }
        let mut worst = 0.0f64;
        self.fan(f, |n, len| {
            if len > ROOT_V_SMALL {
                worst = worst.max(angle_deg(n, sf));
            }
        });
        worst
    }

    fn concavity(&self, f: usize) -> f64 {
        let fi = FaceIx::new(f);
        let l = self.mesh.face_vertices(fi);
        let sf = self.g.face_area[f];
        let len = norm(sf);
        if l.len() <= 3 || len <= ROOT_V_SMALL {
            return 0.0;
        }
        let nhat = scale(sf, 1.0 / len);
        let n = l.len();
        let mut max_sin = 0.0f64;
        let mut e0 = sub(self.point(l[0]), self.point(l[n - 1]));
        for i in 0..n {
            let e1 = sub(self.point(l[(i + 1) % n]), self.point(l[i]));
            let c = cross(e0, e1);
            if dot(c, nhat) < 0.0 {
                let den = norm(e0) * norm(e1);
                if den > 0.0 {
                    max_sin = max_sin.max(norm(c) / den);
                }
            }
            e0 = e1;
        }
        max_sin.min(1.0).asin().to_degrees()
    }

    fn face_value(&self, m: FvMetric, f: usize) -> f64 {
        let fi = FaceIx::new(f);
        if !self.mesh.face_active(fi) {
            return f64::NAN;
        }
        let (sf, cf) = (self.g.face_area[f], self.g.face_centre[f]);
        let own = self.mesh.face_left(fi).get();
        let nei = self.mesh.face_right(fi).map(CellIx::get);
        match m {
            FvMetric::NonOrthogonality => {
                let Some(n) = nei else { return f64::NAN };
                let d = sub(self.g.cell_centre[n], self.g.cell_centre[own]);
                if norm(d) * norm(sf) == 0.0 { 90.0 } else { angle_deg(d, sf) }
            }
            FvMetric::Skewness => self.skewness(f),
            FvMetric::VolumeRatio => {
                let Some(n) = nei else { return f64::NAN };
                let (a, b) = (self.g.cell_volume[own], self.g.cell_volume[n]);
                let (lo, hi) = (a.min(b), a.max(b));
                if lo.is_nan() || hi.is_nan() {
                    f64::NAN
                } else if lo <= 0.0 {
                    DBL_MAX
                } else {
                    (hi / lo).min(DBL_MAX)
                }
            }
            FvMetric::InterpolationWeight => {
                let Some(n) = nei else { return f64::NAN };
                let to_own = dot(sf, sub(cf, self.g.cell_centre[own])).abs();
                let to_nei = dot(sf, sub(self.g.cell_centre[n], cf)).abs();
                let w = to_nei / (to_own + to_nei + V_SMALL);
                w.min(1.0 - w)
            }
            FvMetric::FaceFlatness => self.flatness(f),
            FvMetric::FaceWarp => self.warp(f),
            FvMetric::FaceConcavity => self.concavity(f),
            FvMetric::OrthogonalQuality | FvMetric::CellDeterminant => f64::NAN,
        }
    }

    fn cell_value(&self, m: FvMetric, c: usize) -> f64 {
        let ci = CellIx::new(c);
        if !self.mesh.cell_active(ci) {
            return f64::NAN;
        }
        let cc = self.g.cell_centre[c];
        let faces = self.mesh.cell_faces(ci);
        match m {
            FvMetric::OrthogonalQuality => {
                if faces.is_empty() {
                    return f64::NAN;
                }
                let mut q = f64::INFINITY;
                for sf in faces {
                    let f = sf.face().get();
                    let out = scale(self.g.face_area[f], sf.sign());
                    q = q.min(cos(out, sub(self.g.face_centre[f], cc)));
                    if let Some(other) = self.mesh.across(*sf) {
                        q = q.min(cos(out, sub(self.g.cell_centre[other.get()], cc)));
                    }
                }
                q
            }
            FvMetric::CellDeterminant => {
                if faces.is_empty() {
                    return f64::NAN;
                }
                let mut mt = [[0.0f64; 3]; 3];
                for sf in faces {
                    let s = self.g.face_area[sf.face().get()];
                    let len = norm(s);
                    if len <= V_SMALL {
                        continue;
                    }
                    for (i, row) in mt.iter_mut().enumerate() {
                        for (j, e) in row.iter_mut().enumerate() {
                            *e += s[i] * s[j] / len;
                        }
                    }
                }
                let tr3 = (mt[0][0] + mt[1][1] + mt[2][2]) / 3.0;
                if tr3 <= 0.0 {
                    return 0.0;
                }
                let det = mt[0][0] * (mt[1][1] * mt[2][2] - mt[1][2] * mt[2][1])
                    - mt[0][1] * (mt[1][0] * mt[2][2] - mt[1][2] * mt[2][0])
                    + mt[0][2] * (mt[1][0] * mt[2][1] - mt[1][1] * mt[2][0]);
                det / (tr3 * tr3 * tr3)
            }
            _ => f64::NAN,
        }
    }
}

/// Angle in degrees between two vectors, `atan2(|a x b|, a . b)` (equal to `acos` of the
/// normalised dot product, but accurate near 0).
fn angle_deg(a: V, b: V) -> f64 {
    norm(cross(a, b)).atan2(dot(a, b)).to_degrees()
}

/// Cosine of the angle between `a` and `b`, 0 when either is zero (Fluent's worst value).
fn cos(a: V, b: V) -> f64 {
    dot(a, b) / (norm(a) * norm(b) + V_SMALL)
}

/// Compute the requested finite-volume metrics over `mesh` from `geom`.
///
/// One parallel pass per metric over fixed chunks of [`CHUNK`]; each entity has its own output
/// slot. Errors with [`QualityError::BadArgument`] for a dimension-2 mesh or if `geom` does not
/// match the mesh (its counts differ).
pub fn fv_compute(mesh: &dyn MeshRead, geom: &FvGeometry, metrics: &[FvMetric]) -> Result<Vec<FvColumn>> {
    if mesh.dim() == MeshDim::Two {
        return Err(QualityError::BadArgument("finite-volume metrics need a dimension-3 mesh".into()));
    }
    let (nf, nc) = (mesh.face_count(), mesh.cell_count());
    if geom.face_area.len() != nf || geom.face_centre.len() != nf || geom.cell_centre.len() != nc || geom.cell_volume.len() != nc {
        return Err(QualityError::BadArgument(format!(
            "geometry has {} faces and {} cells, the mesh has {nf} and {nc}",
            geom.face_area.len(),
            geom.cell_centre.len()
        )));
    }
    let ctx = Ctx { mesh, co: mesh.coords(), g: geom };
    Ok(metrics
        .iter()
        .map(|&m| {
            let entity = m.entity();
            let mut values = vec![f64::NAN; if entity == FvEntity::Face { nf } else { nc }];
            values.par_chunks_mut(CHUNK).enumerate().for_each(|(ci, chunk)| {
                for (i, slot) in chunk.iter_mut().enumerate() {
                    let ix = ci * CHUNK + i;
                    *slot = match entity {
                        FvEntity::Face => ctx.face_value(m, ix),
                        FvEntity::Cell => ctx.cell_value(m, ix),
                    };
                }
            });
            FvColumn { metric: m, entity, values }
        })
        .collect())
}

/// How [`face_to_cell`] combines the face values of a cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Reduce {
    /// Largest face value.
    Max,
    /// Smallest face value.
    Min,
}

/// Reduce a face column to a cell column: per cell, the max or min over its faces, skipping NaN
/// (a cell with no finite face value gets NaN). Face indices beyond `face_values` are skipped.
pub fn face_to_cell(mesh: &dyn MeshRead, face_values: &[f64], r: Reduce) -> Vec<f64> {
    let mut out = vec![f64::NAN; mesh.cell_count()];
    out.par_chunks_mut(CHUNK).enumerate().for_each(|(ci, chunk)| {
        for (i, slot) in chunk.iter_mut().enumerate() {
            let mut acc: Option<f64> = None;
            for sf in mesh.cell_faces(CellIx::new(ci * CHUNK + i)) {
                let Some(&v) = face_values.get(sf.face().get()) else { continue };
                if v.is_nan() {
                    continue;
                }
                acc = Some(match (acc, r) {
                    (None, _) => v,
                    (Some(a), Reduce::Max) => a.max(v),
                    (Some(a), Reduce::Min) => a.min(v),
                });
            }
            *slot = acc.unwrap_or(f64::NAN);
        }
    });
    out
}
