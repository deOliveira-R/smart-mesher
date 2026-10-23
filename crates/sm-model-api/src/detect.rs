//! Symmetry detection from colored invariants, then staged verification.
//!
//! Stage 0 (cheap, necessary): per color, the volume, barycentre and inertia (all from moments)
//! must be invariant under the candidate transform. Candidate directions come from the principal
//! axes of the global and per-color tensors, the coordinate axes, planar-face normals,
//! face-centroid offsets, bisectors and offsets of same-color volume pairs, and cross products of
//! axes with those directions, so degenerate inertia (hex and C4 assemblies) is covered. Stage 1:
//! every volume must have an image of the same color and moments. Stage 2: every face must have an
//! image with the same moments, face color, boundary record and side volumes.
//!
//! Only the maximal cyclic order per axis is reported, plus a minimal generator set. Detection
//! never changes the model. All tolerances are relative to the model's length scale.
//! [`DetectionResult::complete`] is false when an eigenspace was degenerate and no geometric
//! direction source (face normal, face or volume offset) lay inside it.

use crate::boundary::Deck;
use crate::error::{ModelError, Result};
use crate::model::Model;
use crate::symmetry::{CutSpec, Keep, MAX_GROUP_ORDER, Provenance, dir_perp_to, point_group};
use crate::xf::{Axis, Bins, GeomCache, Mat, Plane, Tolerance, Turns, pt, sym_eigen, transform_face_moments, transform_moments, vector_area_sign, face_shape_close, volume_close};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_topo_api::{Dim, Moments, Region, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Barycentre and inertia tensor of one color.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColorInvariant {
    /// The volume color.
    pub color: EntityId,
    /// Total volume of the color.
    pub volume: f64,
    /// Barycentre.
    pub barycentre: Point3,
    /// Inertia tensor about the barycentre, `J = tr(S) I - S` with `S` the central second moment.
    pub inertia: [[f64; 3]; 3],
}

/// A candidate symmetry element, through the global barycentre.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CandidateKind {
    /// Mirror plane.
    Mirror(Plane),
    /// Rotation by `2 pi / order` about `axis`; `order` is the maximal order found on the axis.
    Rotation {
        /// The axis.
        axis: Axis,
        /// Maximal cyclic order.
        order: u32,
    },
}

/// The verification stage that rejected a candidate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Stage {
    /// Per-color invariants (volume, barycentre, inertia).
    Invariants,
    /// Volume images (same color and moments).
    Volumes,
    /// Face images (moments, face color, boundary record, side colors).
    Faces,
}

/// Outcome of verifying a candidate against the whole model.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Verification {
    /// Not verified (invariants only).
    Unverified,
    /// Volumes, faces, colors and boundary records all map; `max_dev` is the largest centroid
    /// mismatch of matched faces and volumes.
    Verified {
        /// Worst mismatch.
        max_dev: f64,
    },
    /// Something failed to map.
    Rejected {
        /// The failing stage.
        stage: Stage,
        /// A measure of the failure (distance to the nearest candidate image, or invariant deviation).
        residual: f64,
        /// Human-readable cause.
        reason: String,
    },
}

/// One detected candidate.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Candidate {
    /// What it is.
    pub kind: CandidateKind,
    /// Verification status.
    pub verification: Verification,
}

impl Candidate {
    /// The transform of this symmetry element.
    pub fn transform(&self) -> Transform {
        match self.kind {
            CandidateKind::Mirror(p) => p.reflection(),
            CandidateKind::Rotation { axis, order } => Transform::rotation(axis.point, axis.dir, core::f64::consts::TAU / f64::from(order)).unwrap_or(Transform::IDENTITY),
        }
    }

    /// True if verified.
    pub fn is_verified(&self) -> bool {
        matches!(self.verification, Verification::Verified { .. })
    }

    /// The cut that keeps one chamber of this element: a mirror cut, or a `1/order` wedge.
    pub fn chamber(&self, keep: Keep) -> CutSpec {
        match self.kind {
            CandidateKind::Mirror(plane) => CutSpec { deck: Deck::Mirror { plane }, keep, origin: pt(plane.normal * plane.offset) },
            CandidateKind::Rotation { axis, order } => {
                let angle = Turns::new(1, order).unwrap_or(Turns { num: 1, den: 2 });
                CutSpec { deck: Deck::Rotation { axis, angle }, keep, origin: axis.point + dir_perp_to(axis.dir) }
            }
        }
    }
}

/// Detection result.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DetectionResult {
    /// Per-color invariants, ascending by color id.
    pub invariants: Vec<ColorInvariant>,
    /// Candidates: mirrors then rotations, each by canonical direction.
    pub candidates: Vec<Candidate>,
    /// Indices into `candidates` of a minimal generator set of the verified elements.
    pub generators: Vec<usize>,
    /// Order of the group generated by the verified elements (0 if it did not close).
    pub group_order: u32,
    /// False when an eigenspace was degenerate and no in-plane direction source existed, so
    /// symmetry elements may have been missed.
    pub complete: bool,
}

impl DetectionResult {
    /// Candidates that verified.
    pub fn verified(&self) -> impl Iterator<Item = &Candidate> {
        self.candidates.iter().filter(|c| c.is_verified())
    }
}

fn color_moments(m: &Model, c: &mut GeomCache<'_>) -> Result<BTreeMap<EntityId, Moments>> {
    let mut per: BTreeMap<EntityId, Moments> = BTreeMap::new();
    let vols: Vec<EntityId> = c.t.cells(Dim::Volume).to_vec();
    for v in vols {
        let col = m.volume_color(v).ok_or_else(|| ModelError::Invalid(format!("volume {v} is not colored")))?;
        let mv = c.volume_res(v)?;
        let e = per.entry(col).or_insert(Moments { volume: 0.0, first: Vector3::default(), second: [[0.0; 3]; 3] });
        *e = e.add(&mv);
    }
    Ok(per)
}

fn dev(a: &Moments, b: &Moments, l: f64) -> f64 {
    let v = a.volume.abs().max(1e-300);
    let df = (a.first - b.first).norm() / (v * l);
    let mut ds: f64 = 0.0;
    for i in 0..3 {
        for j in 0..3 {
            ds = ds.max((a.second[i][j] - b.second[i][j]).abs());
        }
    }
    ((a.volume - b.volume).abs() / v).max(df).max(ds / (v * l * l))
}

/// Stage 0: every color's moments are invariant under `x`.
fn invariants_stage(per: &BTreeMap<EntityId, Moments>, x: &Transform, l: f64, tol: &Tolerance) -> core::result::Result<(), f64> {
    let mut worst: f64 = 0.0;
    let mut bad = false;
    for mc in per.values() {
        let img = transform_moments(mc, x);
        if !volume_close(&img, mc, l, tol) {
            bad = true;
        }
        worst = worst.max(dev(&img, mc, l));
    }
    if bad { Err(worst) } else { Ok(()) }
}

/// Check that `x` maps the whole colored model onto itself: per-color invariants, volumes (color
/// and moments), then faces (moments, face color, boundary record, side volumes). Usable for any
/// transform, including declared symmetries.
pub fn verify_transform(m: &Model, t: &dyn TopologyView, x: &Transform, tol: Tolerance) -> Result<Verification> {
    let mut c = GeomCache::new(t);
    let l = c.length_scale();
    let per = color_moments(m, &mut c)?;
    if let Err(residual) = invariants_stage(&per, x, l, &tol) {
        return Ok(Verification::Rejected { stage: Stage::Invariants, residual, reason: "a color's volume, barycentre or inertia is not invariant".into() });
    }
    verify_stages(m, &mut c, x, l, &tol)
}

fn verify_stages(m: &Model, c: &mut GeomCache<'_>, x: &Transform, l: f64, tol: &Tolerance) -> Result<Verification> {
    let t = c.t;
    let h = (1e-3 * l).max(100.0 * tol.linear);
    let vols: Vec<EntityId> = t.cells(Dim::Volume).to_vec();
    let mut vm = Vec::with_capacity(vols.len());
    let mut bins = Bins::new(h);
    for (i, v) in vols.iter().enumerate() {
        let mv = c.volume_res(*v)?;
        if let Some(p) = mv.centroid() {
            bins.insert(p, i);
        }
        vm.push(mv);
    }
    let mut used = vec![false; vols.len()];
    let mut vmap: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    let mut worst: f64 = 0.0;
    for (i, v) in vols.iter().enumerate() {
        let img = transform_moments(&vm[i], x);
        let Some(p) = img.centroid() else { continue };
        let hit = bins.near(p).into_iter().find(|&j| !used[j] && m.volume_color(vols[j]) == m.volume_color(*v) && volume_close(&img, &vm[j], l, tol));
        match hit {
            Some(j) => {
                used[j] = true;
                vmap.insert(*v, vols[j]);
                if let Some(q) = vm[j].centroid() {
                    worst = worst.max((q - p).norm());
                }
            }
            None => {
                let residual = bins.near(p).into_iter().filter_map(|j| vm[j].centroid()).map(|q| (q - p).norm()).fold(f64::INFINITY, f64::min);
                return Ok(Verification::Rejected { stage: Stage::Volumes, residual, reason: format!("volume {v} has no image of the same color and moments") });
            }
        }
    }
    let faces: Vec<EntityId> = t.cells(Dim::Face).to_vec();
    let mut fm = Vec::with_capacity(faces.len());
    let mut fbins = Bins::new(h);
    for (i, f) in faces.iter().enumerate() {
        let mf = c.face_res(*f)?;
        if let Some(p) = mf.centroid() {
            fbins.insert(p, i);
        }
        fm.push(mf);
    }
    let map_region = |r: Region| match r {
        Region::Volume(v) => vmap.get(&v).map(|w| Region::Volume(*w)),
        Region::Exterior => Some(Region::Exterior),
    };
    let record = |f: EntityId| m.face_boundary(f).map(|fb| (fb.boundary, std::mem::discriminant(&fb.role)));
    let mut fused = vec![false; faces.len()];
    for (i, f) in faces.iter().enumerate() {
        let img = transform_face_moments(&fm[i], x);
        let Some(p) = img.centroid() else { continue };
        let Some(sf) = t.face_sides(*f) else { continue };
        let (lf, rf) = (map_region(sf.left), map_region(sf.right));
        let hit = fbins.near(p).into_iter().find(|&j| {
            if fused[j] || m.face_color(faces[j]) != m.face_color(*f) || record(faces[j]) != record(*f) || !face_shape_close(&img, &fm[j], l, tol) {
                return false;
            }
            let Some(sg) = t.face_sides(faces[j]) else { return false };
            match vector_area_sign(img.vector_area, fm[j].vector_area, img.area, tol) {
                1 => lf == Some(sg.left) && rf == Some(sg.right),
                -1 => lf == Some(sg.right) && rf == Some(sg.left),
                _ => false,
            }
        });
        match hit {
            Some(j) => {
                fused[j] = true;
                if let Some(q) = fm[j].centroid() {
                    worst = worst.max((q - p).norm());
                }
            }
            None => {
                let residual = fbins.near(p).into_iter().filter_map(|j| fm[j].centroid()).map(|q| (q - p).norm()).fold(f64::INFINITY, f64::min);
                return Ok(Verification::Rejected { stage: Stage::Faces, residual, reason: format!("face {f} has no image with the same moments, color, boundary record and side volumes") });
            }
        }
    }
    Ok(Verification::Verified { max_dev: worst })
}

fn canonical(d: Vector3) -> Vector3 {
    for c in [d.x, d.y, d.z] {
        if c.abs() > 1e-9 {
            return if c < 0.0 { -d } else { d };
        }
    }
    d
}

struct Dirs {
    v: Vec<Vector3>,
}

impl Dirs {
    fn add(&mut self, d: Vector3) -> bool {
        let n = d.norm();
        if !(n.is_finite() && n > 1e-9) || self.v.len() >= 600 {
            return false;
        }
        let d = canonical(d * (1.0 / n));
        if self.v.iter().any(|u| (*u - d).norm() < 1e-6 || (*u + d).norm() < 1e-6) {
            return false;
        }
        self.v.push(d);
        true
    }
}

fn central(m: &Moments) -> (Point3, Mat) {
    let c = m.centroid().unwrap_or_default();
    (c, m.central_second())
}

fn inertia_of(s: &Mat) -> Mat {
    let tr = s[0][0] + s[1][1] + s[2][2];
    let mut j = [[0.0; 3]; 3];
    for (a, row) in j.iter_mut().enumerate() {
        for (b, e) in row.iter_mut().enumerate() {
            *e = if a == b { tr } else { 0.0 } - s[a][b];
        }
    }
    j
}

/// Detect mirror planes and rotation axes of the colored model.
///
/// Requires volume moments for every volume and a color for every volume. Deterministic: all
/// iteration is over ordered tables. Tolerances scale with the model's length scale.
pub fn detect_symmetry(m: &Model, t: &dyn TopologyView, tol: Tolerance) -> Result<DetectionResult> {
    let mut c = GeomCache::new(t);
    let l = c.length_scale();
    let per = color_moments(m, &mut c)?;
    let total = per.values().fold(Moments { volume: 0.0, first: Vector3::default(), second: [[0.0; 3]; 3] }, |a, b| a.add(b));
    if total.volume <= 0.0 {
        return Err(ModelError::Invalid("model has no volume".into()));
    }
    let (g, s_tot) = central(&total);
    let invariants: Vec<ColorInvariant> = per
        .iter()
        .map(|(col, mc)| {
            let (b, s) = central(mc);
            ColorInvariant { color: *col, volume: mc.volume, barycentre: b, inertia: inertia_of(&s) }
        })
        .collect();
    let dtol = tol.relative * l + tol.linear;

    // Candidate directions.
    let mut dirs = Dirs { v: Vec::new() };
    let base = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)];
    let mut axes: Vec<Vector3> = base.to_vec();
    for d in base {
        dirs.add(d);
    }
    let (evals, evecs) = sym_eigen(&s_tot);
    for e in evecs {
        dirs.add(e);
        axes.push(e);
    }
    for mc in per.values() {
        for e in sym_eigen(&central(mc).1).1 {
            dirs.add(e);
        }
    }
    let emax = evals.iter().fold(0.0f64, |a, b| a.max(b.abs())).max(1e-300);
    let close = |a: f64, b: f64| (a - b).abs() < 1e-6 * emax;
    // Geometric direction sources (not cross products, which are derived), new or not: the
    // eigenvectors of a degenerate eigenspace are arbitrary, so what matters is whether the
    // geometry itself offers a direction inside it.
    let mut sources: Vec<Vector3> = Vec::new();
    let mut add_src = |dirs: &mut Dirs, d: Vector3| {
        let n = d.norm();
        if n.is_finite() && n > 1e-9 {
            sources.push(d * (1.0 / n));
        }
        dirs.add(d);
    };
    let faces: Vec<EntityId> = t.cells(Dim::Face).to_vec();
    for f in &faces {
        let fm = c.face_res(*f)?;
        if let Some((n, _)) = fm.plane(1e-6 * l + tol.linear) {
            add_src(&mut dirs, n);
        }
        if let Some(cf) = fm.centroid() {
            add_src(&mut dirs, cf - g);
        }
    }
    let mut bary: BTreeMap<EntityId, Vec<Point3>> = BTreeMap::new();
    let volumes: Vec<EntityId> = t.cells(Dim::Volume).to_vec();
    for v in &volumes {
        if let (Some(col), Some(b)) = (m.volume_color(*v), c.volume_res(*v)?.centroid()) {
            bary.entry(col).or_default().push(b);
        }
    }
    for pts in bary.values() {
        for (i, b) in pts.iter().enumerate() {
            if (*b - g).norm() > dtol {
                add_src(&mut dirs, *b - g);
            }
            for b2 in pts.iter().skip(i + 1).take(24) {
                add_src(&mut dirs, *b - *b2);
            }
        }
    }
    let snapshot = dirs.v.clone();
    for a in &axes {
        for d in &snapshot {
            dirs.add(a.cross(*d));
        }
    }
    let mut ds = dirs.v;
    ds.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)).then(a.z.total_cmp(&b.z)));

    let mut candidates = Vec::new();
    for d in &ds {
        let Ok(plane) = Plane::through(g, *d) else { continue };
        if invariants_stage(&per, &plane.reflection(), l, &tol).is_ok() {
            let verification = verify_stages(m, &mut c, &plane.reflection(), l, &tol)?;
            candidates.push(Candidate { kind: CandidateKind::Mirror(plane), verification });
        }
    }
    for d in &ds {
        let Ok(axis) = Axis::new(g, *d) else { continue };
        let mut found: Vec<(u32, Verification)> = Vec::new();
        for order in (2..=12u32).rev() {
            let Some(x) = Transform::rotation(axis.point, axis.dir, core::f64::consts::TAU / f64::from(order)) else { continue };
            if invariants_stage(&per, &x, l, &tol).is_ok() {
                let v = verify_stages(m, &mut c, &x, l, &tol)?;
                let ok = matches!(v, Verification::Verified { .. });
                found.push((order, v));
                if ok {
                    break;
                }
            }
        }
        if let Some((order, verification)) = found.iter().find(|(_, v)| matches!(v, Verification::Verified { .. })).or(found.first()).cloned() {
            candidates.push(Candidate { kind: CandidateKind::Rotation { axis, order }, verification });
        }
    }

    // Group and minimal generators of the verified elements.
    let mut gens: Vec<Transform> = Vec::new();
    let mut generators = Vec::new();
    let mut order = 1usize;
    let mut closed = true;
    for (i, cand) in candidates.iter().enumerate().filter(|(_, c)| c.is_verified()) {
        let mut trial = gens.clone();
        trial.push(cand.transform());
        match point_group(&trial, MAX_GROUP_ORDER) {
            Ok(el) if el.len() > order => {
                order = el.len();
                gens = trial;
                generators.push(i);
            }
            Ok(_) => {}
            Err(_) => closed = false,
        }
    }
    // A degenerate eigenspace leaves its in-plane (or in-space) axes undetermined; they are only
    // found if a geometric source gave a direction inside it.
    let in_space = |basis: &[Vector3]| sources.iter().any(|d| basis.iter().all(|n| d.dot(*n).abs() < 1e-6));
    let complete = if close(evals[0], evals[2]) {
        !sources.is_empty()
    } else if close(evals[0], evals[1]) {
        in_space(&[evecs[2]])
    } else if close(evals[1], evals[2]) {
        in_space(&[evecs[0]])
    } else {
        true
    };
    Ok(DetectionResult { invariants, candidates, generators, group_order: if closed { order as u32 } else { 0 }, complete })
}

impl Model {
    /// Record that a detected candidate was accepted. Unverified candidates are refused unless
    /// `force` is set. Returns the chamber cut for `keep`; execute it with
    /// [`crate::plan_cut`] and the topology kernel, then [`Model::apply_cut`].
    pub fn declare_detected(&mut self, c: &Candidate, keep: Keep, force: bool) -> Result<CutSpec> {
        if !c.is_verified() && !force {
            return Err(ModelError::Unverified);
        }
        self.provenance.push(Provenance::Detected(c.verification.clone()));
        Ok(c.chamber(keep))
    }
}
