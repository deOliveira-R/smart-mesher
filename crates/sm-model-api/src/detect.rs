//! Symmetry detection from colored invariants, then verification.
//!
//! Step 1 (cheap, necessary conditions): per color, the barycentre and inertia tensor are
//! computed from exact volume moments. A color-preserving mirror or rotation must fix every
//! color's barycentre and leave every color's inertia tensor invariant. Candidate directions
//! come from the principal axes of the global and per-color tensors plus the coordinate axes.
//! Step 2 (sufficient, geometric): the candidate's transform is applied to the whole model and
//! must map volumes to volumes (same color, same moments), faces to faces (same outline,
//! color and boundary kind) and preserve face-volume incidence (with orientation flipped for
//! improper transforms). Detection reports candidates with a [`Verification`] status; it never
//! changes the model.
//!
//! Limitation: when all tensors are degenerate in a plane, only the coordinate and
//! eigenvector axes are tried inside that plane; planes at other angles are missed.

use crate::action::{Axis, Plane, reflection, rotation};
use crate::error::{ModelError, Result};
use crate::model::Model;
use crate::symmetry::{ChamberCut, ChamberSpec, Keep};
use crate::topology::{FaceSide, Moments, Sense, Tolerance, TopologyView};
use sm_core::{EntityId, Point3, Transform, Vector3};
use std::collections::BTreeMap;

/// Barycentre and inertia tensor of one color.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ColorInvariants {
    /// The color.
    pub color: EntityId,
    /// Total volume of the color.
    pub volume: f64,
    /// Barycentre.
    pub barycentre: Point3,
    /// Inertia tensor about the barycentre, `J = tr(S) I - S` with `S` the central second moment.
    pub inertia: [[f64; 3]; 3],
}

/// A candidate symmetry element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CandidateKind {
    /// Mirror plane.
    Mirror {
        /// The plane.
        plane: Plane,
    },
    /// Rotation by `2π/order` about `axis`.
    Rotation {
        /// The axis.
        axis: Axis,
        /// Rotation order.
        order: u32,
    },
}

/// Outcome of verifying a candidate against the full model.
#[derive(Clone, Debug, PartialEq)]
pub enum Verification {
    /// Only the invariants were checked.
    Unverified,
    /// Topology, geometry, colors and boundary kinds all map. `max_deviation` is the largest
    /// point mismatch encountered.
    Verified {
        /// Worst mismatch of mapped points.
        max_deviation: f64,
    },
    /// Something failed to map.
    Rejected {
        /// Human-readable cause.
        reason: String,
    },
}

/// One detected candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    /// What it is.
    pub kind: CandidateKind,
    /// Largest normalized invariant residual (barycentre offset, inertia mismatch).
    pub residual: f64,
    /// Verification status.
    pub status: Verification,
}

impl Candidate {
    /// The transform of this symmetry element.
    pub fn transform(&self) -> Transform {
        match self.kind {
            CandidateKind::Mirror { plane } => reflection(&plane),
            CandidateKind::Rotation { axis, order } => rotation(&axis, core::f64::consts::TAU / f64::from(order)),
        }
    }

    /// True if verified.
    pub fn is_verified(&self) -> bool {
        matches!(self.status, Verification::Verified { .. })
    }

    /// A chamber spec that cuts along this element and keeps the positive side / first sector.
    pub fn chamber(&self, name: &str) -> ChamberSpec {
        let cut = match self.kind {
            CandidateKind::Mirror { plane } => ChamberCut::Mirror { plane, keep: Keep::Positive },
            CandidateKind::Rotation { axis, order } => {
                // Any direction perpendicular to the axis: take the coordinate axis least aligned with it.
                let d = axis.dir;
                let e = if d.x.abs() <= d.y.abs() && d.x.abs() <= d.z.abs() {
                    Vector3::new(1.0, 0.0, 0.0)
                } else if d.y.abs() <= d.z.abs() {
                    Vector3::new(0.0, 1.0, 0.0)
                } else {
                    Vector3::new(0.0, 0.0, 1.0)
                };
                let p = e - d * e.dot(d);
                ChamberCut::Wedge { axis, order, start: p * (1.0 / p.norm()) }
            }
        };
        ChamberSpec { cut, name: name.to_owned(), cap_color: None }
    }
}

/// Detection parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetectOptions {
    /// Tolerances.
    pub tol: Tolerance,
    /// Highest rotation order tried.
    pub max_rotation_order: u32,
    /// Run the geometric verification of each candidate.
    pub verify: bool,
}

impl Default for DetectOptions {
    fn default() -> Self {
        Self { tol: Tolerance::default(), max_rotation_order: 12, verify: true }
    }
}

/// Detection result.
#[derive(Clone, Debug, PartialEq)]
pub struct DetectionResult {
    /// Per-color invariants, ascending by color id.
    pub invariants: Vec<ColorInvariants>,
    /// Global barycentre (all symmetry elements pass through it).
    pub global_barycentre: Point3,
    /// Candidates in canonical order: mirrors then rotations, by direction then order.
    pub candidates: Vec<Candidate>,
}

impl DetectionResult {
    /// Candidates that verified.
    pub fn verified(&self) -> impl Iterator<Item = &Candidate> {
        self.candidates.iter().filter(|c| c.is_verified())
    }
}

type Mat = [[f64; 3]; 3];

fn mat_mul(a: &Mat, b: &Mat) -> Mat {
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}

fn transpose(a: &Mat) -> Mat {
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = a[j][i];
        }
    }
    m
}

fn max_abs_diff(a: &Mat, b: &Mat) -> f64 {
    (0..3).flat_map(|i| (0..3).map(move |j| (a[i][j] - b[i][j]).abs())).fold(0.0, f64::max)
}

fn max_abs(a: &Mat) -> f64 {
    a.iter().flatten().fold(0.0, |m, v| m.max(v.abs()))
}

fn outer(a: Vector3, b: Vector3) -> Mat {
    let (a, b) = ([a.x, a.y, a.z], [b.x, b.y, b.z]);
    let mut m = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = a[i] * b[j];
        }
    }
    m
}

/// Eigenvectors of a symmetric 3x3 matrix by cyclic Jacobi (deterministic). Columns of the
/// returned matrix are unit eigenvectors, sorted by ascending eigenvalue.
fn sym_eigenvectors(a: &Mat) -> [Vector3; 3] {
    let mut a = *a;
    let mut v: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off <= 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() <= 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            let mut r: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            r[p][p] = c;
            r[q][q] = c;
            r[p][q] = s;
            r[q][p] = -s;
            a = mat_mul(&transpose(&r), &mat_mul(&a, &r));
            v = mat_mul(&v, &r);
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|i, j| a[*i][*i].total_cmp(&a[*j][*j]));
    idx.map(|c| Vector3::new(v[0][c], v[1][c], v[2][c]))
}

fn canonical_sign(d: Vector3) -> Vector3 {
    for c in [d.x, d.y, d.z] {
        if c.abs() > 1e-12 {
            return if c < 0.0 { -d } else { d };
        }
    }
    d
}

fn accumulate(acc: &mut Moments, m: &Moments) {
    acc.volume += m.volume;
    acc.first = acc.first + m.first;
    for i in 0..3 {
        for j in 0..3 {
            acc.second[i][j] += m.second[i][j];
        }
    }
}

fn central(m: &Moments) -> (Point3, Mat) {
    let c = Point3::new(m.first.x / m.volume, m.first.y / m.volume, m.first.z / m.volume);
    let cv = Vector3::new(c.x, c.y, c.z);
    let mut s = m.second;
    let o = outer(cv, cv);
    for i in 0..3 {
        for j in 0..3 {
            s[i][j] -= m.volume * o[i][j];
        }
    }
    (c, s)
}

fn inertia_of(s: &Mat) -> Mat {
    let tr = s[0][0] + s[1][1] + s[2][2];
    let mut j = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            j[a][b] = if a == b { tr } else { 0.0 } - s[a][b];
        }
    }
    j
}

/// Detect mirror planes and rotation axes of the colored model.
///
/// Requires exact moments for every colored volume. Deterministic: all iteration is over
/// ordered tables.
pub fn detect_symmetry(model: &Model, topo: &dyn TopologyView, opts: &DetectOptions) -> Result<DetectionResult> {
    let zero = Moments { volume: 0.0, first: Vector3::default(), second: [[0.0; 3]; 3] };
    let mut per_color: BTreeMap<EntityId, Moments> = BTreeMap::new();
    let mut total = zero;
    for (v, c) in &model.volume_color {
        let m = topo.volume_moments(*v).ok_or_else(|| ModelError::Invalid(format!("no moments for volume {v}")))?;
        accumulate(per_color.entry(*c).or_insert(zero), &m);
        accumulate(&mut total, &m);
    }
    if total.volume <= 0.0 {
        return Err(ModelError::Invalid("model has no colored volume".into()));
    }
    let (g, s_total) = central(&total);
    let invariants: Vec<ColorInvariants> = per_color
        .iter()
        .map(|(c, m)| {
            let (b, s) = central(m);
            ColorInvariants { color: *c, volume: m.volume, barycentre: b, inertia: inertia_of(&s) }
        })
        .collect();

    // Candidate directions.
    let mut dirs: Vec<Vector3> = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)];
    dirs.extend(sym_eigenvectors(&s_total));
    for m in per_color.values() {
        dirs.extend(sym_eigenvectors(&central(m).1));
    }
    let mut uniq: Vec<Vector3> = Vec::new();
    for d in dirs.into_iter().map(canonical_sign) {
        if !uniq.iter().any(|u| (*u - d).norm() <= 1e-9) {
            uniq.push(d);
        }
    }

    let scale = invariants.iter().map(|i| max_abs(&i.inertia)).fold(1.0, f64::max);
    let length_scale = 1.0 + invariants.iter().map(|i| (i.barycentre - g).norm()).fold(0.0, f64::max);
    let check = |t: &Transform, dist: &dyn Fn(Point3) -> f64| -> Option<f64> {
        let mut res: f64 = 0.0;
        for i in &invariants {
            let d = dist(i.barycentre) / length_scale;
            res = res.max(d);
            let rj = mat_mul(&t.m, &mat_mul(&i.inertia, &transpose(&t.m)));
            res = res.max(max_abs_diff(&rj, &i.inertia) / scale);
        }
        (res <= opts.tol.linear).then_some(res)
    };

    let mut candidates = Vec::new();
    for d in &uniq {
        if let Some(plane) = Plane::new(g, *d) {
            let t = reflection(&plane);
            if let Some(residual) = check(&t, &|p| plane.signed_distance(p).abs()) {
                candidates.push(Candidate { kind: CandidateKind::Mirror { plane }, residual, status: Verification::Unverified });
            }
        }
    }
    for d in &uniq {
        if let Some(axis) = Axis::new(g, *d) {
            for order in 2..=opts.max_rotation_order {
                let t = rotation(&axis, core::f64::consts::TAU / f64::from(order));
                if let Some(residual) = check(&t, &|p| axis.distance(p)) {
                    candidates.push(Candidate { kind: CandidateKind::Rotation { axis, order }, residual, status: Verification::Unverified });
                }
            }
        }
    }
    if opts.verify {
        for c in &mut candidates {
            c.status = verify_transform(model, topo, &c.transform(), &opts.tol);
        }
    }
    Ok(DetectionResult { invariants, global_barycentre: g, candidates })
}

/// Unit Newell normal of a polygon outline (right-hand rule); zero if degenerate.
fn outline_normal(o: &[Point3]) -> Vector3 {
    let mut n = Vector3::default();
    for i in 0..o.len() {
        let (a, b) = (o[i], o[(i + 1) % o.len()]);
        n = n + Vector3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
    }
    let l = n.norm();
    if l > 1e-300 { n * (1.0 / l) } else { Vector3::default() }
}

fn signed(n: Vector3, s: Sense) -> Vector3 {
    if s == Sense::Forward { n } else { -n }
}

/// Does the image of `fs` (a use of face `f` by volume `v`) appear among `gsides` as a use by `w`
/// with the same outward direction? Loop direction of the two faces is irrelevant: outward
/// normals are compared geometrically, so improper transforms need no special casing.
fn sides_agree(t: &Transform, nf: Vector3, fs: &FaceSide, ng: Vector3, gsides: &[FaceSide], w: EntityId) -> bool {
    let out = t.apply_vector(signed(nf, fs.sense));
    if nf.norm() == 0.0 || ng.norm() == 0.0 {
        return gsides.iter().any(|g| g.volume == w);
    }
    gsides.iter().any(|g| g.volume == w && signed(ng, g.sense).dot(out) > 0.5)
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * (1.0 + a.abs() + b.abs())
}

fn point_set_match(a: &[Point3], b: &[Point3], tol: f64) -> Option<f64> {
    if a.len() != b.len() {
        return None;
    }
    let mut used = vec![false; b.len()];
    let mut worst: f64 = 0.0;
    for p in a {
        let (j, d) = b
            .iter()
            .enumerate()
            .filter(|(j, _)| !used[*j])
            .map(|(j, q)| (j, (*p - *q).norm()))
            .min_by(|x, y| x.1.total_cmp(&y.1))?;
        if d > tol {
            return None;
        }
        used[j] = true;
        worst = worst.max(d);
    }
    Some(worst)
}

/// Check that `t` maps the whole colored model onto itself: volumes (color + moments), faces
/// (outline, color, boundary kind) and face-volume incidence with orientation. This is the
/// verification used for detected candidates and is also usable for declared symmetries.
pub fn verify_transform(model: &Model, topo: &dyn TopologyView, t: &Transform, tol: &Tolerance) -> Verification {
    let rej = |reason: String| Verification::Rejected { reason };
    let vols = topo.volumes();
    let mut vmap: BTreeMap<EntityId, EntityId> = BTreeMap::new();
    let mut used = vec![false; vols.len()];
    let mut worst: f64 = 0.0;
    let rel = tol.linear.max(1e-12);
    for v in &vols {
        let Some(m) = topo.volume_moments(*v) else { return rej(format!("no moments for {v}")) };
        // x' = R x + t
        let rf = t.apply_vector(m.first);
        let first = rf + t.t * m.volume;
        let rs = mat_mul(&t.m, &mat_mul(&m.second, &transpose(&t.m)));
        let (tv, rfv) = (outer(t.t, rf), outer(rf, t.t));
        let mut second = rs;
        let tt = outer(t.t, t.t);
        for i in 0..3 {
            for j in 0..3 {
                second[i][j] += tv[i][j] + rfv[i][j] + m.volume * tt[i][j];
            }
        }
        let color = model.volume_color.get(v).copied();
        let hit = vols.iter().enumerate().find(|(k, w)| {
            if used[*k] || model.volume_color.get(*w).copied() != color {
                return false;
            }
            let Some(n) = topo.volume_moments(**w) else { return false };
            close(n.volume, m.volume, rel)
                && close(n.first.x, first.x, rel)
                && close(n.first.y, first.y, rel)
                && close(n.first.z, first.z, rel)
                && (0..3).all(|i| (0..3).all(|j| close(n.second[i][j], second[i][j], rel)))
        });
        match hit {
            Some((k, w)) => {
                used[k] = true;
                vmap.insert(*v, *w);
            }
            None => return rej(format!("volume {v} has no image of the same color and moments")),
        }
    }
    let faces = topo.faces();
    let mut fused = vec![false; faces.len()];
    for f in &faces {
        let img: Vec<Point3> = topo.face_outline(*f).iter().map(|p| t.apply_point(*p)).collect();
        let nf = outline_normal(&topo.face_outline(*f));
        let fsides = topo.face_sides(*f);
        let kind = model.face_kind(*f);
        let fcolor = model.face_color.get(f).copied();
        let mut found = None;
        for (k, g) in faces.iter().enumerate() {
            if fused[k] || model.face_color.get(g).copied() != fcolor || model.face_kind(*g) != kind {
                continue;
            }
            let gsides = topo.face_sides(*g);
            let ng = outline_normal(&topo.face_outline(*g));
            if gsides.len() != fsides.len() || !fsides.iter().all(|fs| sides_agree(t, nf, fs, ng, &gsides, vmap[&fs.volume])) {
                continue;
            }
            if let Some(d) = point_set_match(&img, &topo.face_outline(*g), tol.linear) {
                found = Some((k, d));
                break;
            }
        }
        match found {
            Some((k, d)) => {
                fused[k] = true;
                worst = worst.max(d);
            }
            None => return rej(format!("face {f} has no image with the same outline, color, boundary kind and sides")),
        }
    }
    Verification::Verified { max_deviation: worst }
}
