//! Small geometric value types and moment algebra used across the crate: tolerances, planes,
//! axes, rational turns, moment transforms, spatial bins and the per-operation geometry cache.

use crate::error::{ModelError, Result};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_topo_api::{FaceMoments, Moments, TopologyView};
use std::collections::BTreeMap;

/// Fixed internal tolerance for identifying elements of a deck group, independent of the user
/// tolerance (group closure must be stable).
pub const GROUP_TOL: f64 = 1e-9;

/// Geometric tolerances used by comparisons in this crate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Tolerance {
    /// Absolute distance tolerance in model units.
    pub linear: f64,
    /// Absolute tolerance on dimensionless quantities (matrix entries, direction cosines).
    pub angular: f64,
    /// Relative tolerance on integrated quantities, scaled by the model's length scale.
    pub relative: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { linear: 1e-8, angular: 1e-9, relative: 1e-9 }
    }
}

/// An oriented plane `{x : normal . x = offset}` with a unit normal.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Plane {
    /// Unit normal.
    pub normal: Vector3,
    /// Signed offset along the normal.
    pub offset: f64,
}

impl Plane {
    /// Build a plane from any non-zero normal (normalized for you; the offset is rescaled).
    pub fn new(normal: Vector3, offset: f64) -> Result<Self> {
        let l = normal.norm();
        if !(l.is_finite() && l > 1e-300) {
            return Err(ModelError::Invalid("plane normal is zero".into()));
        }
        Ok(Self { normal: normal * (1.0 / l), offset: offset / l })
    }

    /// The plane through `p` with normal `n`.
    pub fn through(p: Point3, n: Vector3) -> Result<Self> {
        let off = n.dot(vec_of(p));
        Self::new(n, off)
    }

    /// Signed distance of `p` from the plane (positive on the normal side).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        self.normal.dot(vec_of(p)) - self.offset
    }

    /// The same plane with the opposite orientation.
    pub fn flipped(&self) -> Self {
        Self { normal: -self.normal, offset: -self.offset }
    }

    /// The reflection in this plane.
    pub fn reflection(&self) -> Transform {
        Transform::reflection(self.normal, self.offset).unwrap_or(Transform::IDENTITY)
    }
}

/// A directed axis (unit direction) for rotations.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Axis {
    /// A point on the axis.
    pub point: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

impl Axis {
    /// Build an axis; the direction is normalized.
    pub fn new(point: Point3, dir: Vector3) -> Result<Self> {
        let l = dir.norm();
        if !(l.is_finite() && l > 1e-300) {
            return Err(ModelError::Invalid("axis direction is zero".into()));
        }
        Ok(Self { point, dir: dir * (1.0 / l) })
    }

    /// Distance of `p` from the axis line.
    pub fn distance(&self, p: Point3) -> f64 {
        (p - self.point).cross(self.dir).norm()
    }
}

/// A rational number of full turns, `angle = 2 pi num / den`, always reduced with `den >= 1` and
/// `0 <= num < den`. Exact order and `k * 2 pi / n` are expressible.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Turns {
    /// Numerator (reduced).
    pub num: i32,
    /// Denominator (reduced, at least 1).
    pub den: u32,
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

impl Turns {
    /// `num/den` of a turn, reduced and wrapped into `[0, 1)`.
    pub fn new(num: i32, den: u32) -> Result<Self> {
        if den == 0 {
            return Err(ModelError::Invalid("turns denominator is zero".into()));
        }
        let d = i64::from(den);
        let n = i64::from(num).rem_euclid(d);
        let g = gcd(n as u64, d as u64).max(1) as i64;
        Ok(Self { num: (n / g) as i32, den: (d / g) as u32 })
    }

    /// `1/order` of a turn.
    pub fn one_over(order: u32) -> Result<Self> {
        Self::new(1, order)
    }

    /// The angle in radians.
    pub fn radians(&self) -> f64 {
        core::f64::consts::TAU * f64::from(self.num) / f64::from(self.den)
    }

    /// Order of the generated cyclic group (`den`); 1 means the identity.
    pub fn order(&self) -> u32 {
        self.den
    }

    /// True for the identity rotation.
    pub fn is_zero(&self) -> bool {
        self.num == 0
    }

    /// True if this is a proper reduced fraction `0 < num < den` with `den <= max_den` (the
    /// shape every non-trivial rotation deck must have). Fields are public, so this guards
    /// hand-built values.
    pub fn is_proper(&self, max_den: u32) -> bool {
        self.num > 0 && i64::from(self.num) < i64::from(self.den) && self.den <= max_den && gcd(self.num as u64, u64::from(self.den)) == 1
    }
}

pub(crate) type Mat = [[f64; 3]; 3];

pub(crate) fn mat_mul(a: &Mat, b: &Mat) -> Mat {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    m
}

pub(crate) fn transpose(a: &Mat) -> Mat {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = a[j][i];
        }
    }
    m
}

pub(crate) fn outer(a: Vector3, b: Vector3) -> Mat {
    let (a, b) = ([a.x, a.y, a.z], [b.x, b.y, b.z]);
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e = a[i] * b[j];
        }
    }
    m
}

pub(crate) fn mat_add(a: &Mat, b: &Mat) -> Mat {
    let mut m = *a;
    for (i, row) in m.iter_mut().enumerate() {
        for (j, e) in row.iter_mut().enumerate() {
            *e += b[i][j];
        }
    }
    m
}

pub(crate) fn mat_scale(a: &Mat, s: f64) -> Mat {
    let mut m = *a;
    m.iter_mut().flatten().for_each(|e| *e *= s);
    m
}

pub(crate) fn pt(v: Vector3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

pub(crate) fn vec_of(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// Second moment after applying `x`: `R S R^T + (R f) t^T + t (R f)^T + A t t^T`.
fn second_under(second: &Mat, first: Vector3, measure: f64, x: &Transform) -> Mat {
    let rf = x.apply_vector(first);
    let rs = mat_mul(&x.m, &mat_mul(second, &transpose(&x.m)));
    let cross = mat_add(&outer(rf, x.t), &outer(x.t, rf));
    mat_add(&mat_add(&rs, &cross), &mat_scale(&outer(x.t, x.t), measure))
}

/// Volume moments of the image of a region under the rigid transform `x`.
pub fn transform_moments(m: &Moments, x: &Transform) -> Moments {
    Moments { volume: m.volume, first: x.apply_vector(m.first) + x.t * m.volume, second: second_under(&m.second, m.first, m.volume, x) }
}

/// Face moments of the geometric image of a face under the rigid transform `x`. The vector area
/// is the image of the vector area (`R n`): it follows the geometric outward direction, whether or
/// not `x` is improper. (The image face's loop must be reversed under an improper `x` for its
/// stored normal to follow the right-hand rule again; see `UnfoldedModel::sense`.)
pub fn transform_face_moments(m: &FaceMoments, x: &Transform) -> FaceMoments {
    FaceMoments { area: m.area, first: x.apply_vector(m.first) + x.t * m.area, second: second_under(&m.second, m.first, m.area, x), vector_area: x.apply_vector(m.vector_area) }
}

/// Compare face moments (area, first, second); the vector area is checked separately.
pub(crate) fn face_shape_close(a: &FaceMoments, b: &FaceMoments, l: f64, tol: &Tolerance) -> bool {
    let eps = tol.relative + tol.linear / l.max(1e-300);
    let (l2, l3, l4) = (l * l, l * l * l, l * l * l * l);
    if (a.area - b.area).abs() > eps * l2 {
        return false;
    }
    let f = a.first - b.first;
    if f.x.abs().max(f.y.abs()).max(f.z.abs()) > eps * l3 {
        return false;
    }
    (0..3).all(|i| (0..3).all(|j| (a.second[i][j] - b.second[i][j]).abs() <= eps * l4))
}

/// `+1` if vector areas agree, `-1` if opposed, `0` otherwise (relative to the area).
pub(crate) fn vector_area_sign(a: Vector3, b: Vector3, area: f64, tol: &Tolerance) -> i8 {
    let lim = (tol.relative + tol.angular) * area.max(1e-300) + tol.linear * tol.linear;
    if (a - b).norm() <= lim {
        1
    } else if (a + b).norm() <= lim {
        -1
    } else {
        0
    }
}

pub(crate) fn volume_close(a: &Moments, b: &Moments, l: f64, tol: &Tolerance) -> bool {
    let eps = tol.relative + tol.linear / l.max(1e-300);
    let (l3, l4, l5) = (l * l * l, l * l * l * l, l * l * l * l * l);
    if (a.volume - b.volume).abs() > eps * l3 {
        return false;
    }
    let f = a.first - b.first;
    if f.x.abs().max(f.y.abs()).max(f.z.abs()) > eps * l4 {
        return false;
    }
    (0..3).all(|i| (0..3).all(|j| (a.second[i][j] - b.second[i][j]).abs() <= eps * l5))
}

/// Per-operation cache of moments and the model length scale.
pub(crate) struct GeomCache<'a> {
    pub(crate) t: &'a dyn TopologyView,
    faces: BTreeMap<EntityId, Option<FaceMoments>>,
    vols: BTreeMap<EntityId, Option<Moments>>,
    scale: Option<f64>,
}

impl<'a> GeomCache<'a> {
    pub(crate) fn new(t: &'a dyn TopologyView) -> Self {
        Self { t, faces: BTreeMap::new(), vols: BTreeMap::new(), scale: None }
    }

    pub(crate) fn face(&mut self, f: EntityId) -> Option<FaceMoments> {
        let t = self.t;
        *self.faces.entry(f).or_insert_with(|| t.face_moments(f).ok())
    }

    pub(crate) fn volume(&mut self, v: EntityId) -> Option<Moments> {
        let t = self.t;
        *self.vols.entry(v).or_insert_with(|| t.volume_moments(v).ok())
    }

    /// Like [`GeomCache::face`], with the topology's error when moments are unavailable.
    pub(crate) fn face_res(&mut self, f: EntityId) -> Result<FaceMoments> {
        match self.face(f) {
            Some(m) => Ok(m),
            None => Err(self.t.face_moments(f).err().map_or_else(|| ModelError::Invalid(format!("no moments for face {f}")), ModelError::from)),
        }
    }

    /// Like [`GeomCache::volume`], with the topology's error when moments are unavailable.
    pub(crate) fn volume_res(&mut self, v: EntityId) -> Result<Moments> {
        match self.volume(v) {
            Some(m) => Ok(m),
            None => Err(self.t.volume_moments(v).err().map_or_else(|| ModelError::Invalid(format!("no moments for volume {v}")), ModelError::from)),
        }
    }

    /// Characteristic model length: cube root of the total volume (1 if unavailable).
    pub(crate) fn length_scale(&mut self) -> f64 {
        if let Some(s) = self.scale {
            return s;
        }
        let vols: Vec<EntityId> = self.t.cells(sm_topo_api::Dim::Volume).to_vec();
        let total: f64 = vols.iter().filter_map(|v| self.volume(*v)).map(|m| m.volume.abs()).sum();
        let s = if total > 0.0 && total.is_finite() { total.cbrt() } else { 1.0 };
        self.scale = Some(s);
        s
    }
}

/// Uniform spatial hash for approximate point matching: O(1) expected per query.
pub(crate) struct Bins {
    h: f64,
    cells: BTreeMap<[i64; 3], Vec<usize>>,
}

impl Bins {
    pub(crate) fn new(h: f64) -> Self {
        Self { h: h.max(1e-12), cells: BTreeMap::new() }
    }

    fn key(&self, p: Point3) -> [i64; 3] {
        [(p.x / self.h).floor() as i64, (p.y / self.h).floor() as i64, (p.z / self.h).floor() as i64]
    }

    pub(crate) fn insert(&mut self, p: Point3, idx: usize) {
        let k = self.key(p);
        self.cells.entry(k).or_default().push(idx);
    }

    /// Indices within the 27 cells around `p`, ascending.
    pub(crate) fn near(&self, p: Point3) -> Vec<usize> {
        let k = self.key(p);
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(v) = self.cells.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                        out.extend_from_slice(v);
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// True if two transforms agree entry by entry within `tol`.
pub fn transforms_close(a: &Transform, b: &Transform, tol: f64) -> bool {
    (0..3).all(|i| (0..3).all(|j| (a.m[i][j] - b.m[i][j]).abs() <= tol)) && (a.t - b.t).norm() <= tol
}

/// Grid shift in cells: puts the common values 0, +-1/2, +-1 away from cell borders, so the
/// neighbour probe is rarely needed.
const GRID_SHIFT: f64 = 0.37;

fn transform_cells(x: &Transform, q: f64) -> [f64; 12] {
    let mut v = [0.0; 12];
    for i in 0..3 {
        for j in 0..3 {
            v[3 * i + j] = x.m[i][j] / q + GRID_SHIFT;
        }
    }
    v[9] = x.t.x / q + GRID_SHIFT;
    v[10] = x.t.y / q + GRID_SHIFT;
    v[11] = x.t.z / q + GRID_SHIFT;
    v
}

/// Transforms keyed by quantized entries, with a neighbour-cell probe so entries that straddle a
/// cell boundary are still found. O(log n) per lookup.
#[derive(Clone, Debug, Default)]
pub(crate) struct TransformIndex {
    map: BTreeMap<[i64; 12], Vec<u32>>,
}

const TQ: f64 = 1.234_567_8e-6;

impl TransformIndex {
    pub(crate) fn insert(&mut self, x: &Transform, id: u32) {
        let c = transform_cells(x, TQ);
        let mut k = [0i64; 12];
        for (a, b) in k.iter_mut().zip(c) {
            *a = b.floor() as i64;
        }
        self.map.entry(k).or_default().push(id);
    }

    /// Candidate ids in the home cell and the neighbours of borderline coordinates.
    pub(crate) fn candidates(&self, x: &Transform) -> Vec<u32> {
        let c = transform_cells(x, TQ);
        let mut bases: Vec<[i64; 12]> = vec![[0; 12]];
        for (i, v) in c.iter().enumerate() {
            let f = v.floor();
            let frac = v - f;
            let mut alts = vec![f as i64];
            if frac < 1e-3 {
                alts.push(f as i64 - 1);
            } else if frac > 1.0 - 1e-3 {
                alts.push(f as i64 + 1);
            }
            let mut next = Vec::with_capacity(bases.len() * alts.len());
            for b in &bases {
                for a in &alts {
                    let mut nb = *b;
                    nb[i] = *a;
                    next.push(nb);
                }
            }
            // At most 2^12 combinations; every borderline coordinate is probed.
            bases = next;
        }
        let mut out = Vec::new();
        for b in bases {
            if let Some(v) = self.map.get(&b) {
                out.extend_from_slice(v);
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Eigenvalues (ascending) and unit eigenvectors of a symmetric 3x3 matrix, by cyclic Jacobi
/// (deterministic).
pub(crate) fn sym_eigen(a: &Mat) -> ([f64; 3], [Vector3; 3]) {
    let mut a = *a;
    let mut v: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let diag = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-15 * diag.max(1e-300) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() <= 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = if theta == 0.0 { 1.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
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
    (idx.map(|c| a[c][c]), idx.map(|c| Vector3::new(v[0][c], v[1][c], v[2][c])))
}
