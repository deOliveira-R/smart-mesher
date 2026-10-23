//! The narrow topology interface this layer needs. **This list is an input to the sm-topo-api
//! design**: every method below is used by something in this crate, and nothing else is.
//!
//! Two traits, deliberately separated:
//! - [`TopologyView`]: pure incidence (3 methods required, others provided).
//! - [`GeometryView`]: shape-layer measures of topology entities (4 methods).
//!
//! Who uses what:
//!
//! | Query | Used by |
//! |---|---|
//! | `volumes`, `faces` | validator (coverage), decomposition identity, detection |
//! | `face_sides` | boundary-face discovery (kind required), interface checks, cover/glue |
//! | `volume_faces` (provided from `faces`+`face_sides`) | decomposition, group/block expansion |
//! | `face_centroid`, `face_area`, `face_normal` | deck-pair validation, `faces_on_plane`, detection verification |
//! | `volume_moments` | detection (barycentre + inertia), decomposition measure check, cut verification |

use crate::xf::{Plane, Tolerance};
use sm_core::{EntityId, Point3, Transform, Vector3};

/// The two sides of a topological face. The face normal (from its vertex loop, right-hand rule)
/// points from `fwd` to `rev`. A **boundary face** has `rev == None` and its normal points outward
/// from `fwd`. An interior face has both. Sides may be equal (a face seen from both sides by the
/// same volume).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FaceSides {
    /// Volume the normal points out of.
    pub fwd: Option<EntityId>,
    /// Volume the normal points into (`None` on boundary faces).
    pub rev: Option<EntityId>,
}

impl FaceSides {
    /// True if exactly one side exists.
    pub fn is_boundary(&self) -> bool {
        self.fwd.is_some() != self.rev.is_some()
    }
    /// The sole volume of a boundary face (or `None` for interior/orphan faces).
    pub fn sole(&self) -> Option<EntityId> {
        match (self.fwd, self.rev) {
            (Some(v), None) | (None, Some(v)) => Some(v),
            _ => None,
        }
    }
}

/// Volume integrals: measure, centroid and second moment about the **origin**
/// (`second = ∫ x xᵀ dV`). Enough to derive barycentres and inertia tensors, and closed under
/// rigid transforms ([`Moments::transformed`]).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Moments {
    /// Volume (or area for 2D models).
    pub measure: f64,
    /// Barycentre.
    pub centroid: Point3,
    /// `∫ x xᵀ dV` about the origin.
    pub second: [[f64; 3]; 3],
}

impl Moments {
    /// Zero moments.
    pub const ZERO: Self = Self { measure: 0.0, centroid: Point3::new(0.0, 0.0, 0.0), second: [[0.0; 3]; 3] };

    /// Moments of the image under a rigid (isometric) transform. Improper transforms are fine.
    pub fn transformed(&self, x: &Transform) -> Self {
        let c = self.centroid;
        let cv = Vector3::new(c.x, c.y, c.z);
        let mc = x.apply_vector(cv); // M c
        let t = x.t;
        let mm = self.measure;
        let mut s = [[0.0; 3]; 3];
        let (mcv, tv) = ([mc.x, mc.y, mc.z], [t.x, t.y, t.z]);
        for (i, row) in s.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                let mut acc = 0.0;
                for k in 0..3 {
                    for l in 0..3 {
                        acc += x.m[i][k] * self.second[k][l] * x.m[j][l];
                    }
                }
                *v = acc + mm * (mcv[i] * tv[j] + tv[i] * mcv[j] + tv[i] * tv[j]);
            }
        }
        Self { measure: mm, centroid: x.apply_point(c), second: s }
    }

    /// Sum of moments (union of disjoint regions).
    pub fn add(&self, o: &Self) -> Self {
        let m = self.measure + o.measure;
        let mut s = [[0.0; 3]; 3];
        for (i, row) in s.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v = self.second[i][j] + o.second[i][j];
            }
        }
        let c = if m.abs() > 0.0 {
            Point3::new(
                (self.centroid.x * self.measure + o.centroid.x * o.measure) / m,
                (self.centroid.y * self.measure + o.centroid.y * o.measure) / m,
                (self.centroid.z * self.measure + o.centroid.z * o.measure) / m,
            )
        } else {
            self.centroid
        };
        Self { measure: m, centroid: c, second: s }
    }

    /// Second moment about point `p` (central inertia-like matrix `∫ (x-p)(x-p)ᵀ dV`).
    pub fn second_about(&self, p: Point3) -> [[f64; 3]; 3] {
        let (c, pv) = ([self.centroid.x, self.centroid.y, self.centroid.z], [p.x, p.y, p.z]);
        let m = self.measure;
        let mut s = self.second;
        for (i, row) in s.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v += -m * (c[i] * pv[j] + pv[i] * c[j]) + m * pv[i] * pv[j];
            }
        }
        s
    }

    /// Tolerant comparison of all fields.
    pub fn approx_eq(&self, o: &Self, tol: &Tolerance) -> bool {
        let cs = 1.0 + Vector3::new(self.centroid.x, self.centroid.y, self.centroid.z).norm();
        if !tol.close(self.measure, o.measure) || (self.centroid - o.centroid).norm() > tol.linear + tol.relative * cs {
            return false;
        }
        // Second moments scale like measure * length^2; compare against the largest entry.
        let mut big = 0.0f64;
        for row in &self.second {
            for v in row {
                big = big.max(v.abs());
            }
        }
        let atol = tol.linear + tol.relative * big;
        for i in 0..3 {
            for j in 0..3 {
                if (self.second[i][j] - o.second[i][j]).abs() > atol {
                    return false;
                }
            }
        }
        true
    }
}

/// Pure incidence queries (the part sm-topo-api must provide to every consumer here).
///
/// Ordering contract: `volumes()` and `faces()` return ids sorted ascending and without
/// duplicates, so iteration is deterministic.
pub trait TopologyView {
    /// All volume ids, sorted ascending.
    fn volumes(&self) -> Vec<EntityId>;
    /// All face ids, sorted ascending.
    fn faces(&self) -> Vec<EntityId>;
    /// The sides of a face, or `None` if `face` is unknown.
    fn face_sides(&self, face: EntityId) -> Option<FaceSides>;

    /// Faces bounding `volume`, sorted ascending. The default scans all faces; implementors with
    /// an index should override.
    fn volume_faces(&self, volume: EntityId) -> Vec<EntityId> {
        self.faces()
            .into_iter()
            .filter(|&f| self.face_sides(f).is_some_and(|s| s.fwd == Some(volume) || s.rev == Some(volume)))
            .collect()
    }

    /// Boundary faces (exactly one side), sorted ascending.
    fn boundary_faces(&self) -> Vec<EntityId> {
        self.faces().into_iter().filter(|&f| self.face_sides(f).is_some_and(|s| s.is_boundary())).collect()
    }
}

/// Shape-layer measures of topology entities. Kept apart from [`TopologyView`] because they need
/// the geometry kernel, not just incidence.
pub trait GeometryView: TopologyView {
    /// Area-weighted centroid of a face.
    fn face_centroid(&self, face: EntityId) -> Option<Point3>;
    /// Area of a face.
    fn face_area(&self, face: EntityId) -> Option<f64>;
    /// Unit normal at the face centroid, consistent with the [`FaceSides`] convention (outward on
    /// boundary faces). `None` for unknown faces. Non-planar faces return the normal at the centroid.
    fn face_normal(&self, face: EntityId) -> Option<Vector3>;
    /// Volume moments (measure, barycentre, second moment about the origin).
    fn volume_moments(&self, volume: EntityId) -> Option<Moments>;
}

/// Boundary faces lying on `plane`: centroid within `tol.linear`, normal parallel to the plane
/// normal within `tol.angular`. Used to tag pre-cut imported geometry ("tag existing faces as deck
/// boundaries"). Result sorted ascending.
pub fn faces_on_plane(topo: &dyn GeometryView, plane: &Plane, tol: &Tolerance) -> Vec<EntityId> {
    topo.boundary_faces()
        .into_iter()
        .filter(|&f| {
            let (Some(c), Some(n)) = (topo.face_centroid(f), topo.face_normal(f)) else { return false };
            plane.signed_distance(c).abs() <= tol.linear && (n.cross(plane.normal())).norm() <= tol.angular
        })
        .collect()
}
