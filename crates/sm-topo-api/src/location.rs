//! Locations (rigid/similarity placements) and oriented, located shape references: the
//! topology-level instancing mechanism (OCCT `TopLoc_Location` + `TopoDS_Shape`).

use crate::ids::{InstanceId, Sense};
use sm_core::{EntityId, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A placement of a shared definition in space. Improper placements (mirrors) are legal and
/// reverse the orientation of everything seen through them.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Location(pub Transform);

impl Location {
    /// The identity placement.
    pub const IDENTITY: Self = Self(Transform::IDENTITY);

    /// True for the exact identity.
    pub fn is_identity(&self) -> bool {
        self.0 == Transform::IDENTITY
    }

    /// True if the placement reverses orientation (`det < 0`).
    pub fn flips_orientation(&self) -> bool {
        self.0.is_improper()
    }

    /// Orientation of a definition's use after this placement: flipped when improper.
    pub fn orient(&self, s: Sense) -> Sense {
        if self.flips_orientation() { s.flip() } else { s }
    }

    /// `self ∘ inner`: apply `inner` first, then `self`.
    pub fn compose(&self, inner: &Self) -> Self {
        let (a, b) = (&self.0, &inner.0);
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = (0..3).map(|k| a.m[i][k] * b.m[k][j]).sum();
            }
        }
        let t = a.apply_vector(b.t) + a.t;
        Self(Transform { m, t })
    }

    /// Inverse placement, or `None` if singular.
    pub fn inverse(&self) -> Option<Self> {
        let m = &self.0.m;
        let det = self.0.det();
        if det.abs() < 1e-300 || !det.is_finite() {
            return None;
        }
        let c = |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
        // adjugate / det
        let inv = [
            [c(1, 1, 2, 2) / det, -c(0, 1, 2, 2) / det, c(0, 1, 1, 2) / det],
            [-c(1, 0, 2, 2) / det, c(0, 0, 2, 2) / det, -c(0, 0, 1, 2) / det],
            [c(1, 0, 2, 1) / det, -c(0, 0, 2, 1) / det, c(0, 0, 1, 1) / det],
        ];
        let lin = Transform { m: inv, t: Vector3::new(0.0, 0.0, 0.0) };
        let t = lin.apply_vector(self.0.t) * -1.0;
        Some(Self(Transform { m: inv, t }))
    }
}

/// A reference to a shared definition with an orientation and a location. The definition
/// (`id`) is never copied; N instances share it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ShapeRef {
    /// The shared definition (typically a body or a volume).
    pub id: EntityId,
    /// Orientation of this use relative to the definition.
    pub sense: Sense,
    /// Placement of this use.
    pub location: Location,
}

impl ShapeRef {
    /// Orientation after applying the location (a mirror reverses it).
    pub fn effective_sense(&self) -> Sense {
        self.location.orient(self.sense)
    }
}

/// An instance: a placed use of a shared definition. Nesting (lattices) is expressed by
/// `parent`: the effective placement is the composition up the chain.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct InstanceData {
    /// Id (`EntityKind::Instance`).
    pub id: InstanceId,
    /// What is placed.
    pub target: ShapeRef,
    /// Enclosing instance (e.g. the lattice cell), if any.
    pub parent: Option<InstanceId>,
}
