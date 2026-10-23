//! `proptest` strategies for random bricks, rigid and improper transforms and small hex blocks.
//!
//! Sizes are modest so that a test building a mesh per case stays fast: use `cases <= 64` for
//! anything that builds a complex or a mesh.

use crate::topo::{self, TopoFixture};
use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{Result as MeshResult, SerialAlloc};
use sm_topo_api::fixtures::FixtureError;

/// An axis-aligned brick `[min, min + size]` with `size > 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BrickSpec {
    /// Minimum corner.
    pub min: [f64; 3],
    /// Edge lengths (all in `[0.1, 10]`).
    pub size: [f64; 3],
}

impl BrickSpec {
    /// Exact volume.
    pub fn volume(&self) -> f64 {
        self.size[0] * self.size[1] * self.size[2]
    }

    /// Build the brick complex with its expectations.
    pub fn build(&self) -> Result<TopoFixture, FixtureError> {
        topo::brick(self.min, self.size)
    }
}

/// Random bricks: `min` in `[-10, 10]^3`, `size` in `[0.1, 10]^3`.
pub fn brick_spec() -> impl Strategy<Value = BrickSpec> {
    ([-10.0f64..10.0, -10.0f64..10.0, -10.0f64..10.0], [0.1f64..10.0, 0.1f64..10.0, 0.1f64..10.0]).prop_map(|(min, size)| BrickSpec { min, size })
}

fn vec3(range: core::ops::Range<f64>) -> impl Strategy<Value = Vector3> {
    (range.clone(), range.clone(), range).prop_map(|(x, y, z)| Vector3::new(x, y, z))
}

fn nonzero_vec3() -> impl Strategy<Value = Vector3> {
    vec3(-1.0..1.0).prop_filter("axis must not be near zero", |v| v.norm() > 0.1)
}

/// A random translation (components in `[-10, 10]`).
pub fn translation() -> impl Strategy<Value = Transform> {
    vec3(-10.0..10.0).prop_map(Transform::translation)
}

/// A random rotation about a random axis through a random point (angle in `[-pi, pi]`).
pub fn rotation() -> impl Strategy<Value = Transform> {
    (vec3(-5.0..5.0), nonzero_vec3(), -core::f64::consts::PI..core::f64::consts::PI)
        .prop_filter_map("rotation needs a non-zero axis", |(p, axis, angle)| Transform::rotation(Point3::new(p.x, p.y, p.z), axis, angle))
}

/// A random reflection in a plane (improper, `det = -1`).
pub fn reflection() -> impl Strategy<Value = Transform> {
    (nonzero_vec3(), -5.0f64..5.0).prop_filter_map("reflection needs a non-zero normal", |(n, d)| Transform::reflection(n, d))
}

/// A random composition of one to three rigid motions (translations and rotations): `det = +1`.
pub fn proper_transform() -> impl Strategy<Value = Transform> {
    prop::collection::vec(prop_oneof![translation(), rotation()], 1..=3).prop_map(|v| v.iter().fold(Transform::IDENTITY, |acc, t| acc.compose(t)))
}

/// A random composition of one to three transforms of any kind (reflections included).
pub fn any_transform() -> impl Strategy<Value = Transform> {
    prop::collection::vec(prop_oneof![translation(), rotation(), reflection()], 1..=3).prop_map(|v| v.iter().fold(Transform::IDENTITY, |acc, t| acc.compose(t)))
}

/// A small hex block: `n` cells per axis (1 to 3) and cell sizes `h` in `[0.1, 3]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HexBlockSpec {
    /// Cells per axis.
    pub n: [u32; 3],
    /// Cell sizes.
    pub h: [f64; 3],
}

impl HexBlockSpec {
    /// Exact volume `prod(n h)`.
    pub fn volume(&self) -> f64 {
        (0..3).map(|a| f64::from(self.n[a]) * self.h[a]).product()
    }

    /// Number of cells.
    pub fn cells(&self) -> usize {
        self.n.iter().map(|&n| n as usize).product()
    }

    /// Build the mesh (a fresh id allocator, so ids are reproducible).
    pub fn build(&self) -> MeshResult<ReferenceMesh> {
        hex_block(self.n, self.h, &mut SerialAlloc::new())
    }
}

/// Random small hex blocks.
pub fn hex_block_spec() -> impl Strategy<Value = HexBlockSpec> {
    ([1u32..=3, 1u32..=3, 1u32..=3], [0.1f64..3.0, 0.1f64..3.0, 0.1f64..3.0]).prop_map(|(n, h)| HexBlockSpec { n, h })
}
