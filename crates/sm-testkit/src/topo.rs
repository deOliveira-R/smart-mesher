//! Synthetic B-rep volumes (through the `sm-topo-api` reference complex) and model fixtures
//! (through `sm-model-api`).
//!
//! The topology fixtures reuse `sm_topo_api::fixtures` (brick, cylinder, sphere, touching
//! bricks, webcut) and wrap each in a [`TopoFixture`] with the counts, Euler characteristic,
//! volumes and area that follow from the construction. [`pin_lattice_stub`] and
//! [`mirror_chamber`] add what the topology crate does not have: instancing and a mirror cut
//! with a deck face.

use core::f64::consts::PI;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_model_api::{
    BoundaryKind, CutSpec, Deck, Domain, Fill, InstanceRow, Keep, Lattice, Layout, Model, ModelError, Plane, Tolerance, plan_cut,
};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// A fixture could not be built.
#[derive(Debug, thiserror::Error)]
pub enum BuildError {
    /// A topology fixture failed.
    #[error(transparent)]
    Fixture(#[from] FixtureError),
    /// A model operation failed.
    #[error(transparent)]
    Model(#[from] ModelError),
}

/// A topology complex with the exact expectations of its construction.
#[derive(Clone, Debug)]
pub struct TopoFixture {
    /// Fixture name.
    pub name: &'static str,
    /// The complex.
    pub complex: CellComplex,
    /// Live cell counts `[vertices, edges, faces, volumes, bodies]`.
    pub counts: [usize; 5],
    /// `euler_characteristic` (`V - E + F`, pole edges excluded).
    pub euler: i64,
    /// Exact volume of each volume, ascending. For curved fixtures these are the closed forms;
    /// the reference complex integrates them by quadrature, so compare with a tolerance.
    pub volumes: Vec<f64>,
    /// Exact total boundary-face area (sum over all faces).
    pub area: f64,
}

fn run<T>(t: &mut CellComplex, op: u64, f: impl FnOnce(&mut CellComplex) -> Result<T, FixtureError>) -> Result<T, FixtureError> {
    with_op(t, OpId(op), f)
}

/// Axis-aligned brick `[min, min + size]`: counts `[8, 12, 6, 1, 1]`, Euler 2, volume
/// `sx sy sz`, area `2 (sx sy + sy sz + sx sz)`.
pub fn brick(min: [f64; 3], size: [f64; 3]) -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    run(&mut t, 1, |t| fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size))?;
    Ok(TopoFixture {
        name: "brick",
        complex: t,
        counts: [8, 12, 6, 1, 1],
        euler: 2,
        volumes: vec![size[0] * size[1] * size[2]],
        area: 2.0 * (size[0] * size[1] + size[1] * size[2] + size[0] * size[2]),
    })
}

/// The unit cube: [`brick`] with `min = 0`, `size = 1`: volume 1, area 6.
pub fn cube() -> Result<TopoFixture, FixtureError> {
    let mut f = brick([0.0; 3], [1.0; 3])?;
    f.name = "cube";
    Ok(f)
}

/// Solid cylinder about `z` from 0 to `h`: counts `[2, 3, 3, 1, 1]` (periodic seam used twice
/// by the lateral face), Euler 2, volume `pi r^2 h`, area `2 pi r (r + h)`.
pub fn cylinder(r: f64, h: f64) -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    run(&mut t, 1, |t| fixtures::cylinder(t, r, h))?;
    Ok(TopoFixture { name: "cylinder", complex: t, counts: [2, 3, 3, 1, 1], euler: 2, volumes: vec![PI * r * r * h], area: 2.0 * PI * r * (r + h) })
}

/// Solid sphere: counts `[2, 3, 1, 1, 1]` (poles, a seam and two degenerate pole edges), Euler
/// 2 (pole edges excluded), volume `4/3 pi r^3`, area `4 pi r^2`.
///
/// **Limit:** the faces touch the poles, so the reference complex answers
/// `face_moments`/`volume_moments` with an error; the closed forms are expectations for other
/// implementors and not checkable through the reference topology.
pub fn sphere(r: f64) -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    run(&mut t, 1, |t| fixtures::sphere(t, r))?;
    Ok(TopoFixture { name: "sphere", complex: t, counts: [2, 3, 1, 1, 1], euler: 2, volumes: vec![4.0 / 3.0 * PI * r * r * r], area: 4.0 * PI * r * r })
}

/// Two unit bricks `[0,1]^3` and `[1,2] x [0,1]^2` merged along the shared face: counts
/// `[12, 20, 11, 2, 2]`, Euler 3, volumes `[1, 1]`. The shared face exists once, so the 11 faces
/// (each of area 1) sum to 11. Edges on the junction have three faces: not a manifold surface.
pub fn merged_bricks() -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    let (a, b) = run(&mut t, 1, fixtures::two_bricks)?;
    run(&mut t, 2, |t| fixtures::merge_touching_bricks(t, &a, &b))?;
    Ok(TopoFixture { name: "merged_bricks", complex: t, counts: [12, 20, 11, 2, 2], euler: 3, volumes: vec![1.0, 1.0], area: 11.0 })
}

/// A unit cube webcut by the plane `z = frac`: counts `[12, 20, 11, 2, 1]`, Euler 3, volumes
/// `[min(frac, 1 - frac), max(..)]`, area 7 summed over the 11 faces (the cube's 6, split by the
/// chords, plus the cap of area 1).
pub fn webcut_cube(frac: f64) -> Result<TopoFixture, FixtureError> {
    let mut t = CellComplex::new();
    let b = run(&mut t, 1, |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
    run(&mut t, 2, |t| fixtures::webcut_brick(t, &b, frac))?;
    let mut volumes = vec![frac, 1.0 - frac];
    volumes.sort_by(f64::total_cmp);
    Ok(TopoFixture { name: "webcut_cube", complex: t, counts: [12, 20, 11, 2, 1], euler: 3, volumes, area: 7.0 })
}

/// A model with its topology.
#[derive(Clone, Debug)]
pub struct ModelFixture {
    /// Fixture name.
    pub name: &'static str,
    /// The topology.
    pub complex: CellComplex,
    /// The model tables.
    pub model: Model,
    /// Number of flattened instance paths (`0` if no instancing).
    pub flat_paths: usize,
    /// Number of flattened `(path, volume)` entries.
    pub flat_entries: usize,
    /// Number of faces that belong to a deck (mirror/translation/rotation) boundary.
    pub deck_faces: usize,
    /// Exact total volume of the base topology.
    pub volume: f64,
}

/// Put every boundary face that has no boundary record on one response boundary named
/// `outer-<label>`, and color those without a color with a fresh face color `label`.
pub fn finish_boundaries(m: &mut Model, t: &dyn TopologyView, label: u32) -> Result<(), ModelError> {
    let free: Vec<EntityId> = t.boundary_faces().filter(|f| m.face_boundary(*f).is_none()).collect();
    if free.is_empty() {
        return Ok(());
    }
    let b = m.add_boundary(&format!("outer-{label}"), BoundaryKind::Response)?;
    m.assign_response(b, &free)?;
    let c = m.add_color(label, &format!("bc-{label}"), Domain::Face)?;
    let uncolored: Vec<EntityId> = free.into_iter().filter(|f| m.face_color(*f).is_none()).collect();
    m.color_faces(c, &uncolored)
}

/// A 3x3 pin lattice stub: a pin of two stacked bricks (fuel `0.4 x 0.4 x 1`, clad on top), a
/// prototype of the two volumes, a Cartesian `3 x 3 x 1` lattice with pitch 1.26 and one
/// instance of the lattice at the origin. Flattening gives 9 paths and 18 entries.
pub fn pin_lattice_stub() -> Result<ModelFixture, BuildError> {
    let mut t = CellComplex::new();
    let fuel_v = run(&mut t, 1, |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [0.4, 0.4, 1.0]))?;
    let clad_v = run(&mut t, 2, |t| fixtures::brick(t, Point3::new(0.0, 0.0, 1.0), [0.4, 0.4, 1.0]))?;
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume)?;
    let clad = m.add_color(2, "clad", Domain::Volume)?;
    m.color_volumes(fuel, &[fuel_v.volume])?;
    m.color_volumes(clad, &[clad_v.volume])?;
    finish_boundaries(&mut m, &t, 7)?;
    let pin = m.add_prototype(&[clad_v.volume, fuel_v.volume])?;
    let lat = m.add_lattice(Lattice::new(Layout::Cartesian { n: [3, 3, 1], pitch: Vector3::new(1.26, 1.26, 0.0) }, Transform::IDENTITY, Fill::Prototype(pin), BTreeMap::new()))?;
    m.add_instance(InstanceRow::new(Fill::Lattice(lat), Transform::IDENTITY))?;
    Ok(ModelFixture { name: "pin_lattice_stub", complex: t, model: m, flat_paths: 9, flat_entries: 18, deck_faces: 0, volume: 2.0 * 0.4 * 0.4 })
}

/// A mirror-cut chamber: the unit cube is webcut at `z = 1/2`, the upper half discarded, and
/// the cut recorded in the model as a mirror symmetry with plane `z = 1/2` (keep the negative
/// side). The cap is the deck face (self-paired), colored with a face color. Remaining volume
/// 1/2; 1 deck face.
pub fn mirror_chamber() -> Result<ModelFixture, BuildError> {
    let mut t = CellComplex::new();
    let b = run(&mut t, 1, |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3]))?;
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume)?;
    m.color_volumes(mat, &[b.volume])?;
    finish_boundaries(&mut m, &t, 7)?;
    let cap_color = m.add_color(9, "cap", Domain::Face)?;
    run(&mut t, 2, |t| {
        let w = fixtures::webcut_brick(t, &b, 0.5)?;
        t.kill_cascade(w.volumes[1])?;
        Ok(())
    })?;
    let plane = Plane::new(Vector3::new(0.0, 0.0, 1.0), 0.5)?;
    let plan = plan_cut(&CutSpec::new(Deck::Mirror { plane }, Keep::Negative, Point3::new(0.0, 0.0, 0.5)))?;
    m.apply_cut(&plan, &t.lineage().cut_outcome(OpId(2)), Some(cap_color), &t)?;
    let deck_faces = m.deck_faces().count();
    Ok(ModelFixture { name: "mirror_chamber", complex: t, model: m, flat_paths: 0, flat_entries: 0, deck_faces, volume: 0.5 })
}

/// Validate a model fixture with the default tolerance; `Ok` when there is no error-level issue.
pub fn model_errors(f: &ModelFixture) -> Vec<String> {
    let r = sm_model_api::validate(&f.model, &f.complex, Tolerance::default());
    r.errors().map(|i| format!("{:?}: {}", i.code, i.detail)).collect()
}

/// All topology fixtures with default parameters.
pub fn all_topo_fixtures() -> Result<Vec<TopoFixture>, FixtureError> {
    Ok(vec![cube()?, brick([-1.0, 2.0, 0.5], [2.0, 3.0, 4.0])?, cylinder(1.0, 2.0)?, sphere(1.5)?, merged_bricks()?, webcut_cube(0.25)?])
}
