//! Instanced model fixtures that extend `pin_lattice_stub` with curved pins: a fuel cylinder
//! inside a cladding tube sharing one cylindrical interface face.

use super::{BuildError, ModelFixture, finish_boundaries, run};
use core::f64::consts::{PI, TAU};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::{Circle, Cylinder, CurveDef, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Uv};
use sm_model_api::{Cluster, ClusterMember, Domain, Fill, InstanceRow, Lattice, Layout, Model};
use sm_topo_api::fixtures::FixtureError;
use sm_topo_api::{CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier, TopoError, TopologyEdit};
use std::collections::BTreeMap;

const F: Sense = Sense::Forward;
const R: Sense = Sense::Reversed;

fn bad(msg: &'static str) -> BuildError {
    BuildError::Fixture(FixtureError::Topo(TopoError::InvalidParameter(msg)))
}

fn lp(c: &[(EntityId, Sense)]) -> Vec<CoedgeSpec> {
    c.iter().map(|(edge, sense)| CoedgeSpec { edge: *edge, sense: *sense }).collect()
}

/// The outer boundary of the cladding.
#[derive(Clone, Copy)]
enum Outer {
    /// A cylinder of this radius.
    Round(f64),
    /// A square tube with this half width.
    Square(f64),
}

/// The two volumes of a pin cell.
struct PinCell {
    fuel: EntityId,
    clad: EntityId,
}

fn plane(t: &mut CellComplex, origin: Point3, normal: Vector3) -> Result<EntityId, FixtureError> {
    Ok(t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame: Frame::from_z(origin, normal)? }))?)
}

fn face(t: &mut CellComplex, loops: &[&[CoedgeSpec]], surface: EntityId, sense: Sense) -> Result<EntityId, FixtureError> {
    Ok(t.make_face(loops, Some(SurfaceCarrier { surface, sense }))?)
}

struct Round {
    bot: EntityId,
    top: EntityId,
    seam: EntityId,
    surface: EntityId,
}

/// Circles at `z = 0` and `z = h`, the seam, and the cylinder surface of radius `r`.
fn round_parts(t: &mut CellComplex, r: f64, h: f64) -> Result<Round, FixtureError> {
    let z = Vector3::new(0.0, 0.0, 1.0);
    let x = Vector3::new(1.0, 0.0, 0.0);
    let vb = t.make_vertex(Point3::new(r, 0.0, 0.0))?;
    let vt = t.make_vertex(Point3::new(r, 0.0, h))?;
    let full = Interval::new(0.0, TAU);
    let mut circles = Vec::new();
    for (v, zz) in [(vb, 0.0), (vt, h)] {
        let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(0.0, 0.0, zz), z, x)?, r)?))?;
        circles.push(t.make_edge(v, v, Some(CurveCarrier { curve, range: full, sense: F }))?);
    }
    let line = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(Point3::new(r, 0.0, 0.0), z)?))?;
    let seam = t.make_edge(vb, vt, Some(CurveCarrier { curve: line, range: Interval::new(0.0, h), sense: F }))?;
    let surface = t.shapes_mut()?.add_surface(SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?;
    Ok(Round { bot: circles[0], top: circles[1], seam, surface })
}

/// The cylinder face with outward normal over `p`, with its seam pcurves.
fn lateral(t: &mut CellComplex, p: &Round, h: f64) -> Result<EntityId, FixtureError> {
    let l = lp(&[(p.bot, F), (p.seam, F), (p.top, R), (p.seam, R)]);
    let f = face(t, &[&l], p.surface, F)?;
    let full = Interval::new(0.0, TAU);
    let hr = Interval::new(0.0, h);
    let line2 = |o: (f64, f64), d: (f64, f64), range| PCurve::Line2(Line2 { origin: Uv::new(o.0, o.1), dir: Uv::new(d.0, d.1), range });
    let key = |edge, side| CoedgeKey { face: f, edge, side };
    t.set_pcurve(key(p.bot, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
    t.set_pcurve(key(p.seam, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), hr)))?;
    t.set_pcurve(key(p.top, Side::Right), Some(line2((0.0, h), (1.0, 0.0), full)))?;
    t.set_pcurve(key(p.seam, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), hr)))?;
    Ok(f)
}

fn pin_cell(t: &mut CellComplex, r_fuel: f64, outer: Outer, h: f64) -> Result<PinCell, FixtureError> {
    let z = Vector3::new(0.0, 0.0, 1.0);
    let o = Point3::new(0.0, 0.0, 0.0);
    let oh = Point3::new(0.0, 0.0, h);
    let fuel = round_parts(t, r_fuel, h)?;
    let (s_bot, s_top) = (plane(t, o, -z)?, plane(t, oh, z)?);
    let f_bot = face(t, &[&lp(&[(fuel.bot, R)])], s_bot, F)?;
    let f_top = face(t, &[&lp(&[(fuel.top, F)])], s_top, F)?;
    let f_lat = lateral(t, &fuel, h)?;
    let hole_bot = lp(&[(fuel.bot, F)]);
    let hole_top = lp(&[(fuel.top, R)]);
    let mut clad_faces = Vec::new();
    match outer {
        Outer::Round(rc) => {
            let c = round_parts(t, rc, h)?;
            clad_faces.push(face(t, &[&lp(&[(c.bot, R)]), &hole_bot], s_bot, F)?);
            clad_faces.push(face(t, &[&lp(&[(c.top, F)]), &hole_top], s_top, F)?);
            clad_faces.push(lateral(t, &c, h)?);
        }
        Outer::Square(a) => {
            let sq = [(-a, -a), (a, -a), (a, a), (-a, a)];
            let pb: Vec<Point3> = sq.iter().map(|p| Point3::new(p.0, p.1, 0.0)).collect();
            let pt: Vec<Point3> = sq.iter().map(|p| Point3::new(p.0, p.1, h)).collect();
            let vb: Vec<EntityId> = pb.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
            let vt: Vec<EntityId> = pt.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
            let edge = |t: &mut CellComplex, a: (EntityId, Point3), b: (EntityId, Point3)| -> Result<EntityId, FixtureError> {
                let d = b.1 - a.1;
                let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(a.1, d)?))?;
                Ok(t.make_edge(a.0, b.0, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: F }))?)
            };
            let (mut eb, mut et, mut ec) = (Vec::new(), Vec::new(), Vec::new());
            for i in 0..4 {
                let j = (i + 1) % 4;
                eb.push(edge(t, (vb[i], pb[i]), (vb[j], pb[j]))?);
                et.push(edge(t, (vt[i], pt[i]), (vt[j], pt[j]))?);
                ec.push(edge(t, (vb[i], pb[i]), (vt[i], pt[i]))?);
            }
            let bot = lp(&(0..4).rev().map(|i| (eb[i], R)).collect::<Vec<_>>());
            let top = lp(&(0..4).map(|i| (et[i], F)).collect::<Vec<_>>());
            clad_faces.push(face(t, &[&bot, &hole_bot], s_bot, F)?);
            clad_faces.push(face(t, &[&top, &hole_top], s_top, F)?);
            for i in 0..4 {
                let j = (i + 1) % 4;
                let d = pb[j] - pb[i];
                let s = plane(t, pb[i], Vector3::new(d.y, -d.x, 0.0))?;
                clad_faces.push(face(t, &[&lp(&[(eb[i], F), (ec[j], F), (et[i], R), (ec[i], R)])], s, F)?);
            }
        }
    }
    let left = |fs: &[EntityId]| -> Vec<FaceUseSpec> { fs.iter().map(|f| FaceUseSpec { face: *f, side: Side::Left }).collect() };
    let fuel_uses = left(&[f_bot, f_top, f_lat]);
    let fuel_v = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &fuel_uses }])?;
    let mut clad_uses = left(&clad_faces);
    clad_uses.push(FaceUseSpec { face: f_lat, side: Side::Right });
    let clad_v = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &clad_uses }])?;
    t.make_body(&[fuel_v, clad_v])?;
    Ok(PinCell { fuel: fuel_v, clad: clad_v })
}

/// Colors, boundaries and the pin prototype `[clad, fuel]` of a pin cell.
fn pin_model(t: &CellComplex, cell: &PinCell) -> Result<(Model, EntityId), BuildError> {
    let mut m = Model::new();
    let fuel = m.add_color(1, "fuel", Domain::Volume)?;
    let clad = m.add_color(2, "clad", Domain::Volume)?;
    m.color_volumes(fuel, &[cell.fuel])?;
    m.color_volumes(clad, &[cell.clad])?;
    finish_boundaries(&mut m, t, 7)?;
    let proto = m.add_prototype(&[cell.clad, cell.fuel])?;
    Ok((m, proto))
}

/// Closed-form totals of a [`pin_assembly`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinAssemblyExpect {
    /// Number of pins: `9 n^2`.
    pub pins: usize,
    /// Volume of one fuel cylinder, `pi r_fuel^2 h`.
    pub fuel_volume: f64,
    /// Volume of one cladding tube, `pi (r_clad^2 - r_fuel^2) h`.
    pub clad_volume: f64,
    /// Total fuel volume over all pins.
    pub total_fuel: f64,
    /// Total cladding volume over all pins.
    pub total_clad: f64,
}

/// The expectations of `pin_assembly(n, _, r_fuel, r_clad, h)`.
pub fn pin_assembly_expect(n: usize, r_fuel: f64, r_clad: f64, h: f64) -> PinAssemblyExpect {
    let pins = 9 * n * n;
    let fuel_volume = PI * r_fuel * r_fuel * h;
    let clad_volume = PI * (r_clad * r_clad - r_fuel * r_fuel) * h;
    PinAssemblyExpect { pins, fuel_volume, clad_volume, total_fuel: pins as f64 * fuel_volume, total_clad: pins as f64 * clad_volume }
}

/// A pin assembly with three instancing levels. The base topology is one pin: a fuel cylinder of
/// radius `r_fuel` and a cladding tube `r_fuel..r_clad` (sharing the cylindrical interface face),
/// both of height `h`, colored `fuel` and `clad`. Level 1 is the pin prototype, level 2 an
/// `n x n x 1` Cartesian lattice of pins with `pitch`, level 3 a `3 x 3` [`Cluster`] of such
/// lattices at offsets `n * pitch`, placed by one instance. Flattening gives `9 n^2` paths and
/// `18 n^2` entries; see [`pin_assembly_expect`] for the volumes. `volume` is the base pin,
/// `pi r_clad^2 h`. Needs `n >= 1`, `0 < r_fuel < r_clad`, `2 r_clad <= pitch`, `h > 0`.
pub fn pin_assembly(n: usize, pitch: f64, r_fuel: f64, r_clad: f64, h: f64) -> Result<ModelFixture, BuildError> {
    if n == 0 || n > 1 << 10 || !(0.0 < r_fuel && r_fuel < r_clad && 2.0 * r_clad <= pitch && h > 0.0) {
        return Err(bad("pin_assembly needs 1 <= n, 0 < r_fuel < r_clad, 2 r_clad <= pitch, h > 0"));
    }
    let mut t = CellComplex::new();
    let cell = run(&mut t, 1, |t| pin_cell(t, r_fuel, Outer::Round(r_clad), h))?;
    let (mut m, proto) = pin_model(&t, &cell)?;
    let lat = m.add_lattice(Lattice::new(Layout::Cartesian { n: [n as u32, n as u32, 1], pitch: Vector3::new(pitch, pitch, 0.0) }, Transform::IDENTITY, Fill::Prototype(proto), BTreeMap::new()))?;
    let span = n as f64 * pitch;
    let members: Vec<ClusterMember> = (0..3)
        .flat_map(|j| (0..3).map(move |i| (i, j)))
        .map(|(i, j)| ClusterMember::new(Fill::Lattice(lat), Transform::translation(Vector3::new(i as f64 * span, j as f64 * span, 0.0))))
        .collect();
    let cl = m.add_cluster(Cluster::new(members))?;
    m.add_instance(InstanceRow::new(Fill::Cluster(cl), Transform::IDENTITY))?;
    Ok(ModelFixture { name: "pin_assembly", complex: t, model: m, flat_paths: 9 * n * n, flat_entries: 18 * n * n, deck_faces: 0, volume: PI * r_clad * r_clad * h })
}

/// Edge length of the `instanced_core` cell and the grid spacing (cells are `2 x 2 x 2` bricks).
pub const CORE_CELL: f64 = 2.0;

/// `n` copies of one prototype on a grid, for acceleration-structure (TLAS) build and memory
/// tests: the prototype is the `2 x 2 x 2` brick `[-1, 1]^2 x [0, 2]` with a coaxial cylinder
/// inside it (radius 0.4, height 2: the cylinder fills a cylindrical hole of the brick, sharing
/// the interface face). `n` instance rows place it with spacing `2.5` on a cubic grid of side
/// `ceil(cbrt(n))` (x fastest, then y, then z). Flattening gives `n` paths and `2 n` entries;
/// `volume` is the base cell, `8`. Needs `1 <= n <= 2^22`.
pub fn instanced_core(n: usize) -> Result<ModelFixture, BuildError> {
    if n == 0 || n > 1 << 22 {
        return Err(bad("instanced_core needs 1 <= n <= 2^22"));
    }
    let mut t = CellComplex::new();
    let cell = run(&mut t, 1, |t| pin_cell(t, 0.4, Outer::Square(1.0), CORE_CELL))?;
    let (mut m, proto) = pin_model(&t, &cell)?;
    let mut side = 1usize;
    while side * side * side < n {
        side += 1;
    }
    let spacing = 2.5;
    for k in 0..n {
        let (i, j, l) = (k % side, (k / side) % side, k / (side * side));
        let at = Transform::translation(Vector3::new(i as f64 * spacing, j as f64 * spacing, l as f64 * spacing));
        m.add_instance(InstanceRow::new(Fill::Prototype(proto), at))?;
    }
    Ok(ModelFixture { name: "instanced_core", complex: t, model: m, flat_paths: n, flat_entries: 2 * n, deck_faces: 0, volume: 8.0 })
}
