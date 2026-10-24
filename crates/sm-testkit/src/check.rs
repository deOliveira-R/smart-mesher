//! Property checkers. Each returns `Ok(())` or a [`CheckError`] listing the problems, so tests
//! can `unwrap()` good inputs and match on bad ones.
//!
//! Mesh checks: [`mesh_valid`], [`orientation_rule`], [`closed_cells`], [`positive_volumes`],
//! [`watertight`], [`classification_complete`], [`check_mesh_euler`], [`conformity`].
//! Topology checks: [`topo_valid`], [`check_topo_euler`], [`manifold_edges`], [`model_valid`].
//! Builder contract: [`template_cells_match_default`].
//! Hashes: [`mesh_hash`], [`topo_hash`].
//! Layers: [`snapshot_support_consistent`].

use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{Frame, Line, ParametricCurve, ParametricSurface, Plane};
use sm_mesh_api::{
    MeshBuild, VertexIx, add_template_cells_sequential, CellIx, FaceIx, FaceKind, GeomTol, MeshEntityKind, MeshRead, Side, ViolationCode, cell_volume, cell_volume_centroid, connectivity_hash,
    face_area_vector, face_centroid, geometry_hash, topology_hash, validate, validate_geometry,
};
use sm_model_api::{Model, Tolerance};
use sm_topo_api::{CellFlags, Dim, Severity, TopologyView, euler_characteristic};
use std::collections::{BTreeMap, BTreeSet};

/// A failed check: which check and what it found.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[error("{check} failed with {} problem(s); first: {}", .problems.len(), .problems.first().map_or("", String::as_str))]
pub struct CheckError {
    /// Name of the check.
    pub check: &'static str,
    /// One line per problem (never empty).
    pub problems: Vec<String>,
}

/// Result of a check.
pub type CheckResult = Result<(), CheckError>;

fn done(check: &'static str, problems: Vec<String>) -> CheckResult {
    if problems.is_empty() { Ok(()) } else { Err(CheckError { check, problems }) }
}

fn cells(m: &dyn MeshRead) -> impl Iterator<Item = CellIx> {
    (0..m.cell_count()).map(CellIx::new)
}

fn faces(m: &dyn MeshRead) -> impl Iterator<Item = FaceIx> {
    (0..m.face_count()).map(FaceIx::new)
}

// ------------------------------------------------------------------------------ layer checks

/// A layer that reports `supports_snapshot() == true` must return `Ok(Some(_))` from
/// `snapshot()` (checked on non-empty layers; an empty layer may snapshot to nothing).
pub fn snapshot_support_consistent(layer: &dyn sm_op_api::Layer) -> CheckResult {
    let mut p = Vec::new();
    if layer.supports_snapshot() && !layer.is_empty() {
        match layer.snapshot() {
            Ok(Some(_)) => {}
            Ok(None) => p.push(format!("layer '{}': supports_snapshot() but snapshot() is None", layer.name())),
            Err(e) => p.push(format!("layer '{}': supports_snapshot() but snapshot() failed: {e}", layer.name())),
        }
    }
    done("snapshot_support_consistent", p)
}

// ------------------------------------------------------------------------------- mesh checks

/// Contract rule of `MeshBuild::add_template_cells`: an override gives the same result as the
/// default loop. `make` returns a fresh empty builder (called twice); the points become its
/// vertices. Both paths must return the same `Result` (range, or error text), and on success
/// build the same face table (loops, left/right cells) and the same cell face lists, which is
/// everything `connectivity_hash` reads from topology. After an error both must hold the same
/// number of cells.
pub fn template_cells_match_default<B: MeshBuild>(
    make: impl Fn() -> B,
    points: &[[f64; 3]],
    shapes: &[sm_mesh_api::CellShape],
    offsets: &[u64],
    verts: &[VertexIx],
) -> CheckResult {
    let fill = |b: &mut B| -> Result<(), String> {
        for p in points {
            b.add_vertex(Point3::new(p[0], p[1], p[2])).map_err(|e| e.to_string())?;
        }
        Ok(())
    };
    let (mut a, mut d) = (make(), make());
    if let Err(e) = fill(&mut a).and_then(|()| fill(&mut d)) {
        return done("template_cells_match_default", vec![format!("could not add points: {e}")]);
    }
    let ra = a.add_template_cells(shapes, offsets, verts);
    let rd = add_template_cells_sequential(&mut d, shapes, offsets, verts);
    let mut problems = Vec::new();
    match (&ra, &rd) {
        (Ok(x), Ok(y)) if x == y => {}
        (Err(x), Err(y)) if x.to_string() == y.to_string() => {}
        _ => problems.push(format!("results differ: override {ra:?}, default {rd:?}")),
    }
    if a.cell_count() != d.cell_count() {
        problems.push(format!("cell counts differ: {} vs {}", a.cell_count(), d.cell_count()));
    }
    if ra.is_ok() && rd.is_ok() && problems.is_empty() {
        if a.face_count() != d.face_count() {
            problems.push(format!("face counts differ: {} vs {}", a.face_count(), d.face_count()));
        } else {
            for f in 0..a.face_count() {
                let fi = FaceIx::new(f);
                if a.face_loop(fi) != d.face_loop(fi)
                    || a.face_cell(fi, Side::Left) != d.face_cell(fi, Side::Left)
                    || a.face_cell(fi, Side::Right) != d.face_cell(fi, Side::Right)
                {
                    problems.push(format!("face {f} differs"));
                    break;
                }
            }
            for c in 0..a.cell_count() {
                if a.cell_faces(CellIx::new(c)) != d.cell_faces(CellIx::new(c)) {
                    problems.push(format!("cell {c} face list differs"));
                    break;
                }
            }
        }
    }
    done("template_cells_match_default", problems)
}

/// `sm_mesh_api::validate` and `validate_geometry` (default tolerance) are both empty.
pub fn mesh_valid(m: &dyn MeshRead) -> CheckResult {
    let mut v = validate(m);
    v.extend(validate_geometry(m, &GeomTol::default()));
    done("mesh_valid", v.iter().map(|x| format!("{x:?}")).collect())
}

/// The orientation rule (ir.md section 6), checked independently of `validate`: `left` is the
/// lower cell id, and the face area vector points from the left cell's centroid to the right
/// cell's centroid (outward from the left cell for a one-cell face).
pub fn orientation_rule(m: &dyn MeshRead) -> CheckResult {
    let mut p = Vec::new();
    for f in faces(m) {
        let l = m.face_left(f);
        let n = face_area_vector(m, f);
        let (_, cl) = cell_volume_centroid(m, l);
        let toward = match m.face_right(f) {
            Some(r) => {
                if r <= l {
                    p.push(format!("face {f:?}: left {l:?} is not lower than right {r:?}"));
                }
                let (_, cr) = cell_volume_centroid(m, r);
                cr - cl
            }
            None => face_centroid(m, f) - cl,
        };
        if n.dot(toward) <= 0.0 {
            p.push(format!("face {f:?}: normal does not point from left to right"));
        }
    }
    done("orientation_rule", p)
}

/// Every cell is a closed oriented surface: after orienting each face outward, each undirected
/// edge is traversed once in each direction.
pub fn closed_cells(m: &dyn MeshRead) -> CheckResult {
    let mut p = Vec::new();
    for c in cells(m) {
        let mut net: BTreeMap<(u32, u32), i32> = BTreeMap::new();
        for sf in m.cell_faces(c) {
            let vs = m.face_vertices(sf.face());
            for i in 0..vs.len() {
                let (mut a, mut b) = (vs[i].0, vs[(i + 1) % vs.len()].0);
                if sf.side() == Side::Right {
                    core::mem::swap(&mut a, &mut b);
                }
                *net.entry((a.min(b), a.max(b))).or_insert(0) += if a < b { 1 } else { -1 };
            }
        }
        if let Some(((a, b), k)) = net.iter().find(|(_, k)| **k != 0) {
            p.push(format!("cell {c:?}: edge ({a}, {b}) is not cancelled (net {k})"));
        }
    }
    done("closed_cells", p)
}

/// Every cell has a positive volume (fan rule `smartmesh.fan_vertex_mean.v1`).
pub fn positive_volumes(m: &dyn MeshRead) -> CheckResult {
    let p = cells(m).map(|c| (c, cell_volume(m, c))).filter(|(_, v)| *v <= 0.0).map(|(c, v)| format!("cell {c:?}: volume {v}")).collect();
    done("positive_volumes", p)
}

/// The boundary (faces with no right cell) is a closed surface: each undirected edge of the
/// boundary faces belongs to exactly two of them.
pub fn watertight(m: &dyn MeshRead) -> CheckResult {
    let mut uses: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for f in faces(m).filter(|f| m.face_right(*f).is_none()) {
        let vs = m.face_vertices(f);
        for i in 0..vs.len() {
            let (a, b) = (vs[i].0, vs[(i + 1) % vs.len()].0);
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    done("watertight", uses.iter().filter(|(_, n)| **n != 2).map(|((a, b), n)| format!("boundary edge ({a}, {b}) used {n} times")).collect())
}

/// Classification is complete and consistent: every vertex, face and cell has a valid
/// classification, boundary faces lie on a `TopoFace`, cells on a `TopoVolume` or
/// `PartitionCell`, and `validate` reports no classification violation.
pub fn classification_complete(m: &dyn MeshRead) -> CheckResult {
    let mut p = Vec::new();
    for (k, n) in [(MeshEntityKind::Vertex, m.vertex_count()), (MeshEntityKind::Face, m.face_count()), (MeshEntityKind::Cell, m.cell_count())] {
        for i in 0..n {
            let c = m.classification(k, i as u32);
            if !c.is_valid() {
                p.push(format!("{k:?} {i}: invalid classification {c:?}"));
                continue;
            }
            let ek = c.entity.kind();
            let ok = match k {
                MeshEntityKind::Cell => matches!(ek, EntityKind::TopoVolume | EntityKind::PartitionCell),
                MeshEntityKind::Face if m.face_kind(FaceIx::new(i)) == FaceKind::Boundary => ek == EntityKind::TopoFace,
                _ => true,
            };
            if !ok {
                p.push(format!("{k:?} {i}: classified on a {ek:?}"));
            }
        }
    }
    use ViolationCode::{ClassDimOrder, ClassEntity, ClassIndex, ClassKind, ClassParam};
    p.extend(validate(m).into_iter().filter(|v| matches!(v.code, ClassEntity | ClassParam | ClassKind | ClassDimOrder | ClassIndex)).map(|v| format!("{v:?}")));
    done("classification_complete", p)
}

/// `V - E + F - C` of the mesh (edges counted from the face loops). A mesh of one ball has 1.
pub fn mesh_euler(m: &dyn MeshRead) -> i64 {
    let mut edges: BTreeSet<(u32, u32)> = BTreeSet::new();
    for f in faces(m) {
        let vs = m.face_vertices(f);
        for i in 0..vs.len() {
            let (a, b) = (vs[i].0, vs[(i + 1) % vs.len()].0);
            edges.insert((a.min(b), a.max(b)));
        }
    }
    m.vertex_count() as i64 - edges.len() as i64 + m.face_count() as i64 - m.cell_count() as i64
}

/// [`mesh_euler`] equals `expected`.
pub fn check_mesh_euler(m: &dyn MeshRead, expected: i64) -> CheckResult {
    let got = mesh_euler(m);
    done("mesh_euler", if got == expected { vec![] } else { vec![format!("V - E + F - C = {got}, expected {expected}")] })
}

/// Model carriers by id, for [`conformity`].
#[derive(Default)]
pub struct Carriers {
    /// Surfaces by `TopoFace` id.
    pub surfaces: BTreeMap<EntityId, Box<dyn ParametricSurface>>,
    /// Curves by `TopoEdge` id.
    pub curves: BTreeMap<EntityId, Box<dyn ParametricCurve>>,
    /// Points by `TopoVertex` id.
    pub points: BTreeMap<EntityId, Point3>,
}

impl core::fmt::Debug for Carriers {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Carriers({} surfaces, {} curves, {} points)", self.surfaces.len(), self.curves.len(), self.points.len())
    }
}

/// The carriers of `sm_mesh_api::reference::hex_block(.., h)` with `n` cells per axis: faces
/// `TopoFace#1..6` are the planes `-x, +x, -y, +y, -z, +z` of the box `[0, n h]`, edges
/// `TopoEdge#1..12` and vertices `TopoVertex#1..8` follow the numbering documented there
/// (edge `1 + 4 a + s0 + 2 s1` runs along axis `a` at the sides `s0, s1` of the other two axes in
/// ascending order; vertex `1 + sx + 2 sy + 4 sz`).
pub fn box_carriers(n: [u32; 3], h: [f64; 3]) -> Carriers {
    let ext = [n[0] as f64 * h[0], n[1] as f64 * h[1], n[2] as f64 * h[2]];
    let id = |k, s: u64| EntityId::new(k, s).unwrap_or_else(|| unreachable!("small serial"));
    let mut c = Carriers::default();
    for ax in 0..3 {
        for s in 0..2 {
            let mut z = [0.0; 3];
            z[ax] = 1.0;
            let mut o = [0.0; 3];
            o[ax] = s as f64 * ext[ax];
            if let Ok(fr) = Frame::from_z(Point3::new(o[0], o[1], o[2]), sm_core::Vector3::new(z[0], z[1], z[2])) {
                c.surfaces.insert(id(EntityKind::TopoFace, 1 + (ax * 2 + s) as u64), Box::new(Plane { frame: fr }));
            }
        }
    }
    for a in 0..3usize {
        let others: Vec<usize> = (0..3).filter(|&x| x != a).collect();
        for s0 in 0..2 {
            for s1 in 0..2 {
                let mut o = [0.0; 3];
                o[others[0]] = s0 as f64 * ext[others[0]];
                o[others[1]] = s1 as f64 * ext[others[1]];
                let mut d = [0.0; 3];
                d[a] = 1.0;
                if let Ok(l) = Line::new(Point3::new(o[0], o[1], o[2]), sm_core::Vector3::new(d[0], d[1], d[2])) {
                    c.curves.insert(id(EntityKind::TopoEdge, 1 + (a * 4 + s0 + 2 * s1) as u64), Box::new(l));
                }
            }
        }
    }
    for k in 0..8u64 {
        let p = Point3::new((k & 1) as f64 * ext[0], ((k >> 1) & 1) as f64 * ext[1], ((k >> 2) & 1) as f64 * ext[2]);
        c.points.insert(id(EntityKind::TopoVertex, 1 + k), p);
    }
    c
}

/// Geometric conformity: every vertex lies within `tol` of the model entity it is classified on
/// (surface, curve or point), and every vertex of a boundary face lies within `tol` of the
/// surface the face is classified on. Entities with no carrier are reported.
pub fn conformity(m: &dyn MeshRead, carriers: &Carriers, tol: f64) -> CheckResult {
    let mut p = Vec::new();
    let dist = |e: EntityId, q: Point3| -> Result<f64, String> {
        match e.kind() {
            EntityKind::TopoFace => carriers.surfaces.get(&e).ok_or_else(|| format!("no surface for {e:?}"))?.project(q, None).map(|r| r.distance).map_err(|x| x.to_string()),
            EntityKind::TopoEdge => carriers.curves.get(&e).ok_or_else(|| format!("no curve for {e:?}"))?.project(q, None).map(|r| r.distance).map_err(|x| x.to_string()),
            EntityKind::TopoVertex => carriers.points.get(&e).map(|c| (*c - q).norm()).ok_or_else(|| format!("no point for {e:?}")),
            _ => Ok(0.0),
        }
    };
    // The label is built only when a problem is found (no allocation per element on success).
    let mut check = |what: &dyn Fn() -> String, e: EntityId, q: Point3| match dist(e, q) {
        Ok(d) if d <= tol => {}
        Ok(d) => p.push(format!("{}: {d:e} from {e:?} (tol {tol:e})", what())),
        Err(msg) => p.push(format!("{}: {msg}", what())),
    };
    for v in 0..m.vertex_count() {
        check(&|| format!("vertex {v}"), m.classification(MeshEntityKind::Vertex, v as u32).entity, m.point(v));
    }
    for f in faces(m).filter(|f| m.is_boundary_face(*f)) {
        let e = m.classification(MeshEntityKind::Face, f.0).entity;
        for v in m.face_vertices(f) {
            check(&|| format!("boundary face {f:?} vertex {v:?}"), e, m.point(v.get()));
        }
    }
    done("conformity", p)
}

// -------------------------------------------------------------------------- topology checks

/// `sm_topo_api::validate` and `validate_geometry` report no error.
pub fn topo_valid(v: &dyn TopologyView) -> CheckResult {
    let mut issues = sm_topo_api::validate(v);
    issues.extend(sm_topo_api::validate_geometry(v));
    done("topo_valid", issues.iter().filter(|i| i.severity == Severity::Error).map(|i| format!("{:?} on {:?}: {}", i.rule, i.cell, i.message)).collect())
}

/// `V - E + F` (pole edges excluded) equals `expected`.
pub fn check_topo_euler(v: &dyn TopologyView, expected: i64) -> CheckResult {
    let got = euler_characteristic(v);
    done("topo_euler", if got == expected { vec![] } else { vec![format!("V - E + F = {got}, expected {expected}")] })
}

/// Every non-degenerate edge has exactly two face uses (a manifold surface complex: no fins,
/// baffle edges or T-junctions of several faces).
pub fn manifold_edges(v: &dyn TopologyView) -> CheckResult {
    let p = v
        .cells(Dim::Edge)
        .iter()
        .filter(|e| !v.flags(**e).contains(CellFlags::DEGENERATE))
        .map(|e| (e, v.radial_uses(*e).len()))
        .filter(|(_, n)| *n != 2)
        .map(|(e, n)| format!("edge {e:?} has {n} face uses"))
        .collect();
    done("manifold_edges", p)
}

/// `sm_model_api::validate` reports no error (warnings allowed).
pub fn model_valid(m: &Model, t: &dyn TopologyView) -> CheckResult {
    let r = sm_model_api::validate(m, t, Tolerance::default());
    done("model_valid", r.errors().map(|i| format!("{:?}: {}", i.code, i.detail)).collect())
}

// ------------------------------------------------------------------------------------ hashes

fn mix(parts: &[u64]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for p in parts {
        for b in p.to_le_bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    h
}

/// One hash of everything observable about a mesh: connectivity, topology tables and geometry.
pub fn mesh_hash(m: &dyn MeshRead) -> u64 {
    mix(&[connectivity_hash(m), topology_hash(m), geometry_hash(m)])
}

/// The canonical (cross-platform, quantized) state hash of a topology.
pub fn topo_hash(v: &dyn TopologyView) -> u64 {
    v.canonical_hash()
}
