//! The scheme adapter: [`TetVolume`] meshes a volume piece from the triangle patches of its
//! faces.
//!
//! Orientation is read from the topology, never inferred from geometry: a face use with
//! `Side::Left` has the volume on the left, so the face normal (the winding of the face's
//! triangles) points out of the volume and the triangles are kept; `Side::Right` flips them.
//! Void shells use the same rule, so their triangles point into the cavity.

use crate::{QualityDiagnostic, QualityMetric, SizeSpec, SuggestedFix, SurfaceInput, TetError, TetMesh, TetOptions, tetrahedralize};
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellShape, Param};
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, Firmness, GenCtx, GenerateFailure, NodeKey, Params, Patch, PatchBuilder, PlanCtx,
    Requirements, Scheme, SchemeDim, SchemeNote, SchemeRegistry, SizeField, Target,
};
use sm_topo_api::{CellFlags, Dim, Incidence, Side, TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

/// Name the scheme registers under.
pub const NAME: &str = "tetmesh";

/// Scheme parameters (`mesh.scheme ... params`). Every field is optional; unknown keys are
/// rejected.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TetParams {
    /// Constant target edge length, combined with the sizing field by `min`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub size: Option<f64>,
    /// Radius-edge bound of refinement (default 2).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub radius_edge: Option<f64>,
    /// Optimization level 0..=2 (default 2).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub optimize: Option<u8>,
    /// Size gradation (default 1.3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gradation: Option<f64>,
    /// Reporting floor for the smallest tet dihedral angle in degrees (default
    /// [`MIN_DIHEDRAL_DEG`](crate::MIN_DIHEDRAL_DEG), at most 70.5): tets below it become
    /// located scheme notes. Does not change the mesh.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_dihedral_deg: Option<f64>,
    /// Reporting floor for the tet mean ratio (default
    /// [`MIN_MEAN_RATIO`](crate::MIN_MEAN_RATIO), at most 1). Does not change the mesh.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_mean_ratio: Option<f64>,
    /// Hex-tet transition (W4-HEX-PYR, ruling D5): `none` (default, today's behaviour: every
    /// face must be triangulated) or `pyramid` (a quad on a bounding face becomes the base of
    /// one pyramid whose apex lies inside this volume; its four triangles are frozen boundary
    /// for the tet fill, so the quad nodes stay shared with the hex neighbour).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub transition: Option<Transition>,
    /// Pyramid apex height as a fraction of the mean quad edge (default
    /// [`DEFAULT_PYRAMID_HEIGHT`], range 0.2..=2). Only with `transition: "pyramid"`; a failed
    /// recovery retries with 0.6 and 0.35 times this value.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pyramid_height: Option<f64>,
}

/// Default pyramid height over the mean edge of its base quad (the spike value: 0.6 keeps the
/// pyramid's own faces at about 60 degrees and the neighbouring tets well shaped).
pub const DEFAULT_PYRAMID_HEIGHT: f64 = 0.6;

/// How the tet scheme meets quad faces of a hex neighbour.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Transition {
    /// Triangles on every face (the W3 behaviour).
    None,
    /// One pyramid per quad of a bounding face.
    Pyramid,
}

impl TetParams {
    fn pyramid(&self) -> bool {
        self.transition == Some(Transition::Pyramid)
    }
}

impl TetParams {
    /// Parse and check `params`; the error text is the reason shown to the user.
    pub fn parse(params: &Params) -> Result<Self, String> {
        let p: TetParams = if params.0.is_null() {
            TetParams::default()
        } else {
            serde_json::from_value(params.0.clone()).map_err(|e| format!("tetmesh params: {e}"))?
        };
        if let Some(s) = p.size
            && !(s.is_finite() && s > 0.0) {
                return Err(format!("tetmesh params: size = {s} must be positive and finite"));
            }
        if let Some(r) = p.radius_edge
            && !(r.is_finite() && r >= 1.0) {
                return Err(format!("tetmesh params: radius_edge = {r} must be finite and >= 1"));
            }
        if let Some(o) = p.optimize
            && o > 2 {
                return Err(format!("tetmesh params: optimize = {o} must be 0, 1 or 2"));
            }
        if let Some(g) = p.gradation
            && !(g.is_finite() && g >= 1.0) {
                return Err(format!("tetmesh params: gradation = {g} must be finite and >= 1"));
            }
        if let Some(d) = p.min_dihedral_deg
            && !(d.is_finite() && (0.0..=70.5).contains(&d)) {
                return Err(format!("tetmesh params: min_dihedral_deg = {d} must be in 0..=70.5"));
            }
        if let Some(m) = p.min_mean_ratio
            && !(m.is_finite() && (0.0..=1.0).contains(&m)) {
                return Err(format!("tetmesh params: min_mean_ratio = {m} must be in 0..=1"));
            }
        if let Some(h) = p.pyramid_height {
            if !(h.is_finite() && (0.2..=2.0).contains(&h)) {
                return Err(format!("tetmesh params: pyramid_height = {h} must be in 0.2..=2"));
            }
            if !p.pyramid() {
                return Err("tetmesh params: pyramid_height needs transition = \"pyramid\"".to_owned());
            }
        }
        Ok(p)
    }
}

/// Tetrahedral volume scheme (`"tetmesh"`): Delaunay, boundary recovery with interior Steiner
/// points only, refinement and optimization. Needs a triangle mesh on every piece face.
#[derive(Clone, Copy, Debug, Default)]
pub struct TetVolume;

/// Register [`TetVolume`]. It registers no ops, queries or layers.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(TetVolume))
}

/// The volume's face uses: `(face, side, ring)`, ascending by face, and whether the piece has
/// features the scheme does not support.
struct Uses {
    uses: Vec<(EntityId, Side, u16)>,
    unsupported: Option<String>,
}

fn face_uses(topo: &dyn TopologyView, volume: EntityId) -> Uses {
    let mut uses = Vec::new();
    let mut unsupported = None;
    for inc in topo.boundary(volume) {
        if inc.ring == Incidence::EMBEDDED {
            unsupported = Some("embedded members (wires or points) inside the volume".to_owned());
            continue;
        }
        if topo.dim_of(inc.cell) != Some(Dim::Face) {
            continue;
        }
        if inc.side == Side::Both {
            unsupported = Some(format!("{} is used on both sides (an internal baffle)", inc.cell));
        }
        uses.push((inc.cell, inc.side, inc.ring));
    }
    uses.sort_unstable_by_key(|u| (u.0, u.2));
    Uses { uses, unsupported }
}

/// Every edge of the shell must be traversed once in each direction by the face uses.
fn shell_closed(topo: &dyn TopologyView, uses: &[(EntityId, Side, u16)]) -> bool {
    let mut count: BTreeMap<EntityId, (u32, u32)> = BTreeMap::new();
    for &(f, side, _) in uses {
        for lp in topo.loops(f) {
            for k in lp.keys() {
                if topo.flags(k.edge).contains(CellFlags::DEGENERATE) {
                    continue;
                }
                let forward = (k.side == Side::Left) == (side == Side::Left);
                let e = count.entry(k.edge).or_default();
                if forward {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
        }
    }
    count.values().all(|&(a, b)| a == b && a >= 1)
}

impl Scheme for TetVolume {
    fn name(&self) -> &str {
        NAME
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, params: &Params) -> Caps {
        if TetParams::parse(params).is_ok_and(|p| p.pyramid()) { Caps::TET | Caps::PYRAMID } else { Caps::TET }
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Volume(v) = target else {
            return Applicability::no("not a volume");
        };
        if let Err(reason) = TetParams::parse(params) {
            return Applicability::no(reason);
        }
        if let Err(e) = cx.input.piece_faces(target) {
            return Applicability::no(e.to_string());
        }
        if !cx.input.embedded_edges(v).is_empty() || !cx.input.embedded_vertices(v).is_empty() {
            return Applicability::no("embedded edges or vertices are not supported (W3b)");
        }
        let u = face_uses(cx.input.topo, v);
        if let Some(why) = u.unsupported {
            return Applicability::no(format!("unsupported: {why}"));
        }
        if u.uses.is_empty() {
            return Applicability::no("the volume has no boundary faces");
        }
        if !shell_closed(cx.input.topo, &u.uses) {
            return Applicability::no("the boundary shell is not closed");
        }
        Applicability::yes(10, "last resort: Delaunay + recovery + refinement, needs triangles on every face")
    }

    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, params: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        // With the pyramid transition TRI is a preference: a Hard QUAD of a hex neighbour wins
        // in the same planning pass (`planner::resolve`), a face used only here stays triangles.
        let firmness = if TetParams::parse(params).is_ok_and(|p| p.pyramid()) { Firmness::Soft } else { Firmness::Hard };
        for f in cx.input.piece_faces(target)? {
            out.require_caps(Target::Surface(f), Caps::TRI, firmness);
        }
        Ok(())
    }

    fn generate(&self, cx: &GenCtx<'_>, target: Target, params: &Params) -> Result<Patch, GenerateFailure> {
        let fail = |msg: String| GenerateFailure::fatal(target, NAME, msg);
        let Target::Volume(owner) = target else {
            return Err(fail("not a volume".into()));
        };
        let p = TetParams::parse(params).map_err(fail)?;
        let u = face_uses(cx.input.topo, owner);
        if let Some(why) = u.unsupported {
            return Err(fail(format!("unsupported: {why}")));
        }
        // Sides of the piece faces, from the topology.
        let faces = cx.input.piece_faces(target).map_err(|e| fail(e.to_string()))?;
        let mut side_of: BTreeMap<EntityId, Side> = BTreeMap::new();
        for &(f, s, _) in &u.uses {
            side_of.insert(f, s);
        }
        // Triangles as node keys, with the owning face.
        let mut tri_keys: Vec<[NodeKey; 3]> = Vec::new();
        let mut tri_face: Vec<EntityId> = Vec::new();
        // Quads (outward order) and their faces: only with `transition: "pyramid"`.
        let pyramid = p.pyramid();
        let mut quad_keys: Vec<[NodeKey; 4]> = Vec::new();
        let mut quad_face: Vec<EntityId> = Vec::new();
        for f in &faces {
            let side = *side_of.get(f).ok_or_else(|| fail(format!("{f} is not a face use of {owner}: cannot determine its orientation")))?;
            let patch = cx.surface_patch(*f).ok_or_else(|| fail(format!("the surface patch of {f} is missing (is a triangle scheme set on it?)")))?;
            for (shape, local) in patch.elements.iter() {
                let key = |i: usize| patch.key(local[i]).ok_or_else(|| fail(format!("{f}: dangling node index in its patch")));
                match shape {
                    ElemShape::Tri => {
                        let (a, b, c) = (key(0)?, key(1)?, key(2)?);
                        tri_keys.push(if side == Side::Right { [a, c, b] } else { [a, b, c] });
                        tri_face.push(*f);
                    }
                    ElemShape::Quad if pyramid => {
                        let (a, b, c, d) = (key(0)?, key(1)?, key(2)?, key(3)?);
                        quad_keys.push(if side == Side::Right { [a, d, c, b] } else { [a, b, c, d] });
                        quad_face.push(*f);
                    }
                    _ if pyramid => return Err(fail(format!("{f}: found {shape:?}; the pyramid transition takes triangles and quads"))),
                    _ => return Err(fail(format!("{f} is not triangulated (found {shape:?}); set mesh.scheme trimesh on its faces"))),
                }
            }
        }
        // Dense point ids in ascending key order.
        let keys: BTreeSet<NodeKey> = tri_keys.iter().flatten().chain(quad_keys.iter().flatten()).copied().collect();
        let keys: Vec<NodeKey> = keys.into_iter().collect();
        let id_of: BTreeMap<NodeKey, u32> = keys.iter().enumerate().map(|(i, k)| (*k, i as u32)).collect();
        let mut points: Vec<[f64; 3]> = Vec::with_capacity(keys.len());
        for k in &keys {
            let q = cx.point(*k).ok_or_else(|| fail(format!("node {k:?} has no position")))?;
            points.push([q.x, q.y, q.z]);
        }
        let tris: Vec<[u32; 3]> = tri_keys.iter().map(|t| t.map(|k| id_of[&k])).collect();
        let quads: Vec<[u32; 4]> = quad_keys.iter().map(|q| q.map(|k| id_of[&k])).collect();
        let n_ref = keys.len();
        let plain = SurfaceInput { points, tris };

        // Options. A size field from the context and the constant `size` parameter combine by min.
        let ctx_size: Option<&dyn SizeField> = cx.size;
        let const_size = p.size;
        let field = move |q: [f64; 3]| -> f64 {
            let mut h = f64::INFINITY;
            if let Some(s) = ctx_size {
                h = h.min(s.size(Point3::new(q[0], q[1], q[2])));
            }
            if let Some(c) = const_size {
                h = h.min(c);
            }
            h
        };
        let mut opts = TetOptions::default();
        if ctx_size.is_some() {
            opts.size = SizeSpec::Field(&field);
        } else if let Some(c) = const_size {
            opts.size = SizeSpec::Constant(c);
        }
        if let Some(r) = p.radius_edge {
            opts.radius_edge = r;
        }
        if let Some(o) = p.optimize {
            opts.optimize = o;
        }
        if let Some(g) = p.gradation {
            opts.gradation = g;
        }
        if let Some(d) = p.min_dihedral_deg {
            opts.floor_dihedral_deg = d;
        }
        if let Some(m) = p.min_mean_ratio {
            opts.floor_mean_ratio = m;
        }
        // One shell per apex height: volume triangles + the four side triangles of every
        // pyramid (see `pyramid_apexes`). Without quads this runs once, on the plain input.
        let base = p.pyramid_height.unwrap_or(DEFAULT_PYRAMID_HEIGHT);
        let heights: &[f64] = if quads.is_empty() { &[1.0] } else { &[1.0, 0.6, 0.35] };
        let mut last: Option<GenerateFailure> = None;
        let mut retried_out = false;
        for &scale in heights {
            let mut input = plain.clone();
            let mut keys = keys.clone();
            let mut tri_keys = tri_keys.clone();
            let mut tri_face = tri_face.clone();
            if !quads.is_empty() {
                let apexes = pyramid_apexes(&plain.points, &quads, base * scale).map_err(|(q, why)| {
                    GenerateFailure::new(target, NAME, FailureKind::Geometry, format!("pyramid transition on {}: {why}", quad_face[q]))
                })?;
                // Apexes too close to each other or to foreign shell points make the pyramid
                // shell degenerate; treat it like a recovery failure so a shallower apex is tried.
                if let Some(why) = apex_crowding(&plain.points, &quads, &apexes, base * scale) {
                    last = Some(GenerateFailure::new(target, NAME, FailureKind::Geometry, format!("pyramid transition: {why}")));
                    retried_out = false;
                    continue;
                }
                for (q, ap) in apexes.iter().enumerate() {
                    let key = NodeKey::new(owner, q as u32);
                    let id = input.points.len() as u32;
                    input.points.push(*ap);
                    keys.push(key);
                    let (v, kq) = (quads[q], quad_keys[q]);
                    for e in 0..4 {
                        input.tris.push([v[e], v[(e + 1) % 4], id]);
                        tri_keys.push([kq[e], kq[(e + 1) % 4], key]);
                        tri_face.push(quad_face[q]);
                    }
                }
            }
            if let Some(c) = const_size {
                // The default Steiner budget scales with the boundary; a constant size that is fine
                // against a coarse boundary needs room for the interior points (about one per six
                // tets, a regular tet of edge `c` having volume c^3 / (6 sqrt 2)).
                let vol = input.tris.iter().map(|t| signed_volume6(t.map(|i| input.points[i as usize]))).sum::<f64>().abs() / 6.0;
                let wanted = 2.0 * vol / (c * c * c * 0.1179 * 6.0);
                if wanted.is_finite() {
                    opts.max_steiner = 8 * input.points.len() + 100_000 + wanted.min(1.0e8) as usize;
                }
            }
            match tetrahedralize(&input, &opts) {
                Ok(mesh) if !quads.is_empty() && !tets_positive(&mesh, &quads, &plain.points) => {
                    last = Some(GenerateFailure::new(
                        target,
                        NAME,
                        FailureKind::Geometry,
                        "pyramid transition: pyramid apex height too large, the tetrahedralization has a non-positive or degenerate tet (apexes too close to other apexes or faces)".to_owned(),
                    ));
                    retried_out = false;
                }
                Ok(mesh) => return Ok(build_patch(target, owner, &mesh, &keys, &tri_face, n_ref, &quads)),
                Err(e) => {
                    let retry = matches!(e, TetError::RecoveryFailed { .. } | TetError::SelfIntersection { .. } | TetError::InvertedShell { .. });
                    last = Some(explain(&e, target, &input, &tri_keys, &tri_face, &keys));
                    retried_out = retry;
                    if !retry {
                        break;
                    }
                }
            }
        }
        // With pyramids the failure is usually the apex leaving a thin volume, which a different
        // scheme can fix: type it `Geometry` so the replan loop may fall back (D5).
        match last {
            Some(mut f) if !quads.is_empty() && f.kind == FailureKind::Internal && retried_out => {
                f.kind = FailureKind::Geometry;
                f.reason = format!(
                    "pyramid transition failed: pyramid apex height exceeds the local thickness of the volume; lower pyramid_height or use transition none ({})",
                    f.reason
                );
                Err(f)
            }
            Some(f) => Err(f),
            None => Err(fail("no attempt ran".into())),
        }
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Apex of the pyramid on each outward-ordered quad: the centroid moved by `hfac` times the mean
/// edge length against the outward normal `(c - a) x (d - b)` (into the volume). A quad whose
/// normal vanishes or whose apex would make any corner tetrahedron of the pyramid non-positive
/// (a bow-tie or strongly warped quad) is rejected with its index and the reason.
fn pyramid_apexes(points: &[[f64; 3]], quads: &[[u32; 4]], hfac: f64) -> Result<Vec<[f64; 3]>, (usize, String)> {
    let mut out = Vec::with_capacity(quads.len());
    for (qi, q) in quads.iter().enumerate() {
        let v = q.map(|i| points[i as usize]);
        let n = cross(sub(v[2], v[0]), sub(v[3], v[1]));
        let nl = norm(n);
        let mean_edge = (0..4).map(|e| norm(sub(v[(e + 1) % 4], v[e]))).sum::<f64>() / 4.0;
        if !(nl > 0.0 && mean_edge > 0.0 && nl.is_finite()) {
            return Err((qi, "degenerate quad (zero area)".to_owned()));
        }
        let c = [0, 1, 2].map(|k| (v[0][k] + v[1][k] + v[2][k] + v[3][k]) / 4.0);
        let h = hfac * mean_edge;
        let apex = [0, 1, 2].map(|k| c[k] - h * n[k] / nl);
        // The apex must see every corner triangle of the base on its inner side.
        for e in 0..4 {
            let (a, b, d) = (v[e], v[(e + 1) % 4], v[(e + 3) % 4]);
            let vol = dot(cross(sub(b, a), sub(d, a)), sub(apex, a));
            if vol >= -1e-12 * mean_edge * mean_edge * mean_edge {
                return Err((qi, format!("the quad is not convex enough for a pyramid (corner {e})")));
            }
        }
        out.push(apex);
    }
    Ok(out)
}

/// Minimum clearance of an apex, as a fraction of its own height, from every other apex and from
/// every shell point outside its own base.
const APEX_CLEARANCE: f64 = 0.5;

/// First apex that sits closer than `APEX_CLEARANCE * h` to another apex or a foreign shell
/// point, as a message. Sorted-by-x sweep, so near-linear for ordinary shells.
fn apex_crowding(points: &[[f64; 3]], quads: &[[u32; 4]], apexes: &[[f64; 3]], hfac: f64) -> Option<String> {
    let dist2 = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| (a[k] - b[k]) * (a[k] - b[k])).sum::<f64>();
    let mut cand: Vec<([f64; 3], Option<usize>, u32)> = points.iter().enumerate().map(|(i, p)| (*p, None, i as u32)).collect();
    cand.extend(apexes.iter().enumerate().map(|(q, a)| (*a, Some(q), q as u32)));
    cand.sort_by(|a, b| a.0[0].total_cmp(&b.0[0]));
    let xs: Vec<f64> = cand.iter().map(|c| c.0[0]).collect();
    for (q, a) in apexes.iter().enumerate() {
        let v = quads[q].map(|i| points[i as usize]);
        let mean_edge = (0..4).map(|e| dist2(v[(e + 1) % 4], v[e]).sqrt()).sum::<f64>() / 4.0;
        let r = APEX_CLEARANCE * hfac * mean_edge;
        let lo = xs.partition_point(|&x| x < a[0] - r);
        for c in cand[lo..].iter().take_while(|c| c.0[0] <= a[0] + r) {
            let own = match c.1 {
                Some(o) => o == q,
                None => quads[q].contains(&c.2),
            };
            if !own && dist2(c.0, *a) < r * r {
                return Some(format!(
                    "pyramid apex height exceeds the local thickness: apex of quad {q} lies within {APEX_CLEARANCE} of its height from another apex or shell point; lower pyramid_height or use transition none"
                ));
            }
        }
    }
    None
}

/// Every output tet has a clearly positive volume relative to the pyramid base scale.
fn tets_positive(mesh: &TetMesh, quads: &[[u32; 4]], points: &[[f64; 3]]) -> bool {
    let d = |a: [f64; 3], b: [f64; 3]| norm(sub(a, b));
    let scale = quads.iter().map(|q| (0..4).map(|e| d(points[q[(e + 1) % 4] as usize], points[q[e] as usize])).sum::<f64>() / 4.0).fold(f64::INFINITY, f64::min);
    let tol = 1e-9 * scale * scale * scale;
    mesh.tets.iter().all(|t| {
        let p = t.map(|i| mesh.points[i as usize]);
        dot(cross(sub(p[1], p[0]), sub(p[2], p[0])), sub(p[3], p[0])) > tol
    })
}

/// Six times the signed volume of the cone from the origin over a triangle.
fn signed_volume6(t: [[f64; 3]; 3]) -> f64 {
    let [a, b, c] = t;
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
}

/// Turn a core error into a located, actionable generation failure: the message names the
/// `TopoFace` that owns the offending triangle, its position and a suggested action; the nodes
/// are the triangle's (or edge's) node keys.
fn explain(e: &TetError, target: Target, input: &SurfaceInput, tri_keys: &[[NodeKey; 3]], tri_face: &[EntityId], keys: &[NodeKey]) -> GenerateFailure {
    let key_of = |i: u32| keys[i as usize];
    let (nodes, tri) = locate(e, input, tri_keys, &key_of);
    let what = tri.and_then(|t| tri_face.get(t as usize).copied());
    let centre = tri.and_then(|t| input.tris.get(t as usize)).map(|t| {
        let q = t.map(|i| input.points[i as usize]);
        [(q[0][0] + q[1][0] + q[2][0]) / 3.0, (q[0][1] + q[1][1] + q[2][1]) / 3.0, (q[0][2] + q[1][2] + q[2][2]) / 3.0]
    });
    let at = match (what, centre) {
        (Some(f), Some(c)) => format!(" on {f} near ({:.6}, {:.6}, {:.6})", c[0], c[1], c[2]),
        _ => String::new(),
    };
    let face = what.map_or_else(|| "the boundary faces".to_owned(), |f| f.to_string());
    let action = match e {
        TetError::RecoveryFailed { .. } | TetError::SelfIntersection { .. } => {
            format!("refine intervals on the curves of {face} or remesh it; the surface triangles could not be conformed without boundary Steiner points")
        }
        TetError::NotClosed { .. } | TetError::NonManifold { .. } => "check that the volume shell is closed and every face is meshed (merge coincident faces)".to_owned(),
        TetError::InvertedShell { .. } => "check the orientation of the face meshes of the volume".to_owned(),
        TetError::DegenerateTri { .. } => format!("remesh {face}: it contains a degenerate triangle"),
        TetError::BadIndex { .. } => "the mesh refers to a node that does not exist (internal error in the adapter)".to_owned(),
        TetError::DuplicatePoint { .. } => "two mesh nodes coincide: merge or remove the duplicate".to_owned(),
        TetError::OutOfRange(_) => "scale the model so that coordinates lie within 2^-100 .. 2^100".to_owned(),
        TetError::SteinerBudget { .. } => "increase the element size or simplify the geometry".to_owned(),
        _ => "see the error text".to_owned(),
    };
    // Spec (W3-TET): a TetError is `fatal` with the offending nodes attached. Kept `Internal` on
    // purpose: `Geometry` would let the driver fall back and drop the nodes (an `Attempt`
    // carries only a message), and `tetmesh` is the last resort anyway.
    GenerateFailure::fatal(target, NAME, format!("{e}{at}; suggested action: {action}")).with_nodes(nodes)
}

/// The node keys locating `e` (the offending triangle's corners, the edge's or points' keys) and
/// the offending triangle, if any.
fn locate(e: &TetError, input: &SurfaceInput, tri_keys: &[[NodeKey; 3]], key_of: &dyn Fn(u32) -> NodeKey) -> (Vec<NodeKey>, Option<u32>) {
    let tri = match e {
        TetError::InvertedShell { tri } | TetError::DegenerateTri { tri } | TetError::BadIndex { tri, .. } => Some(*tri),
        TetError::RecoveryFailed { tri, .. } => *tri,
        TetError::SelfIntersection { tri_a, .. } => Some(*tri_a),
        _ => None,
    };
    if let Some(t) = tri
        && let Some(k) = tri_keys.get(t as usize) {
            return (k.to_vec(), Some(t));
        }
    match e {
        TetError::NotClosed { edge } | TetError::NonManifold { edge } => {
            let tri = input.tris.iter().position(|t| edge.iter().all(|x| t.contains(x))).map(|t| t as u32);
            (edge.iter().map(|&i| key_of(i)).collect(), tri)
        }
        TetError::DuplicatePoint { a, b } => (vec![key_of(*a), key_of(*b)], None),
        _ => (Vec::new(), None),
    }
}

/// Emit the patch: Steiner points as nodes owned by the piece, boundary points as references.
/// A core quality diagnostic as a scheme note: the faces owning the nearby input triangles
/// (the volume when the tet is deep inside), the metric, and the suggested fix in words.
fn note_of(d: &QualityDiagnostic, owner: EntityId, tri_face: &[EntityId]) -> SchemeNote {
    let mut entities: Vec<EntityId> = d.triangles.iter().filter_map(|&t| tri_face.get(t as usize).copied()).collect();
    entities.sort_unstable();
    entities.dedup();
    if entities.is_empty() {
        entities.push(owner);
    }
    let metric = match d.metric {
        QualityMetric::MinDihedral => "min_dihedral",
        QualityMetric::MeanRatio => "mean_ratio",
    };
    let suggestion = match d.fix {
        SuggestedFix::RefineSizeNear { h } => format!("refine the size field near this point to about {h:.6e}"),
        SuggestedFix::RefineInputSurface { h } => format!("remesh the listed faces finer (about {h:.6e}) or smooth the surface there"),
    };
    SchemeNote::new(Point3::new(d.position[0], d.position[1], d.position[2]), entities, metric, d.value, d.threshold, suggestion)
}

/// `n_ref` input points are nodes of earlier waves; the following `quads.len()` are pyramid
/// apexes (keys `(owner, 0..)`, owned by the piece); Steiner keys follow the apexes.
fn build_patch(target: Target, owner: EntityId, mesh: &TetMesh, keys: &[NodeKey], tri_face: &[EntityId], n_ref: usize, quads: &[[u32; 4]]) -> Patch {
    let mut pb = PatchBuilder::new(target);
    let mut local: Vec<u32> = Vec::with_capacity(mesh.points.len());
    for (i, p) in mesh.points.iter().enumerate() {
        if i < n_ref {
            local.push(pb.refer(keys[i]));
        } else if i < mesh.n_input {
            local.push(pb.node(keys[i], Point3::new(p[0], p[1], p[2]), Param::None));
        } else {
            let k = (i - n_ref) as u32;
            local.push(pb.node(NodeKey::new(owner, k), Point3::new(p[0], p[1], p[2]), Param::None));
        }
    }
    for t in &mesh.tets {
        pb.push(ElemShape::Cell(CellShape::Tet), &t.map(|i| local[i as usize]));
    }
    // Pyramid template: base counter-clockwise seen from the apex, i.e. the reverse of the
    // outward quad order.
    for (q, v) in quads.iter().enumerate() {
        let apex = local[n_ref + q];
        pb.push(ElemShape::Cell(CellShape::Pyramid), &[local[v[0] as usize], local[v[3] as usize], local[v[2] as usize], local[v[1] as usize], apex]);
    }
    for d in &mesh.stats.quality_diagnostics {
        pb.note(note_of(d, owner, tri_face));
    }
    pb.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    #[test]
    fn apex_of_a_cylinder_quad_lies_inside_and_a_bow_tie_is_rejected() {
        // A quad on the unit cylinder x^2 + y^2 = 1, 20 degrees wide, outward order, outward
        // normal +radial: the apex must be nearer the axis than the surface.
        let t = 10f64.to_radians();
        let p = |a: f64, z: f64| [a.cos(), a.sin(), z];
        // (a, b, c, d) counter-clockwise seen from outside: z up first, then angle up.
        let pts = vec![p(-t, 0.0), p(t, 0.0), p(t, 0.2), p(-t, 0.2)];
        let apex = pyramid_apexes(&pts, &[[0, 1, 2, 3]], 0.6).expect("a smooth quad is accepted")[0];
        assert!((apex[0] * apex[0] + apex[1] * apex[1]).sqrt() < 1.0, "apex {apex:?} is outside the cylinder");
        // Exactly h = 0.6 * mean edge below the centroid along -normal.
        let mean_edge = {
            let d = |i: usize, j: usize| norm(sub(pts[i], pts[j]));
            (d(0, 1) + d(1, 2) + d(2, 3) + d(3, 0)) / 4.0
        };
        let c = [0, 1, 2].map(|k| pts.iter().map(|q| q[k]).sum::<f64>() / 4.0);
        let h = norm(sub(c, apex));
        assert!((h - 0.6 * mean_edge).abs() < 1e-12, "h {h}");
        // A bow-tie (swapped corners) has no consistent inward side: typed rejection, no panic.
        let bow = vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        assert!(pyramid_apexes(&bow, &[[0, 1, 2, 3]], 0.6).is_err());
        // A zero-area quad too.
        let flat = vec![[0.0; 3]; 4];
        assert!(pyramid_apexes(&flat, &[[0, 1, 2, 3]], 0.6).is_err());
    }

    #[test]
    fn recovery_failure_names_the_face_and_carries_the_triangle_keys() {
        let face = EntityId::new(EntityKind::TopoFace, 12).unwrap();
        let vol = EntityId::new(EntityKind::TopoVolume, 1).unwrap();
        let keys: Vec<NodeKey> = (0..4).map(|i| NodeKey::new(face, i)).collect();
        let input = SurfaceInput { points: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], tris: vec![[0, 1, 2], [0, 3, 1]] };
        let tri_keys = vec![[keys[0], keys[1], keys[2]], [keys[0], keys[3], keys[1]]];
        let tri_face = vec![face, face];
        let e = TetError::RecoveryFailed { tri: Some(1), edge: Some([0, 3]), reason: "forced".into() };
        let g = explain(&e, Target::Volume(vol), &input, &tri_keys, &tri_face, &keys);
        assert!(g.reason.contains("TopoFace#12"), "{}", g.reason);
        assert!(g.reason.contains("refine intervals on the curves of TopoFace#12"), "{}", g.reason);
        assert_eq!(g.nodes, tri_keys[1].to_vec());
        assert_eq!(g.kind, sm_scheme_api::FailureKind::Internal);
        // Edge-located errors carry the edge's keys.
        let g = explain(&TetError::NotClosed { edge: [1, 2] }, Target::Volume(vol), &input, &tri_keys, &tri_face, &keys);
        assert_eq!(g.nodes, vec![keys[1], keys[2]]);
        assert!(g.reason.contains("TopoFace#12"));
    }
}
