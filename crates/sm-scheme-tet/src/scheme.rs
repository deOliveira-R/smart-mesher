//! The scheme adapter: [`TetVolume`] meshes a volume piece from the triangle patches of its
//! faces.
//!
//! Orientation is read from the topology, never inferred from geometry: a face use with
//! `Side::Left` has the volume on the left, so the face normal (the winding of the face's
//! triangles) points out of the volume and the triangles are kept; `Side::Right` flips them.
//! Void shells use the same rule, so their triangles point into the cavity.

use crate::{SizeSpec, SurfaceInput, TetError, TetMesh, TetOptions, tetrahedralize};
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{CellShape, Param};
use sm_scheme_api::{
    Applicability, Caps, ElemShape, Firmness, GenCtx, GenerateFailure, NodeKey, Params, Patch, PatchBuilder, PlanCtx,
    Requirements, Scheme, SchemeDim, SchemeRegistry, SizeField, Target,
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
    fn caps(&self, _params: &Params) -> Caps {
        Caps::TET
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
        if u.uses.len() < 4 {
            return Applicability::no("a closed shell needs at least four faces");
        }
        if !shell_closed(cx.input.topo, &u.uses) {
            return Applicability::no("the boundary shell is not closed");
        }
        Applicability::yes(10, "last resort: Delaunay + recovery + refinement, needs triangles on every face")
    }

    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, _params: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        for f in cx.input.piece_faces(target)? {
            out.require_caps(Target::Surface(f), Caps::TRI, Firmness::Hard);
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
        for f in &faces {
            let side = *side_of.get(f).ok_or_else(|| fail(format!("{f} is not a face use of {owner}: cannot determine its orientation")))?;
            let patch = cx.surface_patch(*f).ok_or_else(|| fail(format!("the surface patch of {f} is missing (is a triangle scheme set on it?)")))?;
            for (shape, local) in patch.elements.iter() {
                if shape != ElemShape::Tri {
                    return Err(fail(format!("{f} is not triangulated (found {shape:?}); set mesh.scheme trimesh on its faces")));
                }
                let key = |i: usize| patch.key(local[i]).ok_or_else(|| fail(format!("{f}: dangling node index in its patch")));
                let (a, b, c) = (key(0)?, key(1)?, key(2)?);
                tri_keys.push(if side == Side::Right { [a, c, b] } else { [a, b, c] });
                tri_face.push(*f);
            }
        }
        // Dense point ids in ascending key order.
        let keys: BTreeSet<NodeKey> = tri_keys.iter().flatten().copied().collect();
        let keys: Vec<NodeKey> = keys.into_iter().collect();
        let id_of: BTreeMap<NodeKey, u32> = keys.iter().enumerate().map(|(i, k)| (*k, i as u32)).collect();
        let mut points: Vec<[f64; 3]> = Vec::with_capacity(keys.len());
        for k in &keys {
            let q = cx.point(*k).ok_or_else(|| fail(format!("node {k:?} has no position")))?;
            points.push([q.x, q.y, q.z]);
        }
        let tris: Vec<[u32; 3]> = tri_keys.iter().map(|t| t.map(|k| id_of[&k])).collect();
        let input = SurfaceInput { points, tris };

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
        let mesh = tetrahedralize(&input, &opts).map_err(|e| explain(&e, target, &input, &tri_keys, &tri_face, &keys))?;
        Ok(build_patch(target, owner, &mesh, &keys))
    }
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
        TetError::DuplicatePoint { .. } => "two mesh nodes coincide: merge or remove the duplicate".to_owned(),
        TetError::OutOfRange(_) => "scale the model so that coordinates lie within 2^-100 .. 2^100".to_owned(),
        TetError::SteinerBudget { .. } => "increase the element size or simplify the geometry".to_owned(),
        _ => "see the error text".to_owned(),
    };
    GenerateFailure::fatal(target, NAME, format!("{e}{at}; suggested action: {action}")).with_nodes(nodes)
}

/// The node keys locating `e` (the offending triangle's corners, the edge's or points' keys) and
/// the offending triangle, if any.
fn locate(e: &TetError, input: &SurfaceInput, tri_keys: &[[NodeKey; 3]], key_of: &dyn Fn(u32) -> NodeKey) -> (Vec<NodeKey>, Option<u32>) {
    let tri = match e {
        TetError::InvertedShell { tri } | TetError::DegenerateTri { tri } | TetError::RecoveryFailed { tri, .. } => Some(*tri),
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
fn build_patch(target: Target, owner: EntityId, mesh: &TetMesh, keys: &[NodeKey]) -> Patch {
    let mut pb = PatchBuilder::new(target);
    let mut local: Vec<u32> = Vec::with_capacity(mesh.points.len());
    for (i, p) in mesh.points.iter().enumerate() {
        if i < mesh.n_input {
            local.push(pb.refer(keys[i]));
        } else {
            let k = (i - mesh.n_input) as u32;
            local.push(pb.node(NodeKey::new(owner, k), Point3::new(p[0], p[1], p[2]), Param::None));
        }
    }
    for t in &mesh.tets {
        pb.push(ElemShape::Cell(CellShape::Tet), &t.map(|i| local[i as usize]));
    }
    pb.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    #[test]
    fn recovery_failure_names_the_face_and_carries_the_triangle_keys() {
        let face = EntityId::new(EntityKind::TopoFace, 12).unwrap();
        let vol = EntityId::new(EntityKind::TopoVolume, 1).unwrap();
        let keys: Vec<NodeKey> = (0..4).map(|i| NodeKey::new(face, i)).collect();
        let input = SurfaceInput { points: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], tris: vec![[0, 1, 2], [0, 3, 1]] };
        let tri_keys = vec![[keys[0], keys[1], keys[2]], [keys[0], keys[3], keys[1]]];
        let tri_face = vec![face, face];
        let e = TetError::RecoveryFailed { tri: 1, edge: Some([0, 3]), reason: "forced".into() };
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
