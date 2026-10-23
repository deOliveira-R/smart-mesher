//! Stub scheme implementors used by the integration tests: an equal curve scheme, a map surface
//! scheme (vertex types + interval constraints) and a 1-to-1 sweep volume scheme (source /
//! target / linking roles), plus a scheme that always fails in `apply` (fallback tests).
#![allow(dead_code)]

use sm_core::{Point3, Vector3};
use sm_mesh_api::{CellShape, Param};
use sm_scheme_api::*;
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, Sense};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

pub fn brick_complex() -> (CellComplex, BrickIds) {
    let mut t = CellComplex::new();
    let ids = with_op(&mut t, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).expect("brick fixture");
    (t, ids)
}

pub fn key(id: sm_core::EntityId) -> ShapeKey {
    ShapeKey::Cell(id)
}

// ---------------------------------------------------------------- curve.equal

pub struct EqualCurve;

impl SchemeCore for EqualCurve {
    fn id(&self) -> &str {
        "curve.equal"
    }
    fn dim(&self) -> Dim {
        Dim::Edge
    }
    fn caps(&self) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>) -> Applicability {
        Applicability::yes(0.5, "any curve")
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        Ok(Plan::single(self.id(), cx.target()))
    }
}

impl CurveScheme for EqualCurve {
    fn apply(&self, input: &CurveInput<'_>) -> Result<CurvePatch, SchemeError> {
        let n = input.intervals;
        Ok(CurvePatch { fractions: (1..n).map(|i| f64::from(i) / f64::from(n)).collect() })
    }
}

// ---------------------------------------------------------------- surface.map

pub struct MapSurface;

struct MapSides {
    corners: Vec<usize>,
    sides: [Vec<ShapeKey>; 4],
    types: Vec<CornerAssignment>,
}

fn analyze(cx: &PlanCtx<'_>) -> Result<MapSides, String> {
    let f = cx.target();
    let loops = cx.shape.face_loops(f).map_err(|e| e.to_string())?;
    if loops.len() != 1 {
        return Err(format!("map needs one loop, {f} has {}", loops.len()));
    }
    let types = cx.corners.pick(cx.shape, f, &loops, cx.vertex_types).map_err(|e| e.to_string())?;
    let (uses, types) = (&loops[0].uses, &types[0]);
    if let Some(t) = types.iter().find(|t| matches!(t, VertexType::Corner | VertexType::Reversal)) {
        return Err(format!("map cannot take a {t:?} vertex (use submap)"));
    }
    let corners: Vec<usize> = (0..uses.len()).filter(|&i| types[i] == VertexType::End).collect();
    if corners.len() != 4 {
        return Err(format!("map needs 4 End vertices, found {}", corners.len()));
    }
    let mut sides: [Vec<ShapeKey>; 4] = Default::default();
    for s in 0..4 {
        let (a, b) = (corners[s], corners[(s + 1) % 4]);
        let mut i = a;
        loop {
            sides[s].push(uses[i].curve);
            i = (i + 1) % uses.len();
            if i == b {
                break;
            }
        }
    }
    let assigned = uses
        .iter()
        .zip(types)
        .filter_map(|(u, t)| {
            let corner = u.corner?;
            let firmness = cx.vertex_types.iter().find(|a| a.corner == corner).map_or(Firmness::Soft, |a| a.firmness);
            Some(CornerAssignment { corner, vertex_type: *t, firmness })
        })
        .collect();
    Ok(MapSides { corners, sides, types: assigned })
}

impl SchemeCore for MapSurface {
    fn id(&self) -> &str {
        "surface.map"
    }
    fn dim(&self) -> Dim {
        Dim::Face
    }
    fn caps(&self) -> Caps {
        Caps::QUAD | Caps::STRUCTURED
    }
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability {
        match analyze(cx) {
            Ok(_) => Applicability::yes(0.9, "one loop with 4 End vertices"),
            Err(m) => Applicability::no(m),
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        let a = analyze(cx).map_err(|m| SchemeError::failed(FailureKind::Rejected, m))?;
        let mut p = Plan::single(self.id(), cx.target());
        p.intervals.push(IntervalConstraint::sides_equal(&a.sides[0], &a.sides[2]));
        p.intervals.push(IntervalConstraint::sides_equal(&a.sides[1], &a.sides[3]));
        p.vertex_types = a.types;
        p.data = Params::new().with("corners", ParamValue::List(a.corners.iter().map(|&c| ParamValue::Int(c as i64)).collect()));
        Ok(p)
    }
}

fn arc_fractions(mesh: &CommittedMesh, nodes: &[NodeId]) -> Vec<f64> {
    let mut acc = vec![0.0];
    for w in nodes.windows(2) {
        let d = (mesh.point(w[1]) - mesh.point(w[0])).norm();
        acc.push(acc.last().copied().unwrap_or(0.0) + d);
    }
    let total = acc.last().copied().unwrap_or(1.0).max(f64::MIN_POSITIVE);
    acc.iter().map(|x| x / total).collect()
}

impl SurfaceScheme for MapSurface {
    fn apply(&self, input: &SurfaceInput<'_>) -> Result<SurfacePatch, SchemeError> {
        let l = &input.loops[0];
        let corners: Vec<usize> = match input.plan.data.get("corners") {
            Some(ParamValue::List(v)) => v.iter().filter_map(|x| if let ParamValue::Int(i) = x { Some(*i as usize) } else { None }).collect(),
            _ => return Err(SchemeError::failed(FailureKind::Internal, "plan lost its corners")),
        };
        let n = l.nodes.len();
        let start: Vec<usize> = corners.iter().map(|&c| l.uses[c].first as usize).collect();
        let side = |s: usize| -> Vec<NodeId> {
            let (a, b) = (start[s], start[(s + 1) % 4]);
            let len = (b + n - a) % n;
            (0..=len).map(|k| l.nodes[(a + k) % n]).collect()
        };
        let s: [Vec<NodeId>; 4] = [side(0), side(1), side(2), side(3)];
        if s[0].len() != s[2].len() || s[1].len() != s[3].len() {
            return Err(SchemeError::failed(FailureKind::Constraint, "opposite sides have different interval sums"));
        }
        let (ni, nj) = (s[0].len() - 1, s[1].len() - 1);
        let bottom = |i: usize| s[0][i];
        let right = |j: usize| s[1][j];
        let top = |i: usize| s[2][ni - i];
        let left = |j: usize| s[3][nj - j];
        let (fb, fr, ft, fl) = (arc_fractions(input.mesh, &s[0]), arc_fractions(input.mesh, &s[1]), arc_fractions(input.mesh, &s[2]), arc_fractions(input.mesh, &s[3]));
        let p = |id: NodeId| input.mesh.point(id);
        let (p00, p10, p11, p01) = (p(bottom(0)), p(bottom(ni)), p(top(ni)), p(top(0)));
        let mut grid = Vec::with_capacity((ni + 1) * (nj + 1));
        let mut nodes = Vec::new();
        for j in 0..=nj {
            for i in 0..=ni {
                let on_boundary = if j == 0 {
                    Some(bottom(i))
                } else if j == nj {
                    Some(top(i))
                } else if i == 0 {
                    Some(left(j))
                } else if i == ni {
                    Some(right(j))
                } else {
                    None
                };
                if let Some(b) = on_boundary {
                    grid.push(PatchRef::Old(b));
                    continue;
                }
                let v = 0.5 * (fl[nj - j].mul_add(-1.0, 1.0) + fr[j]);
                let u = 0.5 * (fb[i] + 1.0 - ft[ni - i]);
                let lerp = |a: Point3, b: Point3, t: f64| a + (b - a) * t;
                let edge = lerp(p(left(j)), p(right(j)), u) - Point3::new(0.0, 0.0, 0.0);
                let edge2 = lerp(p(bottom(i)), p(top(i)), v) - Point3::new(0.0, 0.0, 0.0);
                let bil = lerp(lerp(p00, p10, u), lerp(p01, p11, u), v) - Point3::new(0.0, 0.0, 0.0);
                let q: Vector3 = edge + edge2 - bil;
                let (pt, uv) = input.shape.snap_to_face(input.face, Point3::new(q.x, q.y, q.z))?;
                grid.push(PatchRef::New(nodes.len() as u32));
                nodes.push(NewNode { point: pt, on: input.face, param: uv.map_or(Param::None, Param::Surface) });
            }
        }
        let g = Grid { ni: ni as u32, nj: nj as u32, nodes: grid };
        let mut faces = FaceList::new();
        for j in 0..nj as u32 {
            for i in 0..ni as u32 {
                faces.push(&[g.at(i, j), g.at(i + 1, j), g.at(i + 1, j + 1), g.at(i, j + 1)]);
            }
        }
        Ok(SurfacePatch { nodes, faces, grid: Some(g) })
    }
}

// ---------------------------------------------------------------- volume.sweep (1-to-1)

pub struct Sweep;

struct Roles {
    source: ShapeKey,
    target: ShapeKey,
    linking: Vec<ShapeKey>,
}

fn roles(cx: &PlanCtx<'_>) -> Result<Roles, String> {
    let v = cx.target();
    let faces: Vec<ShapeKey> = cx.shape.volume_faces(v).map_err(|e| e.to_string())?.iter().map(|u| u.face).collect();
    let curves = |f: ShapeKey| -> Result<BTreeSet<ShapeKey>, String> { Ok(cx.shape.face_curves(f).map_err(|e| e.to_string())?.into_iter().collect()) };
    let check = |s: ShapeKey, t: ShapeKey| -> Result<Roles, String> {
        if s == t || !faces.contains(&s) || !faces.contains(&t) {
            return Err("source and target must be two faces of the volume".into());
        }
        let (cs, ct) = (curves(s)?, curves(t)?);
        if !cs.is_disjoint(&ct) {
            return Err("source and target share a curve".into());
        }
        let linking: Vec<ShapeKey> = faces.iter().copied().filter(|f| *f != s && *f != t).collect();
        for f in &linking {
            let cf = curves(*f)?;
            if cf.is_disjoint(&cs) || cf.is_disjoint(&ct) {
                return Err(format!("{f} does not link source to target"));
            }
        }
        Ok(Roles { source: s, target: t, linking })
    };
    let one = |k: Option<Vec<ShapeKey>>| k.and_then(|v| if v.len() == 1 { Some(v[0]) } else { None });
    if let (Some(s), Some(t)) = (one(cx.params.keys("source")), one(cx.params.keys("target"))) {
        return check(s, t);
    }
    if let Some(h) = cx.shape.sweep_hint(v)
        && let (Some(s), Some(t)) = (one(Some(h.source)), one(Some(h.target)))
    {
        return check(s, t);
    }
    for (i, &s) in faces.iter().enumerate() {
        for &t in &faces[i + 1..] {
            if let Ok(r) = check(s, t) {
                return Ok(r);
            }
        }
    }
    Err("no source/target pair".into())
}

impl SchemeCore for Sweep {
    fn id(&self) -> &str {
        "volume.sweep"
    }
    fn dim(&self) -> Dim {
        Dim::Volume
    }
    fn caps(&self) -> Caps {
        Caps::HEX | Caps::MESHES_CHILDREN
    }
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability {
        match roles(cx) {
            Ok(r) => Applicability::yes(0.7, format!("1-1 sweep {} -> {}", r.source, r.target)),
            Err(m) => Applicability::no(m),
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        let r = roles(cx).map_err(|m| SchemeError::failed(FailureKind::Rejected, m))?;
        let mut p = Plan::single(self.id(), cx.target());
        p.claims.push(r.target);
        p.require(r.source, Requirement::Role(Role::Source));
        p.require(r.source, Requirement::Caps { need: Caps::QUAD, firmness: Firmness::Hard });
        p.require(r.target, Requirement::Role(Role::Target));
        let mut ends: BTreeSet<ShapeKey> = cx.shape.face_curves(r.source)?.into_iter().collect();
        ends.extend(cx.shape.face_curves(r.target)?);
        let mut link_curves = BTreeSet::new();
        for &f in &r.linking {
            p.require(f, Requirement::Role(Role::Linking));
            p.require(f, Requirement::Caps { need: Caps::QUAD | Caps::STRUCTURED, firmness: Firmness::Hard });
            link_curves.extend(cx.shape.face_curves(f)?.into_iter().filter(|c| !ends.contains(c)));
        }
        let lc: Vec<ShapeKey> = link_curves.into_iter().collect();
        for w in lc.windows(2) {
            p.intervals.push(IntervalConstraint::equal(w[0], w[1]));
        }
        p.data = Params::new()
            .with("source", ParamValue::Key(r.source))
            .with("target", ParamValue::Key(r.target))
            .with("linking", ParamValue::List(r.linking.iter().map(|k| ParamValue::Key(*k)).collect()));
        Ok(p)
    }
}

fn columns_of(g: &Grid<NodeId>, on_source: &BTreeSet<NodeId>) -> Option<Vec<Vec<NodeId>>> {
    let (ni, nj) = (g.ni, g.nj);
    // Side k of the grid as columns walking away from it: (k, s) -> (i, j) of step s on column k.
    let at = |side: u8, k: u32, s: u32| match side {
        0 => g.at(k, s),
        1 => g.at(k, nj - s),
        2 => g.at(s, k),
        _ => g.at(ni - s, k),
    };
    for side in 0..4u8 {
        let (len, depth) = if side < 2 { (ni, nj) } else { (nj, ni) };
        if (0..=len).all(|k| on_source.contains(&at(side, k, 0))) {
            return Some((0..=len).map(|k| (0..=depth).map(|s| at(side, k, s)).collect()).collect());
        }
    }
    None
}

impl VolumeScheme for Sweep {
    fn apply(&self, input: &VolumeInput<'_>) -> Result<VolumePatch, SchemeError> {
        let bad = |m: String| SchemeError::failed(FailureKind::Constraint, m);
        let d = &input.plan.data;
        let one = |k: &str| d.keys(k).and_then(|v| v.first().copied()).ok_or_else(|| SchemeError::failed(FailureKind::Internal, format!("plan lost {k}")));
        let (src, tgt) = (one("source")?, one("target")?);
        let linking = d.keys("linking").unwrap_or_default();
        let v = input.targets[0];
        let sense = |f: ShapeKey| input.faces[0].iter().find(|u| u.face == f).map(|u| u.sense).ok_or_else(|| bad(format!("{f} is not a face of {v}")));
        let (ss, ts) = (sense(src)?, sense(tgt)?);
        let sp = input.mesh.surface(src).ok_or_else(|| bad("source not meshed".into()))?;
        let on_source: BTreeSet<NodeId> = sp.faces.nodes.iter().copied().collect();
        let mut columns: BTreeMap<NodeId, Vec<NodeId>> = BTreeMap::new();
        for lf in &linking {
            let g = grid_of(input.mesh, *lf).ok_or_else(|| bad(format!("linking face {lf} has no grid")))?;
            let cols = columns_of(g, &on_source).ok_or_else(|| bad(format!("linking face {lf} has no grid side on the source")))?;
            for c in cols {
                match columns.get(&c[0]) {
                    Some(old) if *old != c => return Err(bad("linking faces disagree on a column".into())),
                    _ => {
                        columns.insert(c[0], c);
                    }
                }
            }
        }
        let layers = columns.values().next().map(|c| c.len() - 1).ok_or_else(|| bad("no columns".into()))?;
        if columns.values().any(|c| c.len() != layers + 1) {
            return Err(bad("columns have different layer counts".into()));
        }
        let p = |n: NodeId| input.mesh.point(n);
        let disp: Vec<Vector3> = (0..=layers)
            .map(|k| {
                let s = columns.values().fold(Vector3::new(0.0, 0.0, 0.0), |a, c| a + (p(c[k]) - p(c[0])));
                s * (1.0 / columns.len() as f64)
            })
            .collect();
        let mut patch = VolumePatch::default();
        let mut layer: BTreeMap<NodeId, Vec<PatchRef>> = BTreeMap::new();
        for &n in &on_source {
            if let Some(c) = columns.get(&n) {
                layer.insert(n, c.iter().map(|&x| PatchRef::Old(x)).collect());
                continue;
            }
            let mut refs = vec![PatchRef::Old(n)];
            for (k, dk) in disp.iter().enumerate().skip(1) {
                let q = p(n) + *dk;
                let node = if k == layers {
                    let (pt, uv) = input.shape.snap_to_face(tgt, q)?;
                    NewNode { point: pt, on: tgt, param: uv.map_or(Param::None, Param::Surface) }
                } else {
                    NewNode { point: q, on: v, param: Param::None }
                };
                refs.push(PatchRef::New(patch.nodes.len() as u32));
                patch.nodes.push(node);
            }
            layer.insert(n, refs);
        }
        let mut tfaces = FaceList::new();
        for q in sp.faces.iter() {
            let [a, b, c, e] = <[NodeId; 4]>::try_from(q).map_err(|_| SchemeError::Unsupported("stub sweep needs a quad source".into()))?;
            let base = if ss == Sense::Forward { [a, e, c, b] } else { [a, b, c, e] };
            for k in 0..layers {
                let nodes: Vec<PatchRef> = base.iter().map(|n| layer[n][k]).chain(base.iter().map(|n| layer[n][k + 1])).collect();
                patch.cells.push(CellShape::Hex, &nodes, 0);
            }
            let mut t: Vec<PatchRef> = [a, b, c, e].iter().map(|n| layer[n][layers]).collect();
            if ss == ts {
                t.reverse();
            }
            tfaces.push(&t);
        }
        patch.surfaces.push(ClaimedSurface { face: tgt, faces: tfaces, grid: None });
        Ok(patch)
    }
}

// ---------------------------------------------------------------- test.fail

pub struct AlwaysFails;

impl SchemeCore for AlwaysFails {
    fn id(&self) -> &str {
        "test.fail"
    }
    fn dim(&self) -> Dim {
        Dim::Volume
    }
    fn caps(&self) -> Caps {
        Caps::HEX
    }
    fn applicability(&self, _cx: &PlanCtx<'_>) -> Applicability {
        Applicability::yes(1.0, "always claims it can")
    }
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError> {
        Ok(Plan::single(self.id(), cx.target()))
    }
}

impl VolumeScheme for AlwaysFails {
    fn apply(&self, _input: &VolumeInput<'_>) -> Result<VolumePatch, SchemeError> {
        Err(SchemeError::failed(FailureKind::Quality, "inverted element (simulated)"))
    }
}

pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(SchemeImpl::Curve(Arc::new(EqualCurve))).expect("register");
    r.register(SchemeImpl::Surface(Arc::new(MapSurface))).expect("register");
    r.register(SchemeImpl::Volume(Arc::new(Sweep))).expect("register");
    r.register(SchemeImpl::Volume(Arc::new(AlwaysFails))).expect("register");
    r.set_auto(1, &["curve.equal"]).expect("auto");
    r.set_auto(2, &["surface.map"]).expect("auto");
    r.set_auto(3, &["volume.sweep"]).expect("auto");
    r
}

pub fn driver<'a>(registry: &'a SchemeRegistry, shape: &'a dyn ShapeView, intent: &'a MeshIntent) -> Driver<'a> {
    Driver { registry, shape, intent, solver: &SimpleSolver, corners: &AnglePicker, size: None, options: DriverOptions::default() }
}

/// Intent: sweep -z -> +z, size 0.25, a hard count of 3 on one z-edge.
pub fn sweep_intent(ids: &BrickIds) -> MeshIntent {
    let mut intent = MeshIntent::new();
    let vol = key(ids.volume);
    intent.set_scheme(vol, SchemeChoice::hard(SchemeSpec::new("volume.sweep").with("source", ParamValue::Key(key(ids.faces[4]))).with("target", ParamValue::Key(key(ids.faces[5])))));
    intent.set_size(vol, SizeSpec::Size { size: 0.25, firmness: Firmness::Soft });
    intent.set_size(key(ids.edges[8]), SizeSpec::Intervals { n: 3, firmness: Firmness::Hard });
    intent
}
