//! Webcut: split volumes with a plane or a cylinder.
//!
//! The tool is a bare surface. Each target volume's faces are cut along the intersection with the
//! tool (the same kernel as the booleans), the cut edges are chained into closed loops, the loops
//! become the cap face(s) on the tool surface, and `split_volume` separates the volume. Every
//! piece becomes its own body; the caps are shared (merged) by the two pieces unless `merge` is
//! off, in which case each piece gets its own cap (`unmerge_face`, Cubit `nomerge`).

use std::collections::{BTreeMap, BTreeSet};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Frame, Plane, SurfaceDef};
use sm_topo_api::{CoedgeSpec, Dim, Sense, ShellAssign, Side, SurfaceCarrier, TopologyEdit, TopologyView, TopologyViewExt};

use crate::arrange;
use crate::bodies;
use crate::error::BooleanError;
use crate::faceclass;
use crate::geo;
use crate::pieces::{self, Other, PairSpec, Share};

/// Which side(s) of the tool to keep.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum KeepSide {
    /// Both halves (a split).
    #[default]
    Both,
    /// Only the half the tool normal points to (a section).
    Positive,
    /// Only the half opposite to the tool normal.
    Negative,
}

/// What a webcut cuts with. The positive side is the one the tool's surface normal points to.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum CutTool {
    /// An (unbounded) plane through `origin` with normal `normal`.
    Plane {
        /// A point of the plane.
        origin: Point3,
        /// The normal (need not be unit); the positive side is where it points.
        normal: Vector3,
    },
    /// A surface: a plane or a cylinder (a cylinder's positive side is its outside). `window` is a
    /// world box `[xmin, ymin, zmin, xmax, ymax, zmax]` that bounds the tool; it must contain the
    /// targets (`None` takes the targets' box).
    Surface {
        /// The surface definition.
        surface: SurfaceDef,
        /// Bounding box of the tool.
        window: Option<[f64; 6]>,
    },
}

/// Options of [`webcut`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct WebcutOptions {
    /// Share the cut face between the pieces (default true); false is Cubit `nomerge`.
    pub merge: bool,
    /// Imprint the pieces on the other bodies of the model (not available yet).
    pub imprint: bool,
    /// Which halves to keep.
    pub keep: KeepSide,
    /// Delete the tool body (only for body tools, which come with the stretch variants).
    pub consume_tool: bool,
}

impl Default for WebcutOptions {
    fn default() -> Self {
        Self { merge: true, imprint: false, keep: KeepSide::Both, consume_tool: false }
    }
}

/// What [`webcut`] produced; every id is alive.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WebcutOutcome {
    /// Bodies on the side the tool normal points to.
    pub positive: Vec<EntityId>,
    /// Bodies on the other side.
    pub negative: Vec<EntityId>,
    /// Bodies that are on neither side (empty for plane, surface and cylinder tools).
    pub bodies: Vec<EntityId>,
    /// The volumes of every piece.
    pub volumes: Vec<EntityId>,
    /// The cut faces, `record_generated` on the tool surface.
    pub cut_faces: Vec<EntityId>,
}

fn unit_or(v: Vector3) -> Result<Vector3, BooleanError> {
    geo::unit(v).ok_or_else(|| BooleanError::Invalid("the tool normal is zero or not finite".into()))
}

/// Split `targets` (volumes or bodies) with `tool`.
///
/// Runs inside an open operation. Each target body must hold only target volumes; every resulting
/// volume becomes its own body (the old body is deleted and `record_split` names the new ones).
/// A target the tool does not cross is reported on the side it lies on and is not changed.
///
/// # Errors
/// `Unsupported` for non-manifold targets, for tools other than planes and cylinders, for
/// `imprint`, and when the cut curves do not form simple closed loops; `Topo(CutDoesNotSeparate)`
/// when the tool does not split a volume in two.
pub fn webcut<E: TopologyEdit + ?Sized>(topo: &mut E, targets: &[EntityId], tool: &CutTool, o: &WebcutOptions) -> Result<WebcutOutcome, BooleanError> {
    if o.imprint {
        return Err(BooleanError::Unsupported("webcut with imprint is not available yet"));
    }
    let vols = bodies::volumes_of(&*topo, targets)?;
    if vols.is_empty() {
        return Err(BooleanError::Invalid("webcut needs at least one target".into()));
    }
    for &v in &vols {
        bodies::check_manifold(&*topo, v)?;
    }
    let tool_def: SurfaceDef = match tool {
        CutTool::Plane { origin, normal } => {
            let n = unit_or(*normal)?;
            SurfaceDef::Plane(Plane { frame: Frame::from_z(*origin, n)? })
        }
        CutTool::Surface { surface, .. } => surface.clone(),
    };
    match &tool_def {
        SurfaceDef::Plane(_) | SurfaceDef::Cylinder(_) => {}
        _ => return Err(BooleanError::Unsupported("webcut supports plane and cylinder tools")),
    }
    // Every target body must be entirely made of target volumes.
    let target_set: BTreeSet<EntityId> = vols.iter().copied().collect();
    let mut old_bodies: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for &v in &vols {
        for inc in topo.coboundary(v) {
            if topo.dim_of(inc.cell) == Some(Dim::Body) {
                old_bodies.entry(inc.cell).or_default().push(v);
            }
        }
    }
    for (&b, _) in &old_bodies {
        let members: Vec<EntityId> = topo.boundary(b).iter().map(|e| e.cell).collect();
        if members.iter().any(|m| !target_set.contains(m)) {
            return Err(BooleanError::Unsupported("webcut of some volumes of a body that holds other cells: delete or separate them first"));
        }
    }
    let tool_handle = topo.shapes_mut()?.add_surface(tool_def.clone())?;
    let lin = topo.tolerance().linear;
    let mut out = WebcutOutcome::default();
    // child volumes by (positive?, volume) in creation order, and the old body of each target.
    struct Child {
        volume: EntityId,
        positive: bool,
    }
    let mut per_body: BTreeMap<EntityId, Vec<Child>> = BTreeMap::new();
    let mut untouched: Vec<(EntityId, bool)> = Vec::new();
    let mut cut_faces: Vec<EntityId> = Vec::new();
    for &v in &vols {
        let faces: Vec<EntityId> = topo.volume_faces(v).to_vec();
        let pairs: Vec<PairSpec> = faces.iter().map(|&f| PairSpec { a: f, b: Other::Surface(tool_def.clone()) }).collect();
        let computed = pieces::compute(&*topo, &pairs, Share::Shared, &|_| 0, 10.0 * topo.tolerance().linear_max)?;
        let body = old_bodies.iter().find(|(_, vs)| vs.contains(&v)).map(|(b, _)| *b);
        if computed.pieces.is_empty() {
            // The tool does not cross the volume: report the side it is on.
            let Some(positive) = side_of_volume(&*topo, v, tool_handle, lin)? else {
                return Err(BooleanError::Unsupported("cannot tell which side of the tool the volume lies on"));
            };
            untouched.push((v, positive));
            continue;
        }
        let ps: Vec<&pieces::Piece> = computed.pieces.iter().collect();
        let applied = arrange::apply(topo, &ps, lin, &faces)?;
        // The cut edges: new ones and reused boundary edges, ascending and distinct.
        let mut cut_edges: BTreeSet<EntityId> = BTreeSet::new();
        for (e, _) in &applied.cut_edges {
            cut_edges.insert(*e);
        }
        let caps = match &tool_def {
            SurfaceDef::Plane(pl) => build_plane_caps(topo, &cut_edges, pl, tool_handle)?,
            SurfaceDef::Cylinder(_) => return Err(BooleanError::Unsupported("cylinder caps are not built yet")),
            _ => unreachable!("checked above"),
        };
        for &c in &caps {
            topo.record_generated(c, tool_handle)?;
        }
        // Which child is on which side, and where the cavities go.
        let outer_lowest = topo.boundary(v).iter().filter(|e| e.ring == 0).map(|e| e.cell).min();
        let mut assign: Vec<(u16, u8)> = Vec::new();
        let voids: Vec<u16> = {
            let mut r: Vec<u16> = topo.boundary(v).iter().map(|e| e.ring).filter(|r| *r != 0).collect();
            r.sort();
            r.dedup();
            r
        };
        if !voids.is_empty() {
            let Some(lowest) = outer_lowest else { return Err(BooleanError::Unsupported("volume without an outer shell")) };
            let child0_positive = side_of_face(&*topo, lowest, tool_handle, lin)?;
            for r in voids {
                let f = topo.boundary(v).iter().find(|e| e.ring == r).map(|e| e.cell);
                let Some(f) = f else { continue };
                let pos = side_of_face(&*topo, f, tool_handle, lin)?;
                assign.push((r, if pos == child0_positive { 0 } else { 1 }));
            }
        }
        let shell_assign = if assign.is_empty() { ShellAssign::None } else { ShellAssign::Explicit(&assign) };
        let [c0, c1] = topo.split_volume(v, &caps, shell_assign)?;
        // The cap normal points from child 0 to child 1: child 1 is on the tool's positive side
        // when the cap's own normal agrees with the tool normal.
        let cap_sense = match topo.carrier(caps[0]) {
            sm_topo_api::Carrier::Surface(sc) => sc.sense,
            _ => return Err(BooleanError::Unsupported("cap without a surface")),
        };
        let (pos_child, neg_child) = if cap_sense == Sense::Forward { (c1, c0) } else { (c0, c1) };
        cut_faces.extend(caps.iter().copied());
        if let Some(b) = body {
            per_body.entry(b).or_default().extend([Child { volume: neg_child, positive: false }, Child { volume: pos_child, positive: true }]);
        } else {
            per_body.entry(v).or_default().extend([Child { volume: neg_child, positive: false }, Child { volume: pos_child, positive: true }]);
        }
    }
    // Bodies: one per piece. The old body is deleted first (a volume may belong to one body).
    for (&old, pcs) in &per_body {
        let is_body = topo.dim_of(old) == Some(Dim::Body);
        let mut new_bodies: Vec<EntityId> = Vec::new();
        if is_body {
            topo.kill(old)?;
        }
        for p in pcs {
            if (o.keep == KeepSide::Positive && !p.positive) || (o.keep == KeepSide::Negative && p.positive) {
                continue;
            }
            let b = topo.make_body(&[p.volume])?;
            new_bodies.push(b);
            out.volumes.push(p.volume);
            if p.positive {
                out.positive.push(b);
            } else {
                out.negative.push(b);
            }
        }
        if is_body {
            topo.record_split(old, &new_bodies)?;
        }
        // Delete the dropped half (its cells that nothing else uses go with it).
        for p in pcs {
            if (o.keep == KeepSide::Positive && !p.positive) || (o.keep == KeepSide::Negative && p.positive) {
                topo.kill_cascade(p.volume)?;
            }
        }
    }
    for (v, positive) in untouched {
        if (o.keep == KeepSide::Positive && !positive) || (o.keep == KeepSide::Negative && positive) {
            // The whole target is on the dropped side.
            let body = old_bodies.iter().find(|(_, vs)| vs.contains(&v)).map(|(b, _)| *b);
            match body {
                Some(b) => {
                    topo.kill_cascade(b)?;
                }
                None => {
                    topo.kill_cascade(v)?;
                }
            }
            continue;
        }
        out.volumes.push(v);
        let body = old_bodies.iter().find(|(_, vs)| vs.contains(&v)).map(|(b, _)| *b).unwrap_or(v);
        if positive {
            out.positive.push(body);
        } else {
            out.negative.push(body);
        }
    }
    // Cut faces that are still alive; with `merge` off every piece gets its own copy.
    if !o.merge && o.keep == KeepSide::Both {
        let mut extra = Vec::new();
        for &c in &cut_faces {
            let right = topo.face_sides(c).map(|s| s.right);
            if matches!(right, Some(sm_topo_api::Region::Volume(_))) {
                let n = topo.unmerge_face(c, Side::Right)?;
                topo.record_generated(n, tool_handle)?;
                extra.push(n);
            }
        }
        cut_faces.extend(extra);
    }
    cut_faces.retain(|f| topo.is_alive(*f));
    cut_faces.sort();
    out.cut_faces = cut_faces;
    out.positive.sort();
    out.negative.sort();
    out.volumes.retain(|v| topo.is_alive(*v));
    out.volumes.sort();
    Ok(out)
}

/// True if the tool's positive side holds the volume (by the side of a face interior point).
fn side_of_volume<V: TopologyView + ?Sized>(view: &V, v: EntityId, tool: EntityId, lin: f64) -> Result<Option<bool>, BooleanError> {
    for &f in view.volume_faces(v) {
        if let Some(s) = try_side_of_face(view, f, tool, lin)? {
            return Ok(Some(s));
        }
    }
    Ok(None)
}

fn side_of_face<V: TopologyView + ?Sized>(view: &V, f: EntityId, tool: EntityId, lin: f64) -> Result<bool, BooleanError> {
    try_side_of_face(view, f, tool, lin)?.ok_or(BooleanError::Unsupported("a face lies on the tool surface"))
}

fn try_side_of_face<V: TopologyView + ?Sized>(view: &V, f: EntityId, tool: EntityId, lin: f64) -> Result<Option<bool>, BooleanError> {
    let Some(imp) = view.shapes().implicit(tool) else { return Err(BooleanError::Unsupported("the tool has no implicit form")) };
    let Some(p) = faceclass::interior_point(view, f)? else { return Ok(None) };
    let val = imp.value(p);
    Ok((val.abs() > 10.0 * lin).then_some(val > 0.0))
}

// ---------------------------------------------------------------------------------------------
// Caps
// ---------------------------------------------------------------------------------------------

/// A closed loop of cut edges as coedges.
struct Loop {
    edges: Vec<CoedgeSpec>,
}

fn extract_loops<E: TopologyEdit + ?Sized>(topo: &E, edges: &BTreeSet<EntityId>) -> Result<Vec<Loop>, BooleanError> {
    let mut at: BTreeMap<EntityId, Vec<EntityId>> = BTreeMap::new();
    for &e in edges {
        let (s, t) = topo.edge_ends(e).ok_or(BooleanError::Unsupported("a cut edge has no ends"))?;
        at.entry(s).or_default().push(e);
        if t != s {
            at.entry(t).or_default().push(e);
        }
    }
    for list in at.values() {
        if list.len() != 2 && !(list.len() == 1) {
            return Err(BooleanError::Unsupported("the cut curves branch: they do not form simple closed loops"));
        }
    }
    let mut used: BTreeSet<EntityId> = BTreeSet::new();
    let mut loops = Vec::new();
    for &e0 in edges {
        if used.contains(&e0) {
            continue;
        }
        let (s0, t0) = topo.edge_ends(e0).ok_or(BooleanError::Unsupported("a cut edge has no ends"))?;
        let mut seq = vec![CoedgeSpec { edge: e0, sense: Sense::Forward }];
        used.insert(e0);
        let mut cur = t0;
        while cur != s0 {
            let next = at.get(&cur).and_then(|l| l.iter().copied().find(|e| !used.contains(e)));
            let Some(e) = next else { return Err(BooleanError::Unsupported("the cut curves do not close (the tool does not separate the volume)")) };
            used.insert(e);
            let (s, t) = topo.edge_ends(e).ok_or(BooleanError::Unsupported("a cut edge has no ends"))?;
            if s == cur {
                seq.push(CoedgeSpec { edge: e, sense: Sense::Forward });
                cur = t;
            } else {
                seq.push(CoedgeSpec { edge: e, sense: Sense::Reversed });
                cur = s;
            }
        }
        loops.push(Loop { edges: seq });
    }
    Ok(loops)
}

/// Sampled polygon of a loop in the plane frame, `(x, y)` of the local coordinates.
fn plane_polygon<E: TopologyEdit + ?Sized>(topo: &E, lp: &Loop, frame: &Frame) -> Vec<[f64; 2]> {
    let mut poly = Vec::new();
    for ce in &lp.edges {
        for k in 0..16 {
            let f = k as f64 / 16.0;
            let fr = if ce.sense == Sense::Forward { f } else { 1.0 - f };
            if let Some(p) = geo::edge_sample(topo, ce.edge, fr) {
                let l = frame.to_local(p);
                poly.push([l.x, l.y]);
            }
        }
    }
    poly
}

fn point_in_polygon(poly: &[[f64; 2]], q: [f64; 2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

fn reversed(lp: &Loop) -> Loop {
    Loop { edges: lp.edges.iter().rev().map(|c| CoedgeSpec { edge: c.edge, sense: c.sense.flip() }).collect() }
}

fn build_plane_caps<E: TopologyEdit + ?Sized>(topo: &mut E, edges: &BTreeSet<EntityId>, plane: &Plane, tool: EntityId) -> Result<Vec<EntityId>, BooleanError> {
    let loops = extract_loops(&*topo, edges)?;
    if loops.is_empty() {
        return Err(BooleanError::Unsupported("the tool produced no cut curves"));
    }
    let polys: Vec<Vec<[f64; 2]>> = loops.iter().map(|l| plane_polygon(&*topo, l, &plane.frame)).collect();
    let areas: Vec<f64> = polys.iter().map(|p| crate::poly2::signed_area(p)).collect();
    // Nesting depth and parent (smallest containing loop).
    let n = loops.len();
    let mut parent: Vec<Option<usize>> = vec![None; n];
    let mut depth = vec![0usize; n];
    for i in 0..n {
        let probe = polys[i][0];
        let mut containers: Vec<usize> = (0..n).filter(|&j| j != i && areas[j].abs() > areas[i].abs() && point_in_polygon(&polys[j], probe)).collect();
        containers.sort_by(|&a, &b| areas[a].abs().total_cmp(&areas[b].abs()));
        depth[i] = containers.len();
        parent[i] = containers.first().copied();
    }
    let mut caps = Vec::new();
    for i in 0..n {
        if depth[i] % 2 != 0 {
            continue;
        }
        // Outer loop counter-clockwise seen from the plane normal.
        let outer = if areas[i] >= 0.0 { Loop { edges: loops[i].edges.clone() } } else { reversed(&loops[i]) };
        let mut rings: Vec<Loop> = vec![outer];
        for j in 0..n {
            if parent[j] == Some(i) && depth[j] == depth[i] + 1 {
                rings.push(if areas[j] <= 0.0 { Loop { edges: loops[j].edges.clone() } } else { reversed(&loops[j]) });
            }
        }
        let specs: Vec<&[CoedgeSpec]> = rings.iter().map(|r| r.edges.as_slice()).collect();
        let cap = topo.make_face(&specs, Some(SurfaceCarrier { surface: tool, sense: Sense::Forward }))?;
        caps.push(cap);
    }
    Ok(caps)
}
