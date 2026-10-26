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
use sm_geom_api::{Cylinder, Frame, Interval, Line, Plane, SurfaceDef};
use sm_topo_api::CurveCarrier;
use sm_topo_api::{CoedgeSpec, Dim, Sense, Side, SurfaceCarrier, TopologyEdit, TopologyView, TopologyViewExt};

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
/// A target the tool does not cross is reported on the side it lies on and is not changed. That
/// includes a plane that only coincides with a face of the target (for example `z = 0` on a cube
/// above it): the target is returned in `positive` (or `negative`), the other role is empty and
/// the topology is untouched. This is deliberate, so that one tool can be applied to a stack of
/// bricks along their interface; callers that need a real cut check that both roles are non-empty.
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
    for &b in old_bodies.keys() {
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
        if computed.pieces.iter().all(|p| p.faces.is_empty()) {
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
            SurfaceDef::Cylinder(cy) => build_cylinder_caps(topo, &mut cut_edges, cy, tool_handle)?,
            _ => unreachable!("checked above"),
        };
        for &c in &caps {
            topo.record_generated(c, tool_handle)?;
        }
        // Which side of the tool each fragment is on, and the outward normal of the volume there.
        let faces_now: Vec<EntityId> = topo.volume_faces(v).to_vec();
        struct Frag {
            face: EntityId,
            pt: Point3,
            n_out: Vector3,
            positive: bool,
        }
        let fcache = faceclass::FaceCache::new();
        let mut frags: Vec<Frag> = Vec::with_capacity(faces_now.len());
        for &f in &faces_now {
            let p = faceclass::interior_point(&*topo, &fcache, f)?.ok_or(BooleanError::Unsupported("cannot find a point inside a fragment"))?;
            let (nf, _, _) = geo::face_normal_near(&*topo, f, p).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
            let sense = topo.face_use_in(f, v).first().copied().unwrap_or(Sense::Forward);
            let n_out = nf * sense.sign();
            let imp = topo.shapes().implicit(tool_handle).ok_or(BooleanError::Unsupported("the tool has no implicit form"))?;
            let val = imp.value(p);
            let positive = if val.abs() > 10.0 * lin {
                val > 0.0
            } else {
                // The fragment lies on the tool surface: the volume is on the side opposite to
                // its outward normal.
                let g = geo::unit(imp.gradient(p)).ok_or(BooleanError::Unsupported("degenerate tool gradient"))?;
                n_out.dot(g) < 0.0
            };
            frags.push(Frag { face: f, pt: p, n_out, positive });
        }
        if frags.iter().all(|f| f.positive) || frags.iter().all(|f| !f.positive) {
            return Err(BooleanError::Unsupported("the tool does not separate the volume"));
        }
        // Free the volume; freeing a face may reverse it, so the sides are derived afterwards.
        for &f in &faces_now {
            topo.remove_face_use(v, f, Side::Both)?;
        }
        topo.kill(v)?;
        let mut pos_uses: Vec<(EntityId, Side)> = Vec::new();
        let mut neg_uses: Vec<(EntityId, Side)> = Vec::new();
        for fr in &frags {
            let (nf, _, _) = geo::face_normal_near(&*topo, fr.face, fr.pt).ok_or(BooleanError::Unsupported("cannot evaluate a face normal"))?;
            let side = if nf.dot(fr.n_out) > 0.0 { Side::Left } else { Side::Right };
            if fr.positive { pos_uses.push((fr.face, side)) } else { neg_uses.push((fr.face, side)) }
        }
        // A cap's normal points to the positive side: the negative piece is on its left.
        for &c in &caps {
            neg_uses.push((c, Side::Left));
            pos_uses.push((c, Side::Right));
        }
        let mut groups: Vec<(bool, crate::assemble::ShellGroup)> = Vec::new();
        for g in crate::assemble::group_shells(&*topo, &neg_uses, v, "webcut")? {
            groups.push((false, g));
        }
        for g in crate::assemble::group_shells(&*topo, &pos_uses, v, "webcut")? {
            groups.push((true, g));
        }
        groups.sort_by_key(|(_, g)| g.outer.iter().map(|u| u.0).min());
        let mut children: Vec<Child> = Vec::new();
        for (positive, g) in &groups {
            let vol = crate::assemble::make_volumes(topo, std::slice::from_ref(g))?[0];
            children.push(Child { volume: vol, positive: *positive });
        }
        let child_vols: Vec<EntityId> = children.iter().map(|c| c.volume).collect();
        let mut child_faces: BTreeSet<EntityId> = BTreeSet::new();
        for &cv in &child_vols {
            child_faces.extend(topo.volume_faces(cv).iter().copied());
        }
        let cf: Vec<EntityId> = child_faces.into_iter().collect();
        crate::pcurves::fix_faces(topo, &cf)?;
        topo.record_split(v, &child_vols)?;
        cut_faces.extend(caps.iter().copied());
        per_body.entry(body.unwrap_or(v)).or_default().extend(children);
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
            if let Some(sm_topo_api::Region::Volume(rv)) = right {
                let n = unmerge_cap(topo, c, rv)?;
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

fn try_side_of_face<V: TopologyView + ?Sized>(view: &V, f: EntityId, tool: EntityId, lin: f64) -> Result<Option<bool>, BooleanError> {
    let Some(imp) = view.shapes().implicit(tool) else { return Err(BooleanError::Unsupported("the tool has no implicit form")) };
    let Some(p) = faceclass::interior_point(view, &faceclass::FaceCache::new(), f)? else { return Ok(None) };
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

// ---------------------------------------------------------------------------------------------
// Cylinder caps
// ---------------------------------------------------------------------------------------------

fn cyl_uv(frame: &Frame, p: Point3) -> (f64, f64) {
    let l = frame.to_local(p);
    (l.y.atan2(l.x), l.z)
}

/// Sample a loop in the cylinder's `(u, v)` with `u` continuous along the loop. Returns the
/// samples `(u, v, point, coedge index, fraction)`.
fn cyl_trace<E: TopologyEdit + ?Sized>(topo: &E, lp: &Loop, frame: &Frame) -> Vec<(f64, f64, Point3, usize, f64)> {
    const S: usize = 16;
    let mut out: Vec<(f64, f64, Point3, usize, f64)> = Vec::new();
    let mut prev_u: Option<f64> = None;
    for (ci, ce) in lp.edges.iter().enumerate() {
        for k in 0..=S {
            let f = k as f64 / S as f64;
            let fr = if ce.sense == Sense::Forward { f } else { 1.0 - f };
            let Some(p) = geo::edge_sample(topo, ce.edge, fr) else { continue };
            let (mut u, v) = cyl_uv(frame, p);
            if let Some(pu) = prev_u {
                u += core::f64::consts::TAU * ((pu - u) / core::f64::consts::TAU).round();
            }
            prev_u = Some(u);
            out.push((u, v, p, ci, f));
        }
    }
    out
}

/// Total change of `u` around a loop, including the closing step (a multiple of 2 pi).
fn winding(trace: &[(f64, f64, Point3, usize, f64)]) -> f64 {
    let (Some(first), Some(last)) = (trace.first(), trace.last()) else { return 0.0 };
    let mut closing = first.0 - last.0;
    closing -= core::f64::consts::TAU * (closing / core::f64::consts::TAU).round();
    (last.0 + closing - first.0) / core::f64::consts::TAU
}

/// Make sure the loop has a vertex on the seam `u = 0` (a split of the edge where it crosses);
/// returns the seam vertex.
fn seam_vertex<E: TopologyEdit + ?Sized>(topo: &mut E, lp: &Loop, frame: &Frame, edges: &mut BTreeSet<EntityId>) -> Result<EntityId, BooleanError> {
    let lin = topo.tolerance().linear;
    let ly = |p: Point3| frame.to_local(p).y;
    let lx = |p: Point3| frame.to_local(p).x;
    for ce in &lp.edges {
        const S: usize = 32;
        let mut prev: Option<(f64, f64)> = None; // (fraction along the edge, ly)
        for k in 0..=S {
            let f = k as f64 / S as f64;
            let fr = if ce.sense == Sense::Forward { f } else { 1.0 - f };
            let p = geo::edge_sample(topo, ce.edge, fr).ok_or(BooleanError::Unsupported("a cut edge has no geometry"))?;
            let y = ly(p);
            if lx(p) > 0.0 {
                if y.abs() <= lin {
                    // At (or within tolerance of) the seam: an end vertex of the edge or a split.
                    let (s, t) = topo.edge_ends(ce.edge).ok_or(BooleanError::Unsupported("a cut edge has no ends"))?;
                    for v in [s, t] {
                        if let Some(q) = geo::vertex_point(topo, v)
                            && geo::dist(q, p) <= lin
                        {
                            return Ok(v);
                        }
                    }
                    return split_edge_at_point(topo, ce.edge, p, edges);
                }
                if let Some((_, py)) = prev
                    && py * y < 0.0
                {
                    // Bisect on the fraction.
                    let (mut a, mut b) = (prev.map_or(0.0, |x| x.0), f);
                    let (mut ya, _) = (py, y);
                    for _ in 0..100 {
                        let m = 0.5 * (a + b);
                        let fm = if ce.sense == Sense::Forward { m } else { 1.0 - m };
                        let pm = geo::edge_sample(topo, ce.edge, fm).ok_or(BooleanError::Unsupported("a cut edge has no geometry"))?;
                        let ym = ly(pm);
                        if ym * ya > 0.0 {
                            a = m;
                            ya = ym;
                        } else {
                            b = m;
                        }
                    }
                    let m = 0.5 * (a + b);
                    let fm = if ce.sense == Sense::Forward { m } else { 1.0 - m };
                    let pm = geo::edge_sample(topo, ce.edge, fm).ok_or(BooleanError::Unsupported("a cut edge has no geometry"))?;
                    return split_edge_at_point(topo, ce.edge, pm, edges);
                }
            }
            prev = Some((f, y));
        }
    }
    Err(BooleanError::Unsupported("a cut loop around the cylinder has no seam crossing"))
}

/// Split `edge` at the point `p` of it (a new vertex); the cut-edge set is updated.
fn split_edge_at_point<E: TopologyEdit + ?Sized>(topo: &mut E, edge: EntityId, p: Point3, edges: &mut BTreeSet<EntityId>) -> Result<EntityId, BooleanError> {
    let Some((c, cc)) = geo::edge_curve(&*topo, edge) else { return Err(BooleanError::Unsupported("a cut edge has no curve")) };
    let (_, t) = crate::cc::dist_to_range(c, cc.range, p).ok_or(BooleanError::Unsupported("cannot place a vertex on a cut edge"))?;
    let v = topo.make_vertex(p)?;
    let [a, b] = topo.split_edge(edge, v, t)?;
    edges.remove(&edge);
    edges.insert(a);
    edges.insert(b);
    Ok(v)
}

fn rotate_to_start<E: TopologyEdit + ?Sized>(topo: &E, lp: &Loop, v: EntityId) -> Result<Loop, BooleanError> {
    let tail = |c: &CoedgeSpec| -> Option<EntityId> { topo.edge_ends(c.edge).map(|(s, t)| if c.sense == Sense::Forward { s } else { t }) };
    let i = lp.edges.iter().position(|c| tail(c) == Some(v)).ok_or(BooleanError::Unsupported("the seam vertex is not on the loop"))?;
    let mut e = lp.edges.clone();
    e.rotate_left(i);
    Ok(Loop { edges: e })
}

fn build_cylinder_caps<E: TopologyEdit + ?Sized>(topo: &mut E, edges: &mut BTreeSet<EntityId>, cyl: &Cylinder, tool: EntityId) -> Result<Vec<EntityId>, BooleanError> {
    let frame = cyl.frame;
    let loops = extract_loops(&*topo, edges)?;
    if loops.is_empty() {
        return Err(BooleanError::Unsupported("the tool produced no cut curves"));
    }
    // Split every essential loop at the seam, then work with the updated loops.
    let mut essential_idx: Vec<usize> = Vec::new();
    for (i, lp) in loops.iter().enumerate() {
        let tr = cyl_trace(&*topo, lp, &frame);
        if winding(&tr).abs() > 0.5 {
            essential_idx.push(i);
        }
    }
    let mut seam_of: BTreeMap<usize, EntityId> = BTreeMap::new();
    for &i in &essential_idx {
        let v = seam_vertex(topo, &loops[i], &frame, edges)?;
        seam_of.insert(i, v);
    }
    let loops = if essential_idx.is_empty() { loops } else { extract_loops(&*topo, edges)? };
    // Re-identify the essential loops after the splits (same order of discovery is not
    // guaranteed): they are the loops that contain a seam vertex.
    let seam_vs: BTreeSet<EntityId> = seam_of.values().copied().collect();
    let mut caps = Vec::new();
    let mut bands: Vec<(f64, usize, EntityId)> = Vec::new();
    let mut contractible: Vec<usize> = Vec::new();
    for (i, lp) in loops.iter().enumerate() {
        let tr = cyl_trace(&*topo, lp, &frame);
        if winding(&tr).abs() > 0.5 {
            let verts: BTreeSet<EntityId> = lp.edges.iter().flat_map(|c| topo.edge_ends(c.edge).map(|(s, t)| [s, t]).into_iter().flatten()).collect();
            let sv = verts.iter().copied().find(|v| seam_vs.contains(v)).ok_or(BooleanError::Unsupported("lost the seam vertex of a loop"))?;
            let mean_v = tr.iter().map(|x| x.1).sum::<f64>() / tr.len() as f64;
            bands.push((mean_v, i, sv));
        } else {
            contractible.push(i);
        }
    }
    bands.sort_by(|a, b| a.0.total_cmp(&b.0));
    if !bands.len().is_multiple_of(2) {
        return Err(BooleanError::Unsupported("the tool cuts the volume in an odd number of loops around the cylinder: it does not separate it"));
    }
    for pair in bands.chunks(2) {
        let (lo, hi) = (&pair[0], &pair[1]);
        let mut a = rotate_to_start(&*topo, &loops[lo.1], lo.2)?;
        let mut b = rotate_to_start(&*topo, &loops[hi.1], hi.2)?;
        // `a` must run along +u, `b` along -u (counter-clockwise seen from the outside).
        let dir = |topo: &E, lp: &Loop| -> f64 { winding(&cyl_trace(topo, lp, &frame)) };
        if dir(&*topo, &a) < 0.0 {
            a = rotate_to_start(&*topo, &reversed(&a), lo.2)?;
        }
        if dir(&*topo, &b) > 0.0 {
            b = rotate_to_start(&*topo, &reversed(&b), hi.2)?;
        }
        // The seam edge between the two seam vertices, along the generator at u = 0.
        let (p_lo, p_hi) = (geo::vertex_point(&*topo, lo.2).ok_or(BooleanError::Unsupported("vertex without point"))?, geo::vertex_point(&*topo, hi.2).ok_or(BooleanError::Unsupported("vertex without point"))?);
        let origin = frame.origin + frame.x * cyl.radius;
        let line = Line::new(origin, frame.z)?;
        let handle = topo.shapes_mut()?.add_curve(sm_geom_api::CurveDef::Line(line))?;
        let (t_lo, t_hi) = ((p_lo - origin).dot(frame.z), (p_hi - origin).dot(frame.z));
        let seam = topo.make_edge(lo.2, hi.2, Some(CurveCarrier { curve: handle, range: Interval::new(t_lo, t_hi), sense: Sense::Forward }))?;
        let mut ring: Vec<CoedgeSpec> = a.edges.clone();
        ring.push(CoedgeSpec { edge: seam, sense: Sense::Forward });
        ring.extend(b.edges.iter().copied());
        ring.push(CoedgeSpec { edge: seam, sense: Sense::Reversed });
        let cap = topo.make_face(&[&ring], Some(SurfaceCarrier { surface: tool, sense: Sense::Forward }))?;
        caps.push(cap);
    }
    if !contractible.is_empty() {
        return Err(BooleanError::Unsupported("the cylinder tool cuts a patch that does not wrap around it (not available yet)"));
    }
    Ok(caps)
}

/// Cubit `nomerge` for one cap: the volume on the cap's right gets its own copy of the face, of
/// the edges around it and of the vertices where only its own edges meet, so the two pieces
/// share no cell. Returns the new cap.
fn unmerge_cap<E: TopologyEdit + ?Sized>(topo: &mut E, cap: EntityId, right_volume: EntityId) -> Result<EntityId, BooleanError> {
    let left_volume = match topo.face_sides(cap).map(|s| s.left) {
        Some(sm_topo_api::Region::Volume(v)) => Some(v),
        _ => None,
    };
    let cap_edges: Vec<EntityId> = topo.face_edges(cap).iter().copied().filter(|&e| !geo::is_degenerate(&*topo, e)).collect();
    let new = topo.unmerge_face(cap, Side::Right)?;
    let pos_faces: BTreeSet<EntityId> = topo.volume_faces(right_volume).iter().copied().collect();
    for &e in &cap_edges {
        let users: Vec<EntityId> = topo.coboundary(e).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Face) && pos_faces.contains(&i.cell)).map(|i| i.cell).collect();
        if !users.is_empty() {
            topo.unmerge_edge(e, &users)?;
        }
    }
    let edges_of = |t: &E, faces: &mut dyn Iterator<Item = EntityId>| -> BTreeSet<EntityId> { faces.flat_map(|f| t.face_edges(f).to_vec()).collect() };
    let pos_edges = edges_of(&*topo, &mut pos_faces.iter().copied());
    let mut neg_faces: Vec<EntityId> = left_volume.map(|v| topo.volume_faces(v).to_vec()).unwrap_or_default();
    neg_faces.push(cap);
    let neg_edges = edges_of(&*topo, &mut neg_faces.into_iter());
    let mut verts: BTreeSet<EntityId> = BTreeSet::new();
    for &e in &cap_edges {
        if let Some((s, t)) = topo.edge_ends(e) {
            verts.insert(s);
            verts.insert(t);
        }
    }
    for v in verts {
        let mine: Vec<EntityId> = topo.coboundary(v).iter().filter(|i| topo.dim_of(i.cell) == Some(Dim::Edge)).map(|i| i.cell).filter(|e| pos_edges.contains(e) && !neg_edges.contains(e)).collect();
        if !mine.is_empty() {
            topo.unmerge_vertex(v, &mine)?;
        }
    }
    Ok(new)
}
