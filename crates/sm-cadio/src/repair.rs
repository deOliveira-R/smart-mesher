//! Tolerant repair of open shells (`repair: "tolerant"`).
//!
//! A shell is open when some edge's coedges do not cancel. The repair handles the common case
//! of a deleted or missing face: every open edge is used exactly once net, the open edges chain
//! into simple loops, each loop is a polygon of straight edges and lies in one plane. Each loop
//! gets a planar cap face. Anything else (an edge used two or more times net, a branching
//! boundary, a curved edge, a non-planar loop, coplanar loops that would need a hole) is not
//! guessed: the shell stays open and the caller gets the same [`StepError::OpenShell`] as
//! without repair. A cap that lands on an existing face, or a closed shell of about zero volume,
//! is refused too (a displaced face is not a missing one). The result is all or nothing per solid.
//!
//! Sheets are repaired one at a time, so a face of the same part that sits in another sheet is
//! not seen by the cap check. [`note_displaced_faces`] looks for it afterwards and rewrites the
//! `step.repaired` note to say that the cap replaces a displaced face (it does not sew it).

use std::collections::{BTreeMap, BTreeSet};

use sm_core::Vector3;
use sm_geom_api::{CurveDef, Frame, Plane, SurfaceDef};
use sm_topo_api::Sense;

use crate::error::{OpenEdge, StepError};
use crate::model::{Coedge, Face, Note, NoteCode, Solid, SolidKind, SurfaceRec};

/// Close every open shell of `solid` with planar caps; `tol` is the planarity tolerance.
///
/// # Errors
/// [`StepError::OpenShell`] listing the open edges of the first shell that cannot be closed;
/// `solid` is then unchanged.
pub(crate) fn close_shells(solid: &mut Solid, tol: f64) -> Result<Vec<Note>, StepError> {
    let mut notes = Vec::new();
    let mut new_faces: Vec<Face> = Vec::new();
    let mut new_surfaces: Vec<SurfaceRec> = Vec::new();
    for si in 0..solid.shells.len() {
        let first_new = new_faces.len();
        let mut caps: Vec<(Vector3, sm_core::Point3, Vec<usize>)> = Vec::new();
        let mut net: BTreeMap<usize, i64> = BTreeMap::new();
        for &fi in &solid.shells[si].faces {
            for lp in &solid.faces[fi].loops {
                for c in lp {
                    *net.entry(c.edge).or_default() += if c.sense == Sense::Forward { 1 } else { -1 };
                }
            }
        }
        let open: Vec<(usize, i64)> = net.into_iter().filter(|(_, n)| *n != 0).collect();
        if open.is_empty() {
            continue;
        }
        let refuse = || {
            let mut issues: Vec<OpenEdge> = open.iter().map(|(e, n)| OpenEdge { edge: solid.edges[*e].id, net_uses: *n }).collect();
            issues.sort_by_key(|i| i.edge);
            StepError::OpenShell { solid: solid.id, issues }
        };
        // Directed boundary: the cap uses each open edge against the shell's net use.
        let mut out: BTreeMap<usize, (usize, Sense, usize)> = BTreeMap::new();
        let mut targets: BTreeSet<usize> = BTreeSet::new();
        for &(ei, n) in &open {
            let e = &solid.edges[ei];
            if n.abs() != 1 || e.start == e.end || !matches!(solid.curves[e.curve].def, CurveDef::Line(_)) {
                return Err(refuse());
            }
            let (sense, from, to) = if n > 0 { (Sense::Reversed, e.end, e.start) } else { (Sense::Forward, e.start, e.end) };
            if out.insert(from, (ei, sense, to)).is_some() || !targets.insert(to) {
                return Err(refuse());
            }
        }
        if out.keys().ne(targets.iter()) {
            return Err(refuse());
        }
        let mut seen: BTreeSet<usize> = BTreeSet::new();
        let mut planes: Vec<(Vector3, sm_core::Point3)> = Vec::new();
        for &start in out.keys() {
            if seen.contains(&start) {
                continue;
            }
            let (mut lp, mut pts, mut v) = (Vec::new(), Vec::new(), start);
            loop {
                seen.insert(v);
                let (ei, sense, to) = out[&v];
                lp.push(Coedge { edge: ei, sense });
                pts.push(solid.vertices[v].point);
                v = to;
                if v == start {
                    break;
                }
            }
            // Newell normal: counter-clockwise seen from outside, as for faceted faces.
            let mut n = [0.0f64; 3];
            for i in 0..pts.len() {
                let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
                n[0] += (p.y - q.y) * (p.z + q.z);
                n[1] += (p.z - q.z) * (p.x + q.x);
                n[2] += (p.x - q.x) * (p.y + q.y);
            }
            let nv = Vector3::new(n[0], n[1], n[2]);
            let norm = nv.norm();
            let extent = pts.iter().map(|p| (*p - pts[0]).norm()).fold(0.0f64, f64::max);
            if lp.len() < 3 || norm.is_nan() || norm <= 1e-12 * extent * extent {
                return Err(refuse());
            }
            let nh = nv * (1.0 / norm);
            let dev = pts.iter().map(|p| (*p - pts[0]).dot(nh).abs()).fold(0.0f64, f64::max);
            let flat = tol.max(1e-9 * extent);
            if dev > flat {
                return Err(refuse());
            }
            // Two coplanar loops would be a cap with a hole: not guessed.
            if planes.iter().any(|(m, o)| m.dot(nh).abs() > 1.0 - 1e-9 && (pts[0] - *o).dot(nh).abs() <= flat) {
                return Err(refuse());
            }
            planes.push((nh, pts[0]));
            // A cap that lands on an existing face means that face is displaced, not missing.
            let cap_verts: BTreeSet<usize> = lp.iter().map(|c| coedge_start(solid, c)).collect();
            for &fi in &solid.shells[si].faces {
                let f = &solid.faces[fi];
                let SurfaceDef::Plane(pl) = &solid.surfaces[f.surface].def else { continue };
                let coplanar = pl.frame.z.dot(nh).abs() > 1.0 - 1e-9 && (pl.frame.origin - pts[0]).dot(nh).abs() <= flat;
                if coplanar && f.loops.first().is_some_and(|l| l.iter().all(|c| cap_verts.contains(&coedge_start(solid, c)))) {
                    return Err(refuse());
                }
            }
            caps.push((nh, pts[0], cap_verts.into_iter().collect()));
            let frame = Frame::from_z(pts[0], nv).map_err(|_| refuse())?;
            let fid = solid.edges[lp[0].edge].id;
            let ids: Vec<u64> = lp.iter().map(|c| solid.edges[c.edge].id).collect();
            notes.push(Note::warn(
                NoteCode::Repaired,
                format!("closed an open boundary of {} edge(s) with a planar face (max deviation {dev:e}; a face was missing)", lp.len()),
                ids,
            ));
            new_surfaces.push(SurfaceRec { id: fid, def: SurfaceDef::Plane(Plane { frame }) });
            new_faces.push(Face { id: fid, surface: solid.surfaces.len() + new_surfaces.len() - 1, sense: Sense::Forward, loops: vec![lp], shell: si });
        }
        if caps.is_empty() {
            continue;
        }
        // The closed shell must enclose volume and not lie flat in a cap plane.
        let flat_all = solid.shells[si].faces.iter().all(|&fi| match &solid.surfaces[solid.faces[fi].surface].def {
            SurfaceDef::Plane(pl) => caps.iter().any(|(nh, o, _)| pl.frame.z.dot(*nh).abs() > 1.0 - 1e-9 && (pl.frame.origin - *o).dot(*nh).abs() <= tol.max(1e-9)),
            _ => false,
        });
        if flat_all {
            return Err(refuse());
        }
        let all: Vec<&Face> = solid.shells[si].faces.iter().map(|&fi| &solid.faces[fi]).chain(new_faces[first_new..].iter()).collect();
        if let Some((v, extent)) = polyhedron_volume(solid, &all, &new_surfaces)
            && v.abs() <= tol.max(1e-9 * extent) * extent * extent
        {
            return Err(refuse());
        }
    }
    let base = solid.faces.len();
    for (k, f) in new_faces.iter().enumerate() {
        solid.shells[f.shell].faces.push(base + k);
    }
    solid.faces.extend(new_faces);
    solid.surfaces.extend(new_surfaces);
    Ok(notes)
}

fn coedge_start(solid: &Solid, c: &Coedge) -> usize {
    let e = &solid.edges[c.edge];
    if c.sense == Sense::Forward { e.start } else { e.end }
}

/// Signed volume and bounding-box diagonal of a shell whose faces are all planar polygons with
/// straight edges; `None` when any face or edge is curved.
fn polyhedron_volume(solid: &Solid, faces: &[&Face], extra: &[SurfaceRec]) -> Option<(f64, f64)> {
    let (mut v6, mut lo, mut hi) = (0.0, [f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for f in faces {
        let surf = solid.surfaces.get(f.surface).unwrap_or_else(|| &extra[f.surface - solid.surfaces.len()]);
        if !matches!(surf.def, SurfaceDef::Plane(_)) {
            return None;
        }
        for lp in &f.loops {
            if lp.iter().any(|c| !matches!(solid.curves[solid.edges[c.edge].curve].def, CurveDef::Line(_))) {
                return None;
            }
            let pts: Vec<sm_core::Point3> = lp.iter().map(|c| solid.vertices[coedge_start(solid, c)].point).collect();
            for p in &pts {
                for (k, x) in [p.x, p.y, p.z].into_iter().enumerate() {
                    lo[k] = lo[k].min(x);
                    hi[k] = hi[k].max(x);
                }
            }
            for i in 1..pts.len().saturating_sub(1) {
                let (a, b, c) = (pts[0], pts[i], pts[i + 1]);
                v6 += a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
            }
        }
    }
    let extent = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    Some((v6 / 6.0, extent))
}

/// Rewrite the `step.repaired` note of every cap that has a near-coplanar face on the same
/// outline in another (not imported) sheet: that face is displaced, not missing. The cap stays,
/// the note names the face and its offset. Caps are found by their face id (the note's first id).
pub(crate) fn note_displaced_faces(solids: &[Solid], notes: &mut [Note]) {
    for n in notes.iter_mut().filter(|n| n.code == NoteCode::Repaired && n.message.starts_with("closed an open boundary")) {
        let Some(&cap_id) = n.step_ids.first() else { continue };
        let Some((s, cap)) = solids.iter().filter(|s| s.kind != SolidKind::Sheet).find_map(|s| s.faces.iter().find(|f| f.id == cap_id).map(|f| (s, f))) else { continue };
        let Some((nh, o, pts)) = plane_outline(s, cap) else { continue };
        let extent = pts.iter().map(|p| (*p - pts[0]).norm()).fold(0.0f64, f64::max);
        let near = 1e-3 * extent;
        let found = solids.iter().filter(|b| b.kind == SolidKind::Sheet).flat_map(|b| b.faces.iter().map(move |f| (b, f))).find_map(|(b, f)| {
            let (m, p0, q) = plane_outline(b, f)?;
            let off = (p0 - o).dot(nh);
            let same = q.len() == pts.len() && m.dot(nh).abs() > 1.0 - 1e-9 && off.abs() <= near;
            let outline = same && q.iter().all(|p| pts.iter().any(|c| (*p - *c - nh * off).norm() <= near));
            outline.then_some((f.id, off))
        });
        if let Some((id, off)) = found {
            n.message = n.message.replace("a face was missing", &format!("the file holds a face #{id} on this outline at offset {off:.3e} in another sheet: it is replaced by the cap, not sewn"));
        }
    }
}

/// Normal, a point and the outer-loop vertices of a planar, straight-edged face.
fn plane_outline(solid: &Solid, f: &Face) -> Option<(Vector3, sm_core::Point3, Vec<sm_core::Point3>)> {
    let SurfaceDef::Plane(pl) = &solid.surfaces.get(f.surface)?.def else { return None };
    let lp = f.loops.first()?;
    if lp.iter().any(|c| !matches!(solid.curves[solid.edges[c.edge].curve].def, CurveDef::Line(_))) {
        return None;
    }
    let pts = lp.iter().map(|c| solid.vertices[coedge_start(solid, c)].point).collect();
    Some((pl.frame.z, pl.frame.origin, pts))
}
