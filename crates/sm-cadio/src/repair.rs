//! Tolerant repair of open shells (`repair: "tolerant"`).
//!
//! A shell is open when some edge's coedges do not cancel. The repair handles the common case
//! of a deleted or missing face: every open edge is used exactly once net, the open edges chain
//! into simple loops, each loop is a polygon of straight edges and lies in one plane. Each loop
//! gets a planar cap face. Anything else (an edge used two or more times net, a branching
//! boundary, a curved edge, a non-planar loop, coplanar loops that would need a hole) is not
//! guessed: the shell stays open and the caller gets the same [`StepError::OpenShell`] as
//! without repair. The result is all or nothing per solid.

use std::collections::{BTreeMap, BTreeSet};

use sm_core::Vector3;
use sm_geom_api::{CurveDef, Frame, Plane, SurfaceDef};
use sm_topo_api::Sense;

use crate::error::{OpenEdge, StepError};
use crate::model::{Coedge, Face, Note, NoteCode, Solid, SurfaceRec};

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
    }
    let base = solid.faces.len();
    for (k, f) in new_faces.iter().enumerate() {
        solid.shells[f.shell].faces.push(base + k);
    }
    solid.faces.extend(new_faces);
    solid.surfaces.extend(new_surfaces);
    Ok(notes)
}
