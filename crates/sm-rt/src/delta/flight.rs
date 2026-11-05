//! `MajorantTracker` for the mesh tracker: surface and delta segments over the partition.

use super::{F_PRE, JumpIndex, NoHierarchy, Partition};
use crate::core::{C_CONVEX, NONE};
use crate::tracker::{MeshTracker, Probe};
use crate::vec3::{dot, finite3, madd};
use sm_core::{Point3, Vector3};
use sm_rt_api::{
    CellId, FaceId, FlightSegment, HierarchyView, Level, Located, LostCause, MajorantTracker, NodeId, RegionId, Result, RtError, Scratch, SegmentQuery, SegmentScale, StepEnd,
    SurfaceId, Tracker, TrackMode, TrackState, choose_mode,
};

static NO_HIERARCHY: NoHierarchy = NoHierarchy;

pub(crate) fn p3(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// Distance along unit `d` from `o` to the wall of `bx` it leaves through, with the axis and
/// whether it is the high wall.
pub(crate) fn box_exit(o: [f64; 3], d: [f64; 3], bx: &[f64; 6]) -> (f64, usize, bool) {
    let mut best = (f64::INFINITY, 0usize, false);
    for k in 0..3 {
        if d[k] > 0.0 {
            let t = (bx[3 + k] - o[k]) / d[k];
            if t < best.0 {
                best = (t, k, true);
            }
        } else if d[k] < 0.0 {
            let t = (bx[k] - o[k]) / d[k];
            if t < best.0 {
                best = (t, k, false);
            }
        }
    }
    (best.0.max(0.0), best.1, best.2)
}

pub(crate) fn end_face(e: &StepEnd) -> Option<FaceId> {
    match e {
        StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } | StepEnd::Boundary { face, .. } => Some(*face),
        _ => None,
    }
}

impl MeshTracker {
    /// The partition, when the tracker was built with one.
    pub(crate) fn partition(&self) -> Option<&Partition> {
        self.part.as_ref()
    }

    /// The region-jump index, when built.
    pub(crate) fn jump_index(&self) -> Option<&JumpIndex> {
        self.part.as_ref().and_then(|p| p.jump.as_ref())
    }

    /// `locate` over the candidate list of one kd leaf: the same cells, order and probes as the
    /// BVH path of `locate_inner` (bounding boxes padded by `tol` filter the candidates), so the
    /// answer is identical; no BVH descent.
    pub(crate) fn locate_leaf(&self, part: &Partition, kd_leaf: usize, p: [f64; 3], dir: Option<[f64; 3]>, sc: &mut Scratch) -> Located {
        let tol = self.tol();
        if !self.in_bounds(p, tol) {
            return Located::new(CellId::NONE, RegionId::NONE, None);
        }
        let cands = part.leaf_cands(kd_leaf);
        if part.kd[kd_leaf].flags & F_PRE != 0
            && let Some(&c) = cands.first()
        {
            return Located::new(CellId(c), RegionId(self.core.c_region[c as usize]), None);
        }
        let mut dir = dir;
        let mut best: [Option<(u32, u32)>; 3] = [None; 3];
        let mut lowest_face = NONE;
        for &c in cands {
            let b = &self.core.c_bbox[c as usize];
            if (0..3).any(|k| p[k] < b[k] - tol || p[k] > b[3 + k] + tol) {
                continue;
            }
            sc.counters.locate_candidates += 1;
            match self.probe(c as usize, p, &mut dir) {
                Probe::In => return Located::new(CellId(c), RegionId(self.core.c_region[c as usize]), None),
                Probe::Out { .. } => {}
                Probe::On { face, rank } => {
                    lowest_face = lowest_face.min(face);
                    if best[rank as usize].is_none() {
                        best[rank as usize] = Some((c, face));
                    }
                }
            }
        }
        match best[0].or(best[1]) {
            Some((c, f)) => Located::new(CellId(c), RegionId(self.core.c_region[c as usize]), Some(FaceId(f))),
            None => {
                let mut l = self.outside(p);
                l.on_face = (lowest_face != NONE).then_some(FaceId(lowest_face));
                l
            }
        }
    }

    /// Locate from the root of the partition (a kd descent, never the cell BVH).
    pub(crate) fn locate_from_root(&self, part: &Partition, p: [f64; 3], d: [f64; 3], sc: &mut Scratch) -> Located {
        let mut visits = 0;
        let (leaf, _, _) = part.descend(0, part.root_box, p, d, &mut visits);
        sc.counters.node_visits += visits;
        self.locate_leaf(part, leaf, p, Some(d), sc)
    }
}

impl MajorantTracker for MeshTracker {
    fn hierarchy(&self) -> &dyn HierarchyView {
        match &self.part {
            Some(p) => p,
            None => &NO_HIERARCHY,
        }
    }

    fn advance_segment(&self, st: &mut TrackState, q: &SegmentQuery<'_>, sc: &mut Scratch) -> Result<FlightSegment> {
        let Some(part) = self.partition() else { return Err(RtError::Unsupported("tracker was built without a partition")) };
        let (o, d) = ([st.pos.x, st.pos.y, st.pos.z], [st.dir.x, st.dir.y, st.dir.z]);
        if !finite3(o) || !finite3(d) {
            return Err(RtError::NonFinite("state"));
        }
        let (first, region, face_in) = (st.cell, st.region, st.entered_via);
        // The node the segment starts in: a kd descent that prefers the child the ray enters.
        let mut visits = 0;
        let (_, node, node_box) = part.descend(0, part.root_box, o, d, &mut visits);
        sc.counters.node_visits += visits;
        let rec = &part.nodes[node as usize];
        let node_id = NodeId(node);
        // The one majorant entry this call reads, before the state moves.
        let sigma_m = q.input.majorant(node_id)?;
        let mut mode = choose_mode(q.policy, node_id, &q.input);
        // Without a located mesh cell (complement, outside) there is nothing to skip over.
        if first.is_none() || first == CellId::COMPLEMENT || first.get() >= self.core.n_cells() {
            mode = TrackMode::Surface;
        }
        let set = sm_rt_api::MaterialSetId(rec.set);
        let saved = st.clone();
        match mode {
            TrackMode::Surface if q.scale == SegmentScale::Cell => {
                let s = self.step(st, q.max_dist, sc)?;
                Ok(FlightSegment::new(mode, node_id, set, s.cell, s.region, s.dist, sigma_m).with_faces(face_in, end_face(&s.end)).with_resolved(true).with_end(s.end))
            }
            TrackMode::Surface => {
                // One segment per run inside a region (jumps when the tracker can).
                let r = self.fire(st, q.max_dist, self.info.max_events, sc)?;
                let (mut end, mut total) = (r.end, r.dist);
                if let StepEnd::Lost(_) = end {
                    st.clone_from(&saved);
                    total = 0.0;
                    end = r.end;
                }
                Ok(FlightSegment::new(mode, node_id, set, first, region, total, sigma_m).with_faces(face_in, end_face(&end)).with_resolved(false).with_end(end))
            }
            TrackMode::Delta => {
                let (t_wall, ax, hi) = box_exit(o, d, &node_box);
                let want_wall = t_wall < q.max_dist;
                let limit = if want_wall { t_wall } else { q.max_dist };
                let mut total = 0.0;
                let mut end;
                if rec.boundary {
                    // A model boundary or the complement may be inside: walk the cells (events
                    // are those of `step`), but only up to the wall.
                    let mut iters = 0u32;
                    loop {
                        let s = self.step(st, (limit - total).max(0.0), sc)?;
                                total += s.dist;
                        end = s.end;
                        let more = match end {
                            StepEnd::CellFace { .. } => true,
                            StepEnd::RegionBoundary { to_cell, .. } => !to_cell.is_none(),
                            _ => false,
                        };
                        if !more {
                            break;
                        }
                        iters += 1;
                        if iters > self.info.max_events {
                            end = StepEnd::Lost(sm_rt_api::lost(st, LostCause::MaxStepsExceeded));
                            break;
                        }
                    }
                } else {
                    // No cell face of the model lies inside: go straight to the wall or the limit.
                    total = limit;
                    st.pos = p3(madd(o, d, limit));
                    end = StepEnd::Limit;
                }
                if let StepEnd::Lost(_) = end {
                    st.clone_from(&saved);
                    total = 0.0;
                } else if matches!(end, StepEnd::Limit) && (want_wall || !rec.boundary) {
                    let mut pp = [st.pos.x, st.pos.y, st.pos.z];
                    if want_wall {
                        pp[ax] = if hi { node_box[3 + ax] } else { node_box[ax] };
                        st.pos = p3(pp);
                    }
                    let loc = self.locate_from_root(part, pp, d, sc);
                    st.cell = loc.cell;
                    st.region = loc.region;
                    st.entered_via = loc.on_face;
                    end = if loc.cell.is_none() {
                        StepEnd::RegionBoundary { surface: SurfaceId::NONE, face: FaceId::NONE, to_region: RegionId::NONE, to_cell: CellId::NONE }
                    } else if want_wall {
                        StepEnd::SlotChange { to_cell: loc.cell }
                    } else {
                        StepEnd::Limit
                    };
                }
                Ok(FlightSegment::new(mode, node_id, set, first, region, total, sigma_m).with_faces(face_in, end_face(&end)).with_resolved(false).with_end(end))
            }
        }
    }

    fn locate_in_node(&self, node: NodeId, _levels: &[Level], p: Point3, dir: Option<Vector3>, sc: &mut Scratch) -> Result<Located> {
        let Some(part) = self.partition() else { return Err(RtError::Unsupported("tracker was built without a partition")) };
        let Some(rec) = part.nodes.get(node.get()) else { return Err(RtError::UnknownId { kind: "node", id: node.0 }) };
        let pa = [p.x, p.y, p.z];
        if !finite3(pa) {
            return Err(RtError::NonFinite("position"));
        }
        let da = match dir {
            Some(v) => {
                let a = [v.x, v.y, v.z];
                if !finite3(a) {
                    return Err(RtError::NonFinite("direction"));
                }
                a
            }
            None => [0.0; 3],
        };
        let tol = self.tol();
        let inside = (0..3).all(|k| pa[k] >= rec.bounds[k] - tol && pa[k] <= rec.bounds[3 + k] + tol);
        // From the node that owns the majorant; a region-run segment may reach beyond its first
        // node, and then the descent restarts at the root of the kd tree (still no BVH).
        let (start, bx) = if inside { (rec.kd as usize, rec.bounds) } else { (0, part.root_box) };
        let mut visits = 0;
        let (leaf, _, _) = part.descend(start, bx, pa, da, &mut visits);
        sc.counters.node_visits += visits;
        Ok(self.locate_leaf(part, leaf, pa, dir.map(|_| da), sc))
    }

    fn safety_distance(&self, st: &TrackState, _sc: &mut Scratch) -> Result<f64> {
        if st.cell.is_none() || st.cell == CellId::COMPLEMENT || st.cell.get() >= self.core.n_cells() {
            return Ok(0.0);
        }
        let c = st.cell.get();
        if self.core.c_flags[c] & C_CONVEX == 0 {
            return Ok(0.0);
        }
        let p = [st.pos.x, st.pos.y, st.pos.z];
        let mut m = f64::INFINITY;
        for &f in self.core.cell_faces(c) {
            let pl = self.core.f_plane[f as usize];
            let s = self.core.sign(c, f as usize);
            m = m.min(s * (pl[3] - dot([pl[0], pl[1], pl[2]], p)));
        }
        Ok(if m.is_finite() { m.max(0.0) } else { 0.0 })
    }
}
