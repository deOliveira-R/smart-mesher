//! Majorant hierarchy of the analytic stub: one box leaf per lattice element (a flat world is a
//! single leaf), a `Lattice` root with arithmetic children and a real DDA over the elements.
//! Surface mode is the exact cell chords of the stub's own shapes. Brute force inside a leaf.

use super::{AnalyticWorld, comp, compv, set_comp, shift};
use crate::cache::{ContentHash, ContentHasher};
use crate::error::{BuildIssue, LostCause, Result, RtError};
use crate::ids::{CellId, FaceId, MaterialSetId, NodeId, RegionId, SurfaceId};
use crate::majorant::{
    CursorLevel, HierarchyOptions, MajorantHierarchy, MajorantSegment, MajorantTable, MarchCursor, MarchEnd, MarchOptions, MaterialMap,
    MaterialSets, NodeInfo, NodeKind, NodeTable,
};
use crate::state::{Located, Scratch, StepEnd, TrackState};
use crate::tracker::{Tracker, check_direction, lost};
use sm_core::{Aabb, Point3, Vector3};

/// The stub's hierarchy; build it with `AnalyticWorld::majorant_hierarchy`.
#[derive(Clone, Debug)]
pub struct AnalyticHierarchy {
    w: AnalyticWorld,
    nodes: NodeTable,
    sets: MaterialSets,
    /// Singleton set per local cell (region).
    singles: Vec<MaterialSetId>,
    hash: ContentHash,
}

impl AnalyticHierarchy {
    pub(super) fn build(w: &AnalyticWorld, materials: &MaterialMap, _opts: &HierarchyOptions) -> Result<Self> {
        let nl = w.n_local();
        let mats: Vec<u32> = (0..nl).filter_map(|c| materials.material_of(RegionId::new(c))).collect();
        if mats.len() != nl {
            return Err(RtError::Build(vec![BuildIssue::new(
                "majorant.material_map",
                format!("the material map must cover all {nl} regions"),
            )]));
        }
        let mut sets = MaterialSets::new();
        let all = sets.intern(&mats);
        let singles: Vec<MaterialSetId> = mats.iter().map(|&m| sets.intern(&[m])).collect();
        let dims = w.dims();
        let ne = dims[0] * dims[1] * dims[2];
        let mut nodes = Vec::with_capacity(ne + 1);
        let leaf_bounds = |idx: [i32; 3]| Aabb {
            lo: Point3::new(w.wall(idx, 0, false), w.wall(idx, 1, false), w.wall(idx, 2, false)),
            hi: Point3::new(w.wall(idx, 0, true), w.wall(idx, 1, true), w.wall(idx, 2, true)),
        };
        if let Some(l) = w.lattice {
            nodes.push(
                NodeInfo::new(w.domain, all, (ne * nl) as u32)
                    .with_kind(NodeKind::Lattice { origin: l.origin, pitch: l.pitch, dims: l.dims })
                    .with_children(NodeId(1), ne as u32),
            );
            for e in 0..ne {
                nodes.push(NodeInfo::new(leaf_bounds(w.idx_of(e)), all, nl as u32).with_parent(NodeId(0)));
            }
        } else {
            nodes.push(NodeInfo::new(w.domain, all, nl as u32));
        }
        let mut h = ContentHasher::new();
        for v in [w.info.structure_hash.0, materials.content_hash().0, sets.content_hash().0] {
            h.write_u64((v >> 64) as u64);
            h.write_u64(v as u64);
        }
        let out = Self { w: w.clone(), nodes: NodeTable(nodes), sets, singles, hash: h.finish() };
        let issues = out.nodes.validate(&out.sets);
        if issues.is_empty() { Ok(out) } else { Err(RtError::Build(issues)) }
    }

    fn leaf_id(&self, idx: [i32; 3]) -> NodeId {
        if self.w.lattice.is_some() { NodeId::new(1 + self.w.lin_of(idx)) } else { NodeId(0) }
    }

    /// First shape-boundary parameter strictly inside `(t, t_exit)`, else `t_exit`.
    fn next_break(&self, o: Point3, d: Vector3, t: f64, t_exit: f64) -> f64 {
        let mut best = t_exit;
        for s in &self.w.shapes {
            if let Some((a, b)) = s.interval(o, d) {
                for x in [a, b] {
                    if x > t && x < best {
                        best = x;
                    }
                }
            }
        }
        best
    }

    fn start_level(&self, st: &TrackState, idx: [i32; 3]) -> CursorLevel {
        let mut lv = CursorLevel::new(self.leaf_id(idx));
        lv.idx = idx;
        for a in 0..3 {
            let v = compv(st.dir, a);
            if v != 0.0 {
                lv.step[a] = if v > 0.0 { 1 } else { -1 };
                lv.t_next[a] = ((self.w.wall(idx, a, v > 0.0) - comp(st.pos, a)) / v).max(0.0);
            }
        }
        lv
    }
}

impl MajorantHierarchy for AnalyticHierarchy {
    fn nodes(&self) -> &NodeTable {
        &self.nodes
    }
    fn material_sets(&self) -> &MaterialSets {
        &self.sets
    }
    fn structure_hash(&self) -> ContentHash {
        self.hash
    }
    fn node_of_cell(&self, cell: CellId) -> Option<NodeId> {
        (cell.get() < self.w.n_cells()).then(|| self.leaf_id(self.w.idx_of(cell.get() / self.w.n_local())))
    }

    fn locate_in(&self, node: NodeId, p: Point3, dir: Option<Vector3>, scratch: &mut Scratch) -> Result<Located> {
        let n = self.nodes.get(node).ok_or(RtError::UnknownId { kind: "node", id: node.0 })?;
        let none = Located::new(CellId::NONE, RegionId::NONE, None);
        // A point within the linear tolerance of the extent counts as inside: clamp it, so a
        // collision sampled at the very end of a segment still locates in the segment's node.
        let tol = self.w.tol.linear;
        let near = |v: f64, lo: f64, hi: f64| v >= lo - tol && v <= hi + tol;
        if !(near(p.x, n.bounds.lo.x, n.bounds.hi.x) && near(p.y, n.bounds.lo.y, n.bounds.hi.y) && near(p.z, n.bounds.lo.z, n.bounds.hi.z))
        {
            return Ok(none);
        }
        let q = Point3::new(
            p.x.clamp(n.bounds.lo.x, n.bounds.hi.x),
            p.y.clamp(n.bounds.lo.y, n.bounds.hi.y),
            p.z.clamp(n.bounds.lo.z, n.bounds.hi.z),
        );
        let owned = |loc: &Located| matches!(self.node_of_cell(loc.cell), Some(o) if o == node || node == NodeId(0));
        let loc = self.w.locate(q, dir, None, scratch)?;
        if owned(&loc) {
            return Ok(loc);
        }
        // On a wall the direction picked the neighbour: the node's own side is the other one.
        if let Some(d) = dir {
            let back = self.w.locate(q, Some(Vector3::new(-d.x, -d.y, -d.z)), None, scratch)?;
            if owned(&back) {
                return Ok(back);
            }
        }
        Ok(none)
    }

    fn begin(&self, st: &TrackState, opts: &MarchOptions, table: &MajorantTable, scratch: &mut Scratch) -> Result<MarchCursor> {
        check_direction(st.dir, self.w.tol.angular)?;
        if opts.max_dist.is_nan() || opts.max_dist < 0.0 {
            return Err(RtError::InvalidInput("max_dist must be >= 0"));
        }
        if opts.band.get() >= table.bands().len() {
            return Err(RtError::UnknownId { kind: "band", id: opts.band.0 });
        }
        let mut cur = MarchCursor::new(st.pos, st.dir, *opts);
        let loc = self.w.locate(st.pos, Some(st.dir), None, scratch)?;
        match (loc.cell.is_none(), self.w.elem_at(st.pos, Some(st.dir))) {
            (false, Some(idx)) => {
                cur.node = self.leaf_id(idx);
                cur.levels.push(self.start_level(st, idx));
            }
            _ => cur.end = Some(MarchEnd::Lost(lost(st, LostCause::NoCellContainsPoint))),
        }
        Ok(cur)
    }

    fn next(&self, cur: &mut MarchCursor, table: &MajorantTable, _scratch: &mut Scratch) -> Result<Option<MajorantSegment>> {
        let dims = self.w.dims();
        loop {
            if cur.end.is_some() {
                return Ok(None);
            }
            if cur.t >= cur.opts.max_dist {
                cur.end = Some(MarchEnd::Limit);
                return Ok(None);
            }
            if cur.emitted >= cur.opts.max_segments {
                cur.end = Some(MarchEnd::MaxSegments);
                return Ok(None);
            }
            let lv = cur.levels[0];
            let tmin = lv.t_next.iter().copied().fold(f64::INFINITY, f64::min);
            if cur.t >= tmin {
                // The leaf is exhausted: step every axis whose wall is at `tmin` (DDA).
                let mut idx = lv.idx;
                let mut out: Option<usize> = None;
                for a in 0..3 {
                    if lv.t_next[a] == tmin {
                        idx[a] += lv.step[a];
                        if !(0..dims[a] as i32).contains(&idx[a]) && out.is_none() {
                            out = Some(2 * a + usize::from(lv.step[a] > 0));
                        }
                    }
                }
                if let Some(k) = out {
                    let surface = SurfaceId::new(self.w.n() + k);
                    cur.aux[0] = k as f64;
                    cur.end = Some(MarchEnd::Boundary { surface, face: FaceId(surface.0), action: self.w.actions[k] });
                    return Ok(None);
                }
                let mut next = lv;
                next.idx = idx;
                for a in 0..3 {
                    if lv.t_next[a] == tmin {
                        next.t_next[a] = (self.w.wall(idx, a, lv.step[a] > 0) - comp(cur.origin, a)) / compv(cur.dir, a);
                    }
                }
                next.node = self.leaf_id(idx);
                cur.levels[0] = next;
                continue;
            }
            let t_exit = tmin.min(cur.opts.max_dist);
            let leaf = lv.node;
            let set = self.nodes.0[leaf.get()].set;
            if cur.opts.refine.stop_at(table, leaf, set, cur.opts.band) {
                let seg = MajorantSegment::delta(leaf, set, cur.t, t_exit);
                cur.t = t_exit;
                cur.node = leaf;
                cur.emitted += 1;
                return Ok(Some(seg));
            }
            // Surface mode: one maximal cell chord inside this element.
            let e = self.w.lin_of(lv.idx);
            let nl = self.w.n_local();
            let o = shift(cur.origin, self.w.offset(lv.idx));
            let d = cur.dir;
            let mid = |a: f64, b: f64| {
                let m = 0.5 * (a + b);
                Point3::new(o.x + d.x * m, o.y + d.y * m, o.z + d.z * m)
            };
            let first = self.next_break(o, d, cur.t, t_exit);
            let c = self.w.classify(mid(cur.t, first));
            let mut t_end = first;
            while t_end < t_exit {
                let nb = self.next_break(o, d, t_end, t_exit);
                if self.w.classify(mid(t_end, nb)) != c {
                    break;
                }
                t_end = nb;
            }
            let seg = MajorantSegment::surface(leaf, self.singles[c], cur.t, t_end, CellId::new(e * nl + c), RegionId::new(c));
            cur.t = t_end;
            cur.node = leaf;
            cur.emitted += 1;
            return Ok(Some(seg));
        }
    }

    fn finish(&self, cur: &MarchCursor, scratch: &mut Scratch) -> Result<TrackState> {
        let mut p = cur.point_at(cur.t);
        let boundary = matches!(cur.end, Some(MarchEnd::Boundary { .. })).then(|| cur.aux[0] as usize);
        if let (Some(k), Some(lv)) = (boundary, cur.levels.first()) {
            set_comp(&mut p, k / 2, self.w.wall(lv.idx, k / 2, k % 2 == 1));
        }
        let node = if cur.node.is_none() { NodeId(0) } else { cur.node };
        let loc = self.locate_in(node, p, Some(cur.dir), scratch)?;
        if loc.cell.is_none() {
            return Err(RtError::InvalidInput("the march end is outside every cell"));
        }
        let mut st = TrackState::new(p, cur.dir);
        st.cell = loc.cell;
        st.region = loc.region;
        st.levels = loc.levels;
        if let Some(k) = boundary {
            // Apply Reflect/Wrap exactly as `step` does.
            let (e, from, region) = (loc.cell.get() / self.w.n_local(), st.cell, st.region);
            let s = self.w.step_wall(&mut st, e, 0.0, k, from, region)?;
            if matches!(s.end, StepEnd::Lost(_)) {
                return Err(RtError::InvalidInput("the wrap target is outside the model"));
            }
        }
        Ok(st)
    }
}
