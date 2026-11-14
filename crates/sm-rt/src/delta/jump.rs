//! Region jump: `fire` over the faces that bound the current region instead of the cell walk.
//!
//! Per region a BVH holds the faces that separate it from another region or from the outside.
//! From a state inside region `R` the first crossing of those faces that leaves `R` is the end
//! of the region. The state is then placed on that face in the cell on `R`'s side and one
//! ordinary `step` applies the event (`RegionBoundary`, deck, complement) exactly as the walk
//! would, so the end kind, ids and `to_cell` agree; the distance agrees to rounding. Anything
//! unusual (a crossing that enters `R` before the exit, no exit, a lost step) falls back to the
//! walk, so the jump never changes a result beyond the documented tolerance.

use crate::bvh::{Box6, Bvh, EMPTY_BOX, union};
use crate::core::NONE;
use crate::tracker::MeshTracker;
use crate::vec3::{finite3, linf, madd};
use rayon::prelude::*;
use sm_rt_api::{CellId, FireResult, RegionId, Result, RtError, Scratch, StepEnd, Tracker, TrackState};

/// Per-region face BVHs.
#[derive(Debug)]
pub(crate) struct JumpIndex {
    bvh: Vec<Bvh>,
}

impl JumpIndex {
    pub fn build(core: &crate::core::Core, nreg: usize, leaf_size: usize) -> Self {
        let nf = core.n_faces();
        let fbox: Vec<Box6> = (0..nf)
            .into_par_iter()
            .map(|f| {
                let mut b = EMPTY_BOX;
                for &v in core.face_verts(f) {
                    let p = core.pos[v as usize];
                    b = union(&b, &[p[0], p[1], p[2], p[0], p[1], p[2]]);
                }
                b
            })
            .collect();
        let region = |c: u32| if c == NONE { NONE } else { core.c_region[c as usize] };
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); nreg];
        for f in 0..nf {
            if core.f_surface[f] == NONE {
                continue;
            }
            let (l, r) = (region(core.f_left[f]), region(core.f_right[f]));
            if l == r {
                continue;
            }
            for x in [l, r] {
                if x != NONE {
                    lists[x as usize].push(f as u32);
                }
            }
        }
        let bvh = lists.into_par_iter().map(|ids| Bvh::build(&fbox, ids, leaf_size)).collect();
        Self { bvh }
    }

    pub fn bytes(&self) -> usize {
        self.bvh.iter().map(Bvh::bytes).sum()
    }
}

fn tie_width(o: [f64; 3], t: f64) -> f64 {
    64.0 * f64::EPSILON * (1.0 + t.abs().min(1e300) + linf(o))
}

/// The cell walk of `Tracker::fire`'s default implementation.
fn walk(t: &MeshTracker, st: &mut TrackState, limit: f64, max_steps: u32, sc: &mut Scratch) -> Result<FireResult> {
    let mut total = 0.0;
    let mut guard = crate::graze::Guard::new();
    for _ in 0..max_steps {
        let s = t.step(st, (limit - total).max(0.0), sc)?;
        total += s.dist;
        if !matches!(s.end, StepEnd::CellFace { .. } | StepEnd::SlotChange { .. }) {
            return Ok(FireResult::new(total, s.end));
        }
        match t.graze_check(&mut guard, s.dist, st, (limit - total).max(0.0), sc) {
            Ok(hop) => total += hop.unwrap_or(0.0),
            Err(l) => return Ok(FireResult::new(total, StepEnd::Lost(l))),
        }
    }
    Ok(FireResult::new(total, StepEnd::Lost(sm_rt_api::lost(st, sm_rt_api::LostCause::MaxStepsExceeded))))
}

/// `fire` with the region jump (see the module docs).
pub(crate) fn fire(t: &MeshTracker, st: &mut TrackState, limit: f64, max_steps: u32, sc: &mut Scratch) -> Result<FireResult> {
    if limit.is_nan() || limit < 0.0 {
        return Err(RtError::InvalidInput("max_dist must be >= 0 and not NaN"));
    }
    if !finite3([st.pos.x, st.pos.y, st.pos.z]) || !finite3([st.dir.x, st.dir.y, st.dir.z]) {
        return Err(RtError::NonFinite("state"));
    }
    let saved = st.clone();
    match try_jump(t, st, limit, sc) {
        Ok(Some(r)) => {
            sc.counters.jumps += 1;
            Ok(r)
        }
        Ok(None) => {
            *st = saved;
            walk(t, st, limit, max_steps, sc)
        }
        Err(e) => {
            *st = saved;
            Err(e)
        }
    }
}

fn try_jump(t: &MeshTracker, st: &mut TrackState, limit: f64, sc: &mut Scratch) -> Result<Option<FireResult>> {
    let Some(jump) = t.jump_index() else { return Ok(None) };
    let core = &t.core;
    let cell = st.cell;
    if cell.is_none() || cell == CellId::COMPLEMENT || cell.get() >= core.n_cells() {
        return Ok(None);
    }
    let r = core.c_region[cell.get()];
    let Some(bvh) = jump.bvh.get(r as usize) else { return Ok(None) };
    let (o, d) = ([st.pos.x, st.pos.y, st.pos.z], [st.dir.x, st.dir.y, st.dir.z]);
    let tol = t.tol();
    let reg = |c: u32| if c == NONE { NONE } else { core.c_region[c as usize] };
    // The exiting crossings that tie with the nearest one (within the ray tie width): the
    // lowest face id wins, as in the walk's exit rule.
    let mut ties = [(f64::INFINITY, NONE); 8];
    let mut nt = 0usize;
    let mut min_t = f64::INFINITY;
    let mut enter_min = f64::INFINITY;
    let (ids, counters) = (&mut sc.ids, &mut sc.counters);
    bvh.visit_ray((o, d), tol, (-tol, f64::INFINITY), ids, |f| {
        counters.face_tests += 1;
        let fi = f as usize;
        let (l, rt) = (reg(core.f_left[fi]), reg(core.f_right[fi]));
        core.face_crossings(fi, o, d, |tc, along| {
            if tc < -tol {
                return;
            }
            let exits = if along { l == r && rt != r } else { rt == r && l != r };
            if !exits {
                enter_min = enter_min.min(tc);
                return;
            }
            if tc < min_t {
                min_t = tc;
                let w = tie_width(o, min_t);
                let mut k = 0;
                for i in 0..nt {
                    if ties[i].0 <= min_t + w {
                        ties[k] = ties[i];
                        k += 1;
                    }
                }
                nt = k;
            }
            if tc <= min_t + tie_width(o, min_t) && nt < ties.len() {
                ties[nt] = (tc, f);
                nt += 1;
            }
        });
        if nt == 0 { f64::INFINITY } else { min_t + tie_width(o, min_t) }
    });
    let best = ties[..nt].iter().copied().min_by(|a, b| a.1.cmp(&b.1));
    let Some((bt, bf)) = best else { return Ok(None) };
    if enter_min > 4.0 * tol && enter_min < bt {
        return Ok(None);
    }
    if bt > limit {
        let p = madd(o, d, limit);
        let loc = t.locate_inner(p, Some(d), Some(cell), sc);
        if loc.cell.is_none() {
            return Ok(None);
        }
        st.pos = sm_core::Point3::new(p[0], p[1], p[2]);
        st.cell = loc.cell;
        st.region = loc.region;
        st.entered_via = loc.on_face;
        return Ok(Some(FireResult::new(limit, StepEnd::Limit)));
    }
    let t_exit = bt.max(0.0);
    let fi = bf as usize;
    let c_r = if reg(core.f_left[fi]) == r { core.f_left[fi] } else { core.f_right[fi] };
    let q = madd(o, d, t_exit);
    st.pos = sm_core::Point3::new(q[0], q[1], q[2]);
    st.cell = CellId(c_r);
    st.region = RegionId(r);
    st.entered_via = None;
    let s = t.step(st, (limit - t_exit).max(0.0), sc)?;
    match s.end {
        StepEnd::Lost(_) | StepEnd::CellFace { .. } | StepEnd::SlotChange { .. } => Ok(None),
        end => Ok(Some(FireResult::new(t_exit + s.dist, end))),
    }
}
