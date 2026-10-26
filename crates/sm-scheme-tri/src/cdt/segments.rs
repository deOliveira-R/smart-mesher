//! Segment recovery by Sloan flips, Lawson legalization and the parity flood fill.

use super::build::strictly_between;
use super::{C_BND, C_EMB, C_NONE, Cdt, DEAD, GHOST, NONE, Pslg, SegmentKind, TriError, key};
use sm_math::Sign;
use std::collections::VecDeque;

enum Step {
    Done,
    /// The segment passes exactly through this vertex: recover it in two parts.
    Split(u32),
}

impl Cdt {
    /// Recover every segment, in the canonical order of the Hilbert ranks of its end points.
    pub(super) fn recover_all(&mut self, input: &Pslg, rank: &[u32]) -> Result<(), TriError> {
        let mut order: Vec<u32> = (0..input.segments.len() as u32).collect();
        order.sort_by_key(|&i| {
            let s = input.segments[i as usize];
            let (ra, rb) = (rank[s.a as usize], rank[s.b as usize]);
            (ra.min(rb), ra.max(rb), i)
        });
        for i in order {
            let s = input.segments[i as usize];
            let code = match s.kind {
                SegmentKind::Boundary => C_BND,
                SegmentKind::Embedded => C_EMB,
            };
            let mut work = vec![(s.a, s.b)];
            while let Some((a, b)) = work.pop() {
                match self.recover_one(i, a, b, code)? {
                    Step::Done => {}
                    Step::Split(r) => {
                        work.push((r, b));
                        work.push((a, r));
                    }
                }
            }
        }
        Ok(())
    }

    /// Same direction test for collinear `p`, `b` seen from `a` (exact: signs of differences).
    fn same_direction(&self, a: u32, p: u32, b: u32) -> bool {
        let (pa, pp, pb) = (self.p(a), self.p(p), self.p(b));
        (0..2).all(|k| (pp[k] - pa[k]).partial_cmp(&0.0) == (pb[k] - pa[k]).partial_cmp(&0.0))
            && pp != pa
    }

    fn mark_edge(&mut self, a: u32, b: u32, id: u32, code: u8) -> Result<(), TriError> {
        let (t, i) = self
            .find_edge(a, b)
            .ok_or_else(|| TriError::Degenerate(format!("segment edge {a}-{b} missing after recovery")))?;
        let n = self.nbr[t as usize][i];
        let c = self.code[t as usize][i].max(code);
        self.code[t as usize][i] = c;
        if n != NONE {
            let j = self.back_index(n, t);
            self.code[n as usize][j] = c;
        }
        let e = self.seg_of.entry(key(a, b)).or_insert(id);
        *e = (*e).min(id);
        Ok(())
    }

    fn recover_one(&mut self, id: u32, a: u32, b: u32, code: u8) -> Result<Step, TriError> {
        if a == b {
            return Ok(Step::Done);
        }
        if self.find_edge(a, b).is_some() || self.find_edge(b, a).is_some() {
            self.mark_edge_any(a, b, id, code)?;
            return Ok(Step::Done);
        }
        let pb = self.p(b);
        // Find the wedge at `a` that the ray a -> b enters.
        let start = self.vtri[a as usize];
        let mut t = start;
        let mut found: Option<(u32, usize)> = None;
        for _ in 0..=self.tri.len() {
            let k = self.index_of(t, a);
            let tv = self.tri[t as usize];
            if tv[2] != GHOST {
                let (p, q) = (tv[(k + 1) % 3], tv[(k + 2) % 3]);
                let o1 = self.orient(a, p, pb);
                let o2 = self.orient(a, q, pb);
                if o1 == Sign::Zero && self.same_direction(a, p, b) {
                    return Ok(Step::Split(p));
                }
                if o2 == Sign::Zero && self.same_direction(a, q, b) {
                    return Ok(Step::Split(q));
                }
                if o1 == Sign::Positive && o2 == Sign::Negative {
                    found = Some((t, k));
                    break;
                }
            }
            let nx = self.nbr[t as usize][(k + 1) % 3];
            if nx == start || nx == NONE {
                break;
            }
            t = nx;
        }
        let Some((t0, k0)) = found else {
            return Err(TriError::Degenerate(format!(
                "segment {a}-{b}: no wedge at its start"
            )));
        };
        // Walk across the triangles the segment crosses.
        let tv = self.tri[t0 as usize];
        let (mut r_v, mut l_v) = (tv[(k0 + 1) % 3], tv[(k0 + 2) % 3]);
        let mut cur = t0;
        let mut idx = k0;
        let mut crossed: Vec<(u32, u32)> = vec![(r_v, l_v)];
        loop {
            let c = self.code[cur as usize][idx];
            if c != C_NONE {
                let other = self.seg_of.get(&key(r_v, l_v)).copied().unwrap_or(u32::MAX);
                return Err(TriError::SelfIntersection { a: id, b: other });
            }
            let n = self.nbr[cur as usize][idx];
            if n == NONE || self.tri[n as usize][2] == GHOST {
                return Err(TriError::Degenerate(format!(
                    "segment {a}-{b} leaves the triangulation"
                )));
            }
            let nv = self.tri[n as usize];
            let r = *nv
                .iter()
                .find(|&&x| x != r_v && x != l_v)
                .ok_or_else(|| TriError::Degenerate("bad crossing triangle".into()))?;
            if r == b {
                break;
            }
            match self.orient(a, b, self.p(r)) {
                Sign::Zero => {
                    if strictly_between(self.p(a), pb, self.p(r)) {
                        return Ok(Step::Split(r));
                    }
                    return Err(TriError::Degenerate(format!(
                        "segment {a}-{b}: vertex {r} on its line beyond the end"
                    )));
                }
                Sign::Positive => l_v = r,
                Sign::Negative => r_v = r,
            }
            crossed.push((r_v, l_v));
            // The next edge is (r_v, l_v); in `n` its opposite vertex is the dropped one.
            cur = n;
            idx = nv
                .iter()
                .position(|&x| x != r_v && x != l_v)
                .ok_or_else(|| TriError::Degenerate("bad crossing triangle".into()))?;
        }
        let fresh = self.sloan(a, b, crossed)?;
        // Constrain the segment before legalizing, or Lawson flips could remove it again.
        self.mark_edge(a, b, id, code)?;
        self.legalize(fresh)?;
        Ok(Step::Done)
    }

    fn mark_edge_any(&mut self, a: u32, b: u32, id: u32, code: u8) -> Result<(), TriError> {
        if self.find_edge(a, b).is_some() {
            self.mark_edge(a, b, id, code)
        } else {
            self.mark_edge(b, a, id, code)
        }
    }

    fn crosses(&self, a: u32, b: u32, x: u32, y: u32) -> bool {
        let (ox, oy) = (self.orient(a, b, self.p(x)), self.orient(a, b, self.p(y)));
        (ox == Sign::Positive && oy == Sign::Negative)
            || (ox == Sign::Negative && oy == Sign::Positive)
    }

    /// Remove the crossed edges by flipping, then restore the Delaunay property around them.
    fn sloan(&mut self, a: u32, b: u32, crossed: Vec<(u32, u32)>) -> Result<Vec<(u32, u32)>, TriError> {
        let mut queue: VecDeque<(u32, u32)> = crossed.iter().copied().collect();
        let mut fresh: Vec<(u32, u32)> = Vec::new();
        let mut budget = 1000 + 50 * crossed.len() * crossed.len();
        while let Some((u, v)) = queue.pop_front() {
            if budget == 0 {
                return Err(TriError::Degenerate("segment recovery did not converge".into()));
            }
            budget -= 1;
            let (t, i) = self
                .find_edge(u, v)
                .or_else(|| self.find_edge(v, u))
                .ok_or_else(|| TriError::Degenerate("crossed edge vanished".into()))?;
            let n = self.nbr[t as usize][i];
            let x = self.tri[t as usize][i];
            let j = self.back_index(n, t);
            let y = self.tri[n as usize][j];
            let (o1, o2) = (self.orient(x, y, self.p(u)), self.orient(x, y, self.p(v)));
            let convex = (o1 == Sign::Positive && o2 == Sign::Negative)
                || (o1 == Sign::Negative && o2 == Sign::Positive);
            if !convex {
                queue.push_back((u, v));
                continue;
            }
            let (p, q) = self.flip(t, i);
            if self.crosses(a, b, p, q) {
                queue.push_back((p, q));
            } else {
                fresh.push((p, q));
            }
        }
        // The new edges (the segment itself is among them) are legalized by the caller.
        Ok(fresh)
    }

    /// Flip unconstrained edges whose opposite vertex is strictly inside the circumcircle,
    /// until none is left. Only strictly convex quads are flipped; ties are "no flip".
    pub(super) fn legalize(&mut self, mut stack: Vec<(u32, u32)>) -> Result<(), TriError> {
        let mut budget = 64 + 64 * self.tri.len();
        while let Some((u, v)) = stack.pop() {
            if budget == 0 {
                return Err(TriError::Degenerate("legalization did not converge".into()));
            }
            budget -= 1;
            let Some((t, i)) = self.find_edge(u, v).or_else(|| self.find_edge(v, u)) else {
                continue;
            };
            if self.code[t as usize][i] != C_NONE {
                continue;
            }
            let n = self.nbr[t as usize][i];
            if n == NONE || self.tri[n as usize][2] == GHOST || self.tri[t as usize][2] == GHOST {
                continue;
            }
            let x = self.tri[t as usize][i];
            let j = self.back_index(n, t);
            let y = self.tri[n as usize][j];
            if !self.in_circle(t, self.p(y)) {
                continue;
            }
            let (o1, o2) = (self.orient(x, y, self.p(u)), self.orient(x, y, self.p(v)));
            let convex = (o1 == Sign::Positive && o2 == Sign::Negative)
                || (o1 == Sign::Negative && o2 == Sign::Positive);
            if !convex {
                continue;
            }
            self.flip(t, i);
            stack.extend([(x, u), (x, v), (y, u), (y, v)]);
        }
        Ok(())
    }

    /// Keep the inside by parity (a Boundary segment toggles, an Embedded one does not) and drop
    /// the ghosts. Without any Boundary segment everything is kept.
    pub(super) fn flood_fill(&mut self, any_boundary: bool) -> Result<(), TriError> {
        let n = self.tri.len();
        // 0 = unvisited, 1 = outside, 2 = inside.
        let mut state = vec![0u8; n];
        let mut queue: VecDeque<u32> = VecDeque::new();
        for g in 0..n as u32 {
            if self.tri[g as usize][0] == DEAD || self.tri[g as usize][2] != GHOST {
                continue;
            }
            let r = self.nbr[g as usize][2];
            let inside = any_boundary && self.code[g as usize][2] == C_BND;
            let want = if inside || !any_boundary { 2 } else { 1 };
            if state[r as usize] == 0 {
                state[r as usize] = want;
                queue.push_back(r);
            } else if state[r as usize] != want {
                return Err(TriError::Inconsistent("boundary parity conflict at the hull".into()));
            }
        }
        while let Some(t) = queue.pop_front() {
            for i in 0..3 {
                let m = self.nbr[t as usize][i];
                if m == NONE || self.tri[m as usize][2] == GHOST {
                    continue;
                }
                let toggle = any_boundary && self.code[t as usize][i] == C_BND;
                let want = if toggle { 3 - state[t as usize] } else { state[t as usize] };
                if state[m as usize] == 0 {
                    state[m as usize] = want;
                    queue.push_back(m);
                } else if state[m as usize] != want {
                    return Err(TriError::Inconsistent(
                        "boundary segments do not form closed loops".into(),
                    ));
                }
            }
        }
        // Drop ghosts and outside triangles.
        for t in 0..n as u32 {
            if self.tri[t as usize][0] == DEAD {
                continue;
            }
            let keep = self.tri[t as usize][2] != GHOST && state[t as usize] == 2;
            if !keep {
                self.kill_tri(t);
            }
        }
        for t in 0..n {
            if self.tri[t][0] == DEAD {
                continue;
            }
            for i in 0..3 {
                let m = self.nbr[t][i];
                if m != NONE && self.tri[m as usize][0] == DEAD {
                    self.nbr[t][i] = NONE;
                }
            }
        }
        self.ghosts = false;
        for v in self.vtri.iter_mut() {
            *v = NONE;
        }
        let mut hint = NONE;
        for t in 0..n as u32 {
            if self.tri[t as usize][0] == DEAD {
                continue;
            }
            if hint == NONE {
                hint = t;
            }
            for v in self.tri[t as usize] {
                self.vtri[v as usize] = t;
            }
        }
        self.hint = hint;
        // Free list in descending order so slots are reused lowest first.
        self.free.sort_unstable_by(|a, b| b.cmp(a));
        Ok(())
    }
}
