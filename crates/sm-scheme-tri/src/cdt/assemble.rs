//! Building a [`Cdt`] from an explicit triangle list (the advancing front's output) and
//! inserting leftover input points.

use super::{C_BND, C_EMB, C_NONE, Cdt, NONE, Pslg, SegmentKind, TriError, key};
use std::collections::BTreeMap;

impl Cdt {
    /// A triangulation made of the counter-clockwise `tris` over the points of `input` (all of
    /// them, in order), with the segments of `input` as constrained edges. The triangles must
    /// form a manifold region bounded by constrained edges; every segment must be an edge.
    ///
    /// # Errors
    /// [`TriError::Degenerate`] if a triangle is not exactly counter-clockwise, an edge has more
    /// than one triangle on a side, or a segment is not an edge of the triangles.
    pub(crate) fn from_triangles(input: &Pslg, tris: &[[u32; 3]]) -> Result<Cdt, TriError> {
        use sm_math::Sign;
        let mut c = Cdt::empty(input.points.clone());
        c.ghosts = false;
        let mut edge: BTreeMap<(u32, u32), (u32, u8)> = BTreeMap::new();
        for (t, v) in tris.iter().enumerate() {
            if c.orient(v[0], v[1], c.p(v[2])) != Sign::Positive {
                return Err(TriError::Degenerate("front produced a non-positive triangle".into()));
            }
            for i in 0..3u8 {
                let (a, b) = (v[(i as usize + 1) % 3], v[(i as usize + 2) % 3]);
                if edge.insert((a, b), (t as u32, i)).is_some() {
                    return Err(TriError::Degenerate("front produced overlapping triangles".into()));
                }
            }
        }
        for v in tris {
            c.alloc_tri(*v, [NONE; 3], [C_NONE; 3]);
        }
        for (t, v) in tris.iter().enumerate() {
            for i in 0..3usize {
                let (a, b) = (v[(i + 1) % 3], v[(i + 2) % 3]);
                if let Some(&(n, _)) = edge.get(&(b, a)) {
                    c.nbr[t][i] = n;
                }
            }
            for x in v {
                c.vtri[*x as usize] = t as u32;
            }
        }
        c.hint = 0;
        for (i, s) in input.segments.iter().enumerate() {
            let code = match s.kind {
                SegmentKind::Boundary => C_BND,
                SegmentKind::Embedded => C_EMB,
            };
            let mut found = false;
            for (a, b) in [(s.a, s.b), (s.b, s.a)] {
                if let Some(&(t, k)) = edge.get(&(a, b)) {
                    let slot = &mut c.code[t as usize][k as usize];
                    *slot = (*slot).max(code);
                    found = true;
                }
            }
            if !found {
                return Err(TriError::Degenerate(format!("segment {i} is not an edge of the front's triangles")));
            }
            let e = c.seg_of.entry(key(s.a, s.b)).or_insert(i as u32);
            *e = (*e).min(i as u32);
        }
        Ok(c)
    }

    /// Insert the input point `v` (already in the point list, in no triangle yet) into the
    /// triangle that contains it.
    ///
    /// # Errors
    /// [`TriError::Degenerate`] if no triangle contains it or it lies on a constrained edge.
    pub(crate) fn insert_input_point(&mut self, v: u32) -> Result<(), TriError> {
        let p = self.p(v);
        let Some(t0) = self.locate(p, self.hint, true) else {
            return Err(TriError::Degenerate(format!("isolated point {v} is outside the region")));
        };
        if self.tri[t0 as usize].iter().any(|&w| self.p(w) == p) {
            return Err(TriError::DuplicatePoint { a: v, b: v });
        }
        self.build_cavity(p, t0, true);
        if self.check_cavity(p, false).is_err() {
            return Err(TriError::Degenerate(format!("isolated point {v} lies on a constrained segment")));
        }
        self.apply_cavity(v);
        Ok(())
    }
}
