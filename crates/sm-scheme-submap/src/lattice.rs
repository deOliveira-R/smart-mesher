//! The pure lattice layer: the integer polygon a face's logical sides trace in the `(i, j)`
//! frame, with exact validation, Pick's theorem and point location. No topology types.
//!
//! Every logical side is an axis-parallel run of `lens[k]` unit steps along `dirs[k]` (the
//! solved interval sum of the side). The polygon starts at `(0, 0)` and must close, have the
//! face on its left (positive signed area) and be simple. The two interval rows
//! `sum(+i) = sum(-i)` and `sum(+j) = sum(-j)` guarantee closure only, so a spiral or a deep
//! U can still overlap itself; that is caught here, exactly, in integers.

use crate::SubmapError;
use sm_scheme_api::SideDir;

/// Unit step of a side direction in the `(i, j)` lattice.
pub const fn step(d: SideDir) -> (i64, i64) {
    match d {
        SideDir::PlusI => (1, 0),
        SideDir::PlusJ => (0, 1),
        SideDir::MinusI => (-1, 0),
        SideDir::MinusJ => (0, -1),
    }
}

/// Where a lattice point lies relative to a [`LatticePolygon`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Location {
    /// Not in the closed polygon.
    Outside,
    /// On the boundary (a vertex or on a side).
    Boundary,
    /// In the open interior.
    Inside,
}

/// A simple, positively oriented, closed lattice polygon with axis-parallel sides.
///
/// Build it with [`lattice_polygon`], which validates it. The fields are public for reading;
/// a hand-built polygon must keep the three vectors the same non-zero length, `lens` positive
/// and the sides closed, simple and counter-clockwise, or [`side`](Self::side) and
/// [`decompose`](crate::blocks::decompose) may panic or give meaningless results.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LatticePolygon {
    /// Start point of every side; side `k` runs from `verts[k]` to `verts[(k + 1) % n]`.
    pub verts: Vec<(i64, i64)>,
    /// Direction of every side.
    pub dirs: Vec<SideDir>,
    /// Length of every side in lattice steps (its interval sum), all positive.
    pub lens: Vec<u32>,
}

/// Build and validate the lattice polygon of `sides = [(direction, interval sum)]`, starting at
/// `(0, 0)`. Checks, in this order: at least four sides, no zero length, closure, positive
/// signed area, simplicity (no two non-adjacent sides intersect or touch, adjacent sides do not
/// fold back onto each other).
pub fn lattice_polygon(sides: &[(SideDir, u32)]) -> Result<LatticePolygon, SubmapError> {
    let n = sides.len();
    if n < 4 {
        return Err(SubmapError::TooFewSides(n));
    }
    let (mut x, mut y) = (0i64, 0i64);
    let mut verts = Vec::with_capacity(n);
    for (k, (d, l)) in sides.iter().enumerate() {
        if *l == 0 {
            return Err(SubmapError::ZeroLength { side: k });
        }
        verts.push((x, y));
        let (dx, dy) = step(*d);
        x = x
            .checked_add(dx * i64::from(*l))
            .ok_or(SubmapError::Overflow)?;
        y = y
            .checked_add(dy * i64::from(*l))
            .ok_or(SubmapError::Overflow)?;
    }
    if (x, y) != (0, 0) {
        return Err(SubmapError::NotClosed { dx: x, dy: y });
    }
    let poly = LatticePolygon {
        verts,
        dirs: sides.iter().map(|s| s.0).collect(),
        lens: sides.iter().map(|s| s.1).collect(),
    };
    let area2 = poly.area2();
    if area2 <= 0 {
        return Err(SubmapError::NotPositive { area2 });
    }
    poly.check_simple()?;
    Ok(poly)
}

impl LatticePolygon {
    /// Number of sides.
    pub fn len(&self) -> usize {
        self.verts.len()
    }

    /// True for the empty polygon (never returned by [`lattice_polygon`]).
    pub fn is_empty(&self) -> bool {
        self.verts.is_empty()
    }

    /// End points `(from, to)` of side `k`.
    ///
    /// # Panics
    /// If `k >= self.len()`.
    pub fn side(&self, k: usize) -> ((i64, i64), (i64, i64)) {
        debug_assert!(k < self.verts.len(), "side index out of range");
        (self.verts[k], self.verts[(k + 1) % self.verts.len()])
    }

    /// Twice the signed area (shoelace, exact).
    pub fn area2(&self) -> i128 {
        let n = self.verts.len();
        (0..n)
            .map(|k| {
                let (a, b) = (self.verts[k], self.verts[(k + 1) % n]);
                i128::from(a.0) * i128::from(b.1) - i128::from(b.0) * i128::from(a.1)
            })
            .sum()
    }

    /// Bounding box `(x_min, y_min, x_max, y_max)`.
    pub fn bbox(&self) -> (i64, i64, i64, i64) {
        let mut b = (i64::MAX, i64::MAX, i64::MIN, i64::MIN);
        for &(x, y) in &self.verts {
            b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
        }
        b
    }

    /// Pick's theorem: `(A, B, I)` = area, boundary lattice points (the sum of the side
    /// lengths) and interior lattice points, `I = A - B/2 + 1`.
    pub fn pick_area(&self) -> (u64, u64, u64) {
        let a = (self.area2() / 2).max(0) as u64;
        let b: u64 = self.lens.iter().map(|l| u64::from(*l)).sum();
        let i = (i128::from(a) - i128::from(b / 2) + 1).max(0) as u64;
        (a, b, i)
    }

    /// Exact location of the lattice point `(a, b)`.
    pub fn inside(&self, a: i64, b: i64) -> Location {
        let n = self.verts.len();
        let mut crossings = 0u32;
        for k in 0..n {
            let ((x0, y0), (x1, y1)) = self.side(k);
            let (xl, xh, yl, yh) = (x0.min(x1), x0.max(x1), y0.min(y1), y0.max(y1));
            if a >= xl && a <= xh && b >= yl && b <= yh {
                return Location::Boundary;
            }
            // Ray towards +i: vertical sides strictly to the right, half-open in j.
            if x0 == x1 && x0 > a && yl <= b && b < yh {
                crossings += 1;
            }
        }
        if crossings % 2 == 1 {
            Location::Inside
        } else {
            Location::Outside
        }
    }

    /// Cells `[c0, c1)` (unit cells `c0..c1` along `i`) of the polygon in cell row `b`, the
    /// strip between lattice lines `j = b` and `j = b + 1`, sorted and disjoint. `xs` is
    /// scratch.
    pub fn row_spans_into(&self, b: i64, xs: &mut Vec<i64>, out: &mut Vec<(i64, i64)>) {
        xs.clear();
        out.clear();
        for k in 0..self.verts.len() {
            if self.dirs[k].along_i() {
                continue;
            }
            let ((x0, y0), (_, y1)) = self.side(k);
            if y0.min(y1) <= b && b < y0.max(y1) {
                xs.push(x0);
            }
        }
        xs.sort_unstable();
        for pair in xs.as_chunks::<2>().0 {
            out.push((pair[0], pair[1]));
        }
    }

    /// Exact test: closed axis-parallel sides are their own bounding boxes, so two sides meet
    /// iff their boxes overlap.
    fn check_simple(&self) -> Result<(), SubmapError> {
        let n = self.verts.len();
        let bbox = |k: usize| {
            let ((x0, y0), (x1, y1)) = self.side(k);
            (x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1))
        };
        for k in 0..n {
            let next = (k + 1) % n;
            if self.dirs[next] == self.dirs[k].reversed() {
                return Err(SubmapError::NotSimple { a: k, b: next });
            }
        }
        for a in 0..n {
            let ba = bbox(a);
            for b in a + 2..n {
                if a == 0 && b == n - 1 {
                    continue;
                }
                let bb = bbox(b);
                if ba.0 <= bb.2 && bb.0 <= ba.2 && ba.1 <= bb.3 && bb.1 <= ba.3 {
                    return Err(SubmapError::NotSimple { a, b });
                }
            }
        }
        Ok(())
    }
}

/// One interval `[a0, a1]` of consecutive lattice points of a row; `base` is the ordinal of
/// `(a0, row)`.
#[derive(Clone, Copy, Debug)]
struct Run {
    a0: i64,
    a1: i64,
    base: u32,
}

/// Every lattice point of the closed polygon, numbered in row-major `(b, a)` order, with
/// `O(log runs)` point-to-ordinal lookup. Rows are stored as CSR.
#[derive(Clone, Debug)]
pub(crate) struct PointRows {
    y_min: i64,
    row_ptr: Vec<u32>,
    runs: Vec<Run>,
    total: u32,
}

impl PointRows {
    /// Index the closed point set of `p`. `None` if it has more than `u32::MAX` points.
    pub(crate) fn build(p: &LatticePolygon) -> Option<Self> {
        let (_, y_min, _, y_max) = p.bbox();
        let rows = usize::try_from(y_max - y_min + 1).ok()?;
        let mut row_ptr = Vec::with_capacity(rows + 1);
        let mut runs: Vec<Run> = Vec::new();
        let (mut xs, mut prev, mut cur, mut merged) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut total = 0u64;
        for b in y_min..=y_max {
            if b < y_max {
                p.row_spans_into(b, &mut xs, &mut cur);
            } else {
                cur.clear();
            }
            // Points of line `b` touch the cells of rows b - 1 (prev) and b (cur).
            merged.clear();
            merged.extend(prev.iter().chain(cur.iter()).copied());
            merged.sort_unstable();
            row_ptr.push(u32::try_from(runs.len()).ok()?);
            let mut open: Option<(i64, i64)> = None;
            for &(c0, c1) in &merged {
                match open {
                    Some((o0, o1)) if c0 <= o1 => open = Some((o0, o1.max(c1))),
                    Some((o0, o1)) => {
                        runs.push(Run {
                            a0: o0,
                            a1: o1,
                            base: u32::try_from(total).ok()?,
                        });
                        total += (o1 - o0 + 1) as u64;
                        open = Some((c0, c1));
                    }
                    None => open = Some((c0, c1)),
                }
            }
            if let Some((o0, o1)) = open {
                runs.push(Run {
                    a0: o0,
                    a1: o1,
                    base: u32::try_from(total).ok()?,
                });
                total += (o1 - o0 + 1) as u64;
            }
            std::mem::swap(&mut prev, &mut cur);
        }
        row_ptr.push(u32::try_from(runs.len()).ok()?);
        Some(Self {
            y_min,
            row_ptr,
            runs,
            total: u32::try_from(total).ok()?,
        })
    }

    /// Number of lattice points of the closed polygon.
    pub(crate) fn total(&self) -> u32 {
        self.total
    }

    /// Ordinal of `(a, b)` if it belongs to the closed polygon.
    pub(crate) fn ordinal(&self, a: i64, b: i64) -> Option<u32> {
        let r = usize::try_from(b.checked_sub(self.y_min)?).ok()?;
        let (lo, hi) = (
            *self.row_ptr.get(r)? as usize,
            *self.row_ptr.get(r + 1)? as usize,
        );
        let row = &self.runs[lo..hi];
        let at = row.partition_point(|run| run.a1 < a);
        let run = row.get(at)?;
        (run.a0 <= a).then(|| run.base + (a - run.a0) as u32)
    }

    /// All points in ordinal (row-major) order.
    #[cfg(test)]
    pub(crate) fn points(&self) -> impl Iterator<Item = (i64, i64)> + '_ {
        (0..self.row_ptr.len() - 1).flat_map(move |r| {
            let b = self.y_min + r as i64;
            self.runs[self.row_ptr[r] as usize..self.row_ptr[r + 1] as usize]
                .iter()
                .flat_map(move |run| (run.a0..=run.a1).map(move |a| (a, b)))
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use SideDir::{MinusI, MinusJ, PlusI, PlusJ};

    /// The counter-clockwise staircase with column widths `w` and heights `h` (decreasing to
    /// the right), as a side list starting at the origin.
    pub(crate) fn staircase_sides(w: &[u32], h: &[u32]) -> Vec<(SideDir, u32)> {
        assert_eq!(w.len(), h.len());
        let k = w.len() - 1;
        let mut s = vec![(PlusI, w.iter().sum()), (PlusJ, h[k])];
        for j in (0..=k).rev() {
            s.push((MinusI, w[j]));
            if j > 0 {
                s.push((PlusJ, h[j - 1] - h[j]));
            }
        }
        s.push((MinusJ, h[0]));
        s
    }

    /// Independent cell-based counts `(A, B, I)` of a staircase.
    pub(crate) fn brute_counts(w: &[u32], h: &[u32]) -> (u64, u64, u64) {
        let total: u32 = w.iter().sum();
        let col_h: Vec<u32> = (0..total)
            .scan((0usize, 0u32), |st, c| {
                while c >= st.1 + w[st.0] {
                    st.1 += w[st.0];
                    st.0 += 1;
                }
                Some(h[st.0])
            })
            .collect();
        let cell = |c: i64, r: i64| {
            c >= 0 && c < i64::from(total) && r >= 0 && r < i64::from(col_h[c as usize])
        };
        let (mut a, mut closed, mut interior) = (0u64, 0u64, 0u64);
        for c in 0..i64::from(total) {
            for r in 0..i64::from(col_h[c as usize]) {
                debug_assert!(cell(c, r));
                a += 1;
            }
        }
        let hmax = i64::from(*h.iter().max().unwrap_or(&0));
        for x in 0..=i64::from(total) {
            for y in 0..=hmax {
                let cells = [
                    cell(x - 1, y - 1),
                    cell(x, y - 1),
                    cell(x - 1, y),
                    cell(x, y),
                ];
                if cells.iter().any(|c| *c) {
                    closed += 1;
                    if cells.iter().all(|c| *c) {
                        interior += 1;
                    }
                }
            }
        }
        (a, closed - interior, interior)
    }

    /// Small deterministic generator for seeded tests.
    pub(crate) struct Lcg(pub u64);
    impl Lcg {
        pub(crate) fn next(&mut self, lo: u32, hi: u32) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            lo + ((self.0 >> 33) as u32) % (hi - lo + 1)
        }
    }

    pub(crate) fn random_staircase(rng: &mut Lcg) -> (Vec<u32>, Vec<u32>) {
        let k = rng.next(1, 4) as usize;
        let w: Vec<u32> = (0..=k).map(|_| rng.next(1, 4)).collect();
        let mut h = vec![0u32; k + 1];
        h[k] = rng.next(1, 4);
        for j in (0..k).rev() {
            h[j] = h[j + 1] + rng.next(1, 4);
        }
        (w, h)
    }

    fn l_sides() -> Vec<(SideDir, u32)> {
        vec![
            (PlusI, 4),
            (PlusJ, 4),
            (PlusI, 4),
            (PlusJ, 4),
            (MinusI, 8),
            (MinusJ, 8),
        ]
    }

    #[test]
    fn the_l_is_closed_positive_and_simple() {
        let p = lattice_polygon(&l_sides()).expect("L");
        assert_eq!(
            p.verts,
            vec![(0, 0), (4, 0), (4, 4), (8, 4), (8, 8), (0, 8)]
        );
        assert_eq!(p.pick_area(), (48, 32, 33));
        assert_eq!(p.area2(), 96);
        // The 3-3-2-2 counts of acceptance 2.
        let q = lattice_polygon(&[
            (PlusI, 3),
            (PlusJ, 3),
            (PlusI, 2),
            (PlusJ, 2),
            (MinusI, 5),
            (MinusJ, 5),
        ])
        .expect("small L");
        assert_eq!(q.pick_area(), (19, 20, 10));
    }

    #[test]
    fn a_non_simple_u_is_rejected_with_a_lattice_message() {
        let u = [
            (PlusI, 5),
            (PlusJ, 4),
            (MinusI, 1),
            (MinusJ, 4),
            (MinusI, 2),
            (PlusJ, 4),
            (MinusI, 2),
            (MinusJ, 4),
        ];
        let e = lattice_polygon(&u).expect_err("touching notch");
        let text = e.to_string();
        assert!(
            text.contains("lattice:") && text.contains("not simple"),
            "{text}"
        );
        assert!(matches!(e, SubmapError::NotSimple { .. }));
    }

    #[test]
    fn clockwise_unclosed_and_degenerate_sequences_are_typed_errors() {
        let cw = [(PlusJ, 2), (PlusI, 3), (MinusJ, 2), (MinusI, 3)];
        let e = lattice_polygon(&cw).expect_err("clockwise");
        assert!(
            matches!(e, SubmapError::NotPositive { area2 } if area2 < 0),
            "{e}"
        );
        assert!(e.to_string().contains("lattice:"));
        let open = [(PlusI, 3), (PlusJ, 2), (MinusI, 2), (MinusJ, 2)];
        let e = lattice_polygon(&open).expect_err("unclosed");
        assert_eq!(e, SubmapError::NotClosed { dx: 1, dy: 0 });
        assert!(lattice_polygon(&open[..3]).is_err());
        let zero = [(PlusI, 3), (PlusJ, 0), (MinusI, 3), (MinusJ, 0)];
        assert!(matches!(
            lattice_polygon(&zero),
            Err(SubmapError::ZeroLength { side: 1 })
        ));
        // A side that folds straight back onto its predecessor.
        let fold = [
            (PlusI, 3),
            (MinusI, 1),
            (PlusJ, 2),
            (MinusI, 2),
            (MinusJ, 2),
        ];
        assert!(matches!(
            lattice_polygon(&fold),
            Err(SubmapError::NotSimple { .. })
        ));
        // A figure-eight closes and has zero area, a rectangle revisited has none either.
        let rect = [(PlusI, 2), (PlusJ, 2), (MinusI, 2), (MinusJ, 2)];
        assert!(lattice_polygon(&rect).is_ok());
    }

    #[test]
    fn staircases_are_accepted_and_pick_matches_brute_force_counting() {
        let mut rng = Lcg(0x5eed);
        for case in 0..200 {
            let (w, h) = random_staircase(&mut rng);
            let sides = staircase_sides(&w, &h);
            let p =
                lattice_polygon(&sides).unwrap_or_else(|e| panic!("case {case} {w:?} {h:?}: {e}"));
            assert_eq!(
                p.pick_area(),
                brute_counts(&w, &h),
                "case {case}: {w:?} {h:?}"
            );
            let rows = PointRows::build(&p).expect("rows");
            let (_, b, i) = p.pick_area();
            assert_eq!(u64::from(rows.total()), b + i, "case {case}");
        }
    }

    #[test]
    fn inside_is_exact_on_the_l() {
        let p = lattice_polygon(&l_sides()).expect("L");
        assert_eq!(p.inside(2, 2), Location::Inside);
        assert_eq!(p.inside(6, 6), Location::Inside);
        assert_eq!(p.inside(4, 5), Location::Inside); // on the (future) cut
        assert_eq!(p.inside(6, 2), Location::Outside);
        assert_eq!(p.inside(4, 4), Location::Boundary); // the reflex vertex
        assert_eq!(p.inside(6, 4), Location::Boundary);
        assert_eq!(p.inside(0, 3), Location::Boundary);
        assert_eq!(p.inside(-1, 0), Location::Outside);
        assert_eq!(p.inside(9, 8), Location::Outside);
        let rows = PointRows::build(&p).expect("rows");
        for b in -1..10 {
            for a in -1..10 {
                let want = p.inside(a, b) != Location::Outside;
                assert_eq!(rows.ordinal(a, b).is_some(), want, "({a},{b})");
            }
        }
    }

    #[test]
    fn point_rows_number_points_in_row_major_order() {
        let p = lattice_polygon(&l_sides()).expect("L");
        let rows = PointRows::build(&p).expect("rows");
        let pts: Vec<_> = rows.points().collect();
        assert_eq!(pts.len() as u32, rows.total());
        assert!(pts.windows(2).all(|w| (w[0].1, w[0].0) < (w[1].1, w[1].0)));
        for (o, (a, b)) in pts.iter().enumerate() {
            assert_eq!(rows.ordinal(*a, *b), Some(o as u32));
        }
        assert_eq!(rows.ordinal(5, 2), None);
    }

    #[test]
    fn row_spans_of_a_u_shaped_lattice_have_two_prongs() {
        // A U (not the touching one): base height 1.
        let u = [
            (PlusI, 5),
            (PlusJ, 4),
            (MinusI, 1),
            (MinusJ, 3),
            (MinusI, 2),
            (PlusJ, 3),
            (MinusI, 2),
            (MinusJ, 4),
        ];
        let p = lattice_polygon(&u).expect("U");
        let (mut xs, mut out) = (Vec::new(), Vec::new());
        p.row_spans_into(2, &mut xs, &mut out);
        assert_eq!(out, vec![(0, 2), (4, 5)]);
        p.row_spans_into(0, &mut xs, &mut out);
        assert_eq!(out, vec![(0, 5)]);
        let rows = PointRows::build(&p).expect("rows");
        let (_, b, i) = p.pick_area();
        assert_eq!(u64::from(rows.total()), b + i);
    }
}
