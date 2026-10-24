//! Path-independent contact classification for a curve against a surface.
//!
//! A hit is a place where the signed gap `s(t)` between curve and surface is within
//! `tol.linear` of zero. Whatever route found the hit (Newton from a leaf, a Bezier root, a
//! polynomial root), the *contact* is the maximal stretch of the curve around it where
//! `|s| <= tol.linear`. The contact is tangent when the gap leaves the band on the same side at
//! both ends of the stretch (a grazing touch), and a crossing when the sides differ. Hits whose
//! stretches overlap are one contact, also across the seam of a closed curve. Because the rule
//! looks only at the gap function, the answer does not depend on where Newton stopped, on knots
//! or Bezier boundaries, or on where the seam of a closed curve lies.

/// A hit: curve parameter and tangent flag.
pub(crate) type Raw = (f64, bool);

/// The signed gap along a curve, with its parameter range and closedness.
pub(crate) struct Gap<'a> {
    /// Signed gap at a parameter inside `[lo, hi]` (positive on one fixed side of the surface).
    pub f: &'a dyn Fn(f64) -> f64,
    /// Parameter range of the curve.
    pub lo: f64,
    /// End of the parameter range.
    pub hi: f64,
    /// `C(lo) == C(hi)` to tolerance: stretches may run through the seam.
    pub closed: bool,
    /// Band half-width (`tol.linear`).
    pub tol: f64,
    /// Seed step, a parameter step worth about `tol` of arc length.
    pub h0: f64,
}

impl Gap<'_> {
    fn span(&self) -> f64 {
        self.hi - self.lo
    }

    fn wrap(&self, t: f64) -> f64 {
        if self.closed { self.lo + (t - self.lo).rem_euclid(self.span()) } else { t }
    }

    fn at(&self, t: f64) -> f64 {
        (self.f)(self.wrap(t))
    }

    /// A raw hit inside the band stays; one outside is moved to the point of smallest `|s|`
    /// within a window of `4 sqrt(h0)` (sampled, then refined by ternary search), or dropped.
    fn polish(&self, (t, tan): (f64, bool)) -> Option<(f64, bool)> {
        if self.at(t).abs() <= self.tol {
            return Some((t, tan));
        }
        let w = 4.0 * self.h0.sqrt();
        let n = 64usize;
        let step = 2.0 * w / n as f64;
        let mut best = (f64::INFINITY, t);
        for i in 0..=n {
            let u = t - w + step * i as f64;
            if !self.closed && (u < self.lo || u > self.hi) {
                continue;
            }
            let v = self.at(u).abs();
            if v < best.0 {
                best = (v, u);
            }
        }
        let (mut a, mut b) = (best.1 - step, best.1 + step);
        for _ in 0..60 {
            let (m1, m2) = (a + (b - a) / 3.0, b - (b - a) / 3.0);
            if !self.closed && (m1 < self.lo || m2 > self.hi) {
                break;
            }
            if self.at(m1).abs() < self.at(m2).abs() { b = m2 } else { a = m1 }
        }
        let u = 0.5 * (a + b);
        let (vu, vb) = (self.at(u).abs(), best.0);
        let (v, u) = if vu <= vb { (vu, u) } else { (vb, best.1) };
        (v <= self.tol).then_some((self.wrap(u), tan))
    }

    /// One side of the stretch: distance from `t` to the band edge, and the side of the first
    /// sample outside the band (`None` if the curve ends, or the whole closed curve is inside).
    fn edge(&self, t: f64, dir: f64) -> (f64, Option<f64>) {
        let limit = if self.closed {
            0.5 * self.span()
        } else if dir > 0.0 {
            self.hi - t
        } else {
            t - self.lo
        };
        if limit <= 0.0 {
            return (0.0, None);
        }
        let (mut inner, mut step) = (0.0f64, self.h0.max(1e-15 * self.span()));
        let outer = loop {
            let d = step.min(limit);
            let v = self.at(t + dir * d);
            if v.abs() > self.tol {
                break (d, v);
            }
            inner = d;
            if d >= limit {
                return (limit, None);
            }
            step *= 2.0;
        };
        let (mut a, mut b) = (inner, outer.0);
        for _ in 0..40 {
            let m = 0.5 * (a + b);
            if self.at(t + dir * m).abs() <= self.tol {
                a = m;
            } else {
                b = m;
            }
        }
        (a, Some(outer.1.signum()))
    }
}

/// A contact stretch `[a, b]` (unwrapped, `a <= t <= b`) and its tangency (`None`: undecided
/// because the stretch runs into an end of an open curve).
struct Stretch {
    a: f64,
    b: f64,
    tangent: Option<bool>,
    raw_tangent: bool,
    t: f64,
}

fn stretch(g: &Gap, (t, raw_tangent): Raw) -> Stretch {
    let (dl, sl) = g.edge(t, -1.0);
    let (dr, sr) = g.edge(t, 1.0);
    let whole = g.closed && sl.is_none() && sr.is_none() && dl + dr >= g.span();
    let tangent = match (sl, sr) {
        (Some(x), Some(y)) => Some(x * y > 0.0),
        _ if whole => Some(true),
        _ => None,
    };
    Stretch { a: t - dl, b: t + dr, tangent, raw_tangent, t }
}

/// Merge `raw` hits into contacts: one hit per contact stretch, tangent decided by the sign
/// rule, ascending by parameter. A tangent contact is reported at the middle of its stretch.
pub(crate) fn merge_contacts(g: &Gap, raw: &[Raw]) -> Vec<Raw> {
    // A raw hit outside the band has no stretch around it: move it to the best point nearby
    // (smallest |s|) or drop it when none is in the band.
    let mut items: Vec<Stretch> = raw.iter().filter_map(|h| g.polish((g.wrap(h.0), h.1))).map(|h| stretch(g, h)).collect();
    items.sort_by(|x, y| x.t.total_cmp(&y.t));
    // Groups of overlapping stretches: (a, b, tangent, first t, count).
    let mut groups: Vec<(f64, f64, bool, f64, usize)> = Vec::new();
    for s in items {
        let tan = s.tangent.unwrap_or(s.raw_tangent);
        match groups.last_mut() {
            Some(l) if s.a <= l.1 => {
                l.1 = l.1.max(s.b);
                l.2 |= tan;
                l.4 += 1;
            }
            _ => groups.push((s.a, s.b, tan, s.t, 1)),
        }
    }
    if g.closed {
        // The last group meets the first through the seam.
        while groups.len() > 1 {
            let n = groups.len() - 1;
            if groups[n].1 >= groups[0].0 + g.span() {
                let Some(l) = groups.pop() else { break };
                let f = &mut groups[0];
                f.0 = f.0.min(l.0 - g.span());
                f.1 = f.1.max(l.1 - g.span());
                f.2 |= l.2;
                f.3 = l.3 - g.span();
                f.4 += l.4;
            } else {
                break;
            }
        }
    }
    let mut out: Vec<Raw> = groups
        .into_iter()
        .map(|(a, b, tan, t, _)| {
            // A grazing stretch (or a merged group) is reported at its middle.
            let rep = if tan { 0.5 * (a + b) } else { t };
            (g.wrap(rep), tan)
        })
        .collect();
    out.sort_by(|x, y| x.0.total_cmp(&y.0));
    out
}
