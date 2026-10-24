//! 1-D sampling rules shared by edges and grid-meshed faces.
//!
//! Circles use a closed-form segment count, so the chord error of an inscribed polygon is
//! known exactly (`r (1 - cos(theta/2))` with `theta = span / n`). Every other curve goes
//! through [`adaptive_params`], a bisection that tests the chord deviation at three interior
//! fractions and the turning angle at the midpoint.

use sm_geom_api::SurfaceDef;

use crate::options::TessOptions;

/// Hard cap on segments per curve (keeps `u32` indices and memory in range).
pub(crate) const MAX_SEGMENTS: usize = 1 << 20;

/// Cap on the vertices of one tessellation (edge samples plus face-interior points), 2^24.
/// Checked before the points are allocated, so a tiny `chord_tol` gives `TessError::TooLarge`.
pub(crate) const MAX_VERTICES: usize = 1 << 24;
const MAX_DEPTH: u32 = 22;

pub(crate) type P3 = [f64; 3];

fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}

/// Distance from `p` to the segment `a..b`.
fn dist_to_segment(p: P3, a: P3, b: P3) -> f64 {
    let ab = sub(b, a);
    let ap = sub(p, a);
    let l2 = dot(ab, ab);
    let t = if l2 > 0.0 { (dot(ap, ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    norm(sub(ap, [ab[0] * t, ab[1] * t, ab[2] * t]))
}

/// Surfaces whose triangles sag in both parametric directions (everything but planes, cylinders
/// and cones, which are developable: a triangle sags only along the curved direction). For the
/// cone this holds only because the rings share one point count (see `tess::edge_rule`).
pub(crate) fn is_doubly_curved(def: &SurfaceDef) -> bool {
    !matches!(def, SurfaceDef::Plane(_) | SurfaceDef::Cylinder(_) | SurfaceDef::Cone(_))
}

/// Options for sampling a grid face and the edges next to a doubly curved face: the chord budget
/// is split evenly between the two directions, since a triangle sags about `sag_u + sag_v`.
/// Developable surfaces keep the whole budget.
pub(crate) fn grid_opts(def: &SurfaceDef, o: &TessOptions) -> TessOptions {
    if is_doubly_curved(def) { TessOptions { chord_tol: 0.5 * o.chord_tol, ..*o } } else { *o }
}

/// `n + 1` parameters `lo + (hi - lo) * i / n`, both ends exact.
pub(crate) fn uniform(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    let mut v: Vec<f64> = (0..=n).map(|i| lo + (hi - lo) * (i as f64 / n as f64)).collect();
    v[0] = lo;
    v[n] = hi;
    v
}

/// Segments for a circular arc of `span` radians and `radius`: the smallest `n` whose step
/// satisfies the angle tolerance, the chord tolerance (`radius (1 - cos(step/2)) <= chord_tol`)
/// and `max_edge`. A closed circle gets at least 3.
pub(crate) fn circle_segments(span: f64, radius: f64, closed: bool, o: &TessOptions) -> Result<usize, String> {
    let theta_chord = if o.chord_tol >= radius { core::f64::consts::PI } else { 2.0 * (1.0 - o.chord_tol / radius).acos() };
    let step = theta_chord.min(o.angle_tol);
    let mut n = ((span / step) - 1e-9).ceil().max(1.0);
    if o.max_edge.is_finite() {
        n = n.max(((radius * span / o.max_edge) - 1e-9).ceil());
    }
    if closed {
        n = n.max(3.0);
    }
    if !(n.is_finite() && n <= MAX_SEGMENTS as f64) {
        return Err(format!("{n} segments for a circle of radius {radius} exceed the cap of {MAX_SEGMENTS}"));
    }
    Ok(n as usize)
}

/// Segments for a straight run of `len`: one unless `max_edge` forces more.
pub(crate) fn line_segments(len: f64, o: &TessOptions) -> Result<usize, String> {
    if !o.max_edge.is_finite() {
        return Ok(1);
    }
    let n = ((len / o.max_edge) - 1e-9).ceil().max(1.0);
    if n > MAX_SEGMENTS as f64 {
        return Err(format!("{n} segments for a line of length {len} exceed the cap of {MAX_SEGMENTS}"));
    }
    Ok(n as usize)
}

/// Adaptive parameters on `[lo, hi]` for the map `f`: ascending, both ends included.
pub(crate) fn adaptive_params(f: &dyn Fn(f64) -> P3, lo: f64, hi: f64, o: &TessOptions) -> Result<Vec<f64>, String> {
    let mut out = vec![lo];
    let (pa, pb) = (f(lo), f(hi));
    refine(f, lo, pa, hi, pb, 0, o, &mut out)?;
    if out.len() - 1 > MAX_SEGMENTS {
        return Err(format!("more than {MAX_SEGMENTS} segments"));
    }
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn refine(f: &dyn Fn(f64) -> P3, a: f64, pa: P3, b: f64, pb: P3, depth: u32, o: &TessOptions, out: &mut Vec<f64>) -> Result<(), String> {
    let m = 0.5 * (a + b);
    let pm = f(m);
    if depth < MAX_DEPTH && needs_split(f, a, pa, b, pb, pm, o) {
        refine(f, a, pa, m, pm, depth + 1, o, out)?;
        refine(f, m, pm, b, pb, depth + 1, o, out)?;
    } else {
        out.push(b);
    }
    if out.len() > MAX_SEGMENTS + 1 {
        return Err(format!("more than {MAX_SEGMENTS} segments"));
    }
    Ok(())
}

fn needs_split(f: &dyn Fn(f64) -> P3, a: f64, pa: P3, b: f64, pb: P3, pm: P3, o: &TessOptions) -> bool {
    let len = norm(sub(pb, pa));
    if len > o.max_edge {
        return true;
    }
    if dist_to_segment(pm, pa, pb) > o.chord_tol {
        return true;
    }
    for fr in [0.25, 0.75] {
        if dist_to_segment(f(a + (b - a) * fr), pa, pb) > o.chord_tol {
            return true;
        }
    }
    // Turning angle at the midpoint between the two half chords; the arc's total turn is twice that.
    let (u, v) = (sub(pm, pa), sub(pb, pm));
    let (lu, lv) = (norm(u), norm(v));
    if lu > 1e-14 && lv > 1e-14 {
        let c = (dot(u, v) / (lu * lv)).clamp(-1.0, 1.0);
        if 2.0 * c.acos() > o.angle_tol {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle(r: f64) -> impl Fn(f64) -> P3 {
        move |t| [r * t.cos(), r * t.sin(), 0.0]
    }

    #[test]
    fn circle_count_meets_chord_bound() {
        let o = TessOptions { chord_tol: 1e-3, angle_tol: 3.0, max_edge: f64::INFINITY };
        let n = circle_segments(core::f64::consts::TAU, 2.0, true, &o).unwrap();
        let theta = core::f64::consts::TAU / n as f64;
        assert!(2.0 * (1.0 - (theta / 2.0).cos()) <= 1e-3 + 1e-15);
        // One fewer segment would violate the bound.
        let theta1 = core::f64::consts::TAU / (n - 1) as f64;
        assert!(2.0 * (1.0 - (theta1 / 2.0).cos()) > 1e-3);
    }

    #[test]
    fn angle_tol_dominates_for_coarse_chord() {
        let o = TessOptions { chord_tol: 10.0, angle_tol: 0.5, max_edge: f64::INFINITY };
        let n = circle_segments(core::f64::consts::TAU, 1.0, true, &o).unwrap();
        assert_eq!(n, (core::f64::consts::TAU / 0.5).ceil() as usize);
    }

    #[test]
    fn closed_circle_has_at_least_three_segments() {
        let o = TessOptions { chord_tol: 100.0, angle_tol: core::f64::consts::PI, max_edge: f64::INFINITY };
        assert_eq!(circle_segments(core::f64::consts::TAU, 1.0, true, &o).unwrap(), 3);
        assert_eq!(circle_segments(1.0, 1.0, false, &o).unwrap(), 1);
    }

    #[test]
    fn max_edge_forces_more_segments() {
        let o = TessOptions { chord_tol: 100.0, angle_tol: core::f64::consts::PI, max_edge: 0.5 };
        assert_eq!(line_segments(2.0, &o).unwrap(), 4);
        assert_eq!(line_segments(2.0, &TessOptions::default()).unwrap(), 1);
        assert_eq!(circle_segments(core::f64::consts::TAU, 1.0, true, &o).unwrap(), 13);
    }

    #[test]
    fn adaptive_circle_is_uniform_and_within_tolerance() {
        let o = TessOptions { chord_tol: 1e-3, angle_tol: 0.5, max_edge: f64::INFINITY };
        let t = adaptive_params(&circle(3.0), 0.0, core::f64::consts::TAU, &o).unwrap();
        let n = t.len() - 1;
        assert!(n.is_power_of_two());
        let step = core::f64::consts::TAU / n as f64;
        assert!(t.windows(2).all(|w| (w[1] - w[0] - step).abs() < 1e-12));
        assert!(3.0 * (1.0 - (step / 2.0).cos()) <= 1e-3);
        assert!(step <= 0.5);
    }

    #[test]
    fn adaptive_straight_line_is_one_segment() {
        let f = |t: f64| [t, 2.0 * t, 0.0];
        let t = adaptive_params(&f, 0.0, 5.0, &TessOptions::default()).unwrap();
        assert_eq!(t, vec![0.0, 5.0]);
    }

    #[test]
    fn uniform_has_exact_ends() {
        let t = uniform(0.1, 0.7, 7);
        assert_eq!((t[0], t[7], t.len()), (0.1, 0.7, 8));
    }
}
