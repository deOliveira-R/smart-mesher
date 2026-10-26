//! Tiny 2D polygon helpers (areas, convexity, convex clipping) used for overlap areas.

/// A 2D point.
pub(crate) type P2 = [f64; 2];

/// Signed area (counter-clockwise positive).
pub(crate) fn signed_area(poly: &[P2]) -> f64 {
    let n = poly.len();
    let mut a = 0.0;
    for i in 0..n {
        let (p, q) = (poly[i], poly[(i + 1) % n]);
        a += p[0] * q[1] - q[0] * p[1];
    }
    0.5 * a
}

/// True if the polygon is convex within `eps` (collinear points allowed), either winding.
pub(crate) fn is_convex(poly: &[P2], eps: f64) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut sign = 0.0f64;
    for i in 0..n {
        let (a, b, c) = (poly[i], poly[(i + 1) % n], poly[(i + 2) % n]);
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if cross.abs() <= eps {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if cross.signum() != sign {
            return false;
        }
    }
    true
}

/// Sutherland-Hodgman: `subject` clipped to the convex polygon `clip` (both counter-clockwise).
pub(crate) fn clip_convex(subject: &[P2], clip: &[P2]) -> Vec<P2> {
    let mut out: Vec<P2> = subject.to_vec();
    let n = clip.len();
    for i in 0..n {
        if out.is_empty() {
            break;
        }
        let (a, b) = (clip[i], clip[(i + 1) % n]);
        let inside = |p: P2| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0;
        let inter = |p: P2, q: P2| -> P2 {
            let (dx, dy) = (q[0] - p[0], q[1] - p[1]);
            let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
            let den = dx * ey - dy * ex;
            if den == 0.0 {
                return p;
            }
            let t = ((a[0] - p[0]) * ey - (a[1] - p[1]) * ex) / den;
            [p[0] + t * dx, p[1] + t * dy]
        };
        let input = std::mem::take(&mut out);
        for j in 0..input.len() {
            let (cur, prev) = (input[j], input[(j + input.len() - 1) % input.len()]);
            match (inside(cur), inside(prev)) {
                (true, true) => out.push(cur),
                (true, false) => {
                    out.push(inter(prev, cur));
                    out.push(cur);
                }
                (false, true) => out.push(inter(prev, cur)),
                (false, false) => {}
            }
        }
    }
    out
}

/// Area of the intersection of two simple polygons when at least one is convex, else `None`.
pub(crate) fn intersection_area(a: &[P2], b: &[P2], eps: f64) -> Option<f64> {
    let orient = |p: &[P2]| -> Vec<P2> {
        let mut v = p.to_vec();
        if signed_area(&v) < 0.0 {
            v.reverse();
        }
        v
    };
    let (a, b) = (orient(a), orient(b));
    let (subject, clip) = if is_convex(&b, eps) {
        (&a, &b)
    } else if is_convex(&a, eps) {
        (&b, &a)
    } else {
        return None;
    };
    let c = clip_convex(subject, clip);
    Some(if c.len() < 3 { 0.0 } else { signed_area(&c).abs() })
}
