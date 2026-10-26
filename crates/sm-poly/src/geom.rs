//! Small allocation-free vector helpers and the planarity / coplanarity predicates.

/// A point or vector as plain coordinates.
pub(crate) type V3 = [f64; 3];

#[inline]
pub(crate) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub(crate) fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub(crate) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub(crate) fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// Mean of points, summed in order.
#[inline]
pub(crate) fn mean(pts: &[V3]) -> V3 {
    let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
    for p in pts {
        sx += p[0];
        sy += p[1];
        sz += p[2];
    }
    let n = pts.len().max(1) as f64;
    [sx / n, sy / n, sz / n]
}

/// The `GeomTol::planarity` metric of a polygon: the largest distance of a vertex from the
/// plane through the vertex mean with the fan area normal, relative to the square root of the
/// area. Same arithmetic as `sm_mesh_api::validate_geometry`; 0 for triangles and degenerate
/// polygons.
pub(crate) fn nonplanarity(pts: &[V3]) -> f64 {
    let n = pts.len();
    if n <= 3 {
        return 0.0;
    }
    let apex = mean(pts);
    let mut a = [0.0; 3];
    for i in 0..n {
        let c = cross(sub(pts[i], apex), sub(pts[(i + 1) % n], apex));
        a = [a[0] + c[0] * 0.5, a[1] + c[1] * 0.5, a[2] + c[2] * 0.5];
    }
    let len = norm(a);
    if len == 0.0 {
        return 0.0;
    }
    let inv = 1.0 / len;
    let nh = [a[0] * inv, a[1] * inv, a[2] * inv];
    let dev = pts
        .iter()
        .map(|p| dot(sub(*p, apex), nh).abs())
        .fold(0.0, f64::max);
    dev / len.sqrt()
}

/// True if the triangles `(a, b, c)` and `(a, b, d)` that share the edge `a-b` lie in one plane:
/// `d` is within `rel * L` of the plane of the first and `c` within `rel * L` of the plane of
/// the second, `L` the longest of the five edges. Symmetric in the two triangles.
pub(crate) fn coplanar(a: V3, b: V3, c: V3, d: V3, rel: f64) -> bool {
    let ab = sub(b, a);
    let (ac, ad) = (sub(c, a), sub(d, a));
    let (n1, n2) = (cross(ab, ac), cross(ab, ad));
    let (l1, l2) = (norm(n1), norm(n2));
    if l1 == 0.0 || l2 == 0.0 {
        return false;
    }
    let len = [ab, ac, ad, sub(d, b), sub(c, b)]
        .iter()
        .map(|v| norm(*v))
        .fold(0.0, f64::max);
    let tol = rel * len;
    (dot(ad, n1) / l1).abs() <= tol && (dot(ac, n2) / l2).abs() <= tol
}
