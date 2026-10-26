//! Input validation.

use crate::error::TetError;
use crate::geom::{V3, coord_ok};
use sm_math::PredicateError;

fn norm0(x: f64) -> f64 {
    if x == 0.0 { 0.0 } else { x }
}

/// Finite, in exact range, and no duplicate coordinates.
pub(crate) fn check_points(points: &[V3]) -> Result<(), TetError> {
    for p in points {
        if !p.iter().all(|&x| x.is_finite() && coord_ok(x)) {
            return Err(TetError::OutOfRange(PredicateError::OutOfRange));
        }
    }
    let idx = lex_order(points);
    for w in idx.windows(2) {
        let (p, q) = (points[w[0] as usize], points[w[1] as usize]);
        if (0..3).all(|k| norm0(p[k]) == norm0(q[k])) {
            return Err(TetError::DuplicatePoint { a: w[0], b: w[1] });
        }
    }
    Ok(())
}

/// Point indices in lexicographic coordinate order (ties by index).
pub(crate) fn lex_order(points: &[V3]) -> Vec<u32> {
    let mut idx: Vec<u32> = (0..points.len() as u32).collect();
    idx.sort_unstable_by(|&a, &b| {
        let (p, q) = (points[a as usize], points[b as usize]);
        norm0(p[0])
            .total_cmp(&norm0(q[0]))
            .then(norm0(p[1]).total_cmp(&norm0(q[1])))
            .then(norm0(p[2]).total_cmp(&norm0(q[2])))
            .then(a.cmp(&b))
    });
    idx
}

fn find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// Check the surface: indices, degenerate triangles, closure/manifoldness with consistent
/// orientation, and shell volumes.
pub(crate) fn check_surface(points: &[V3], tris: &[[u32; 3]]) -> Result<(), TetError> {
    check_points(points)?;
    let n = points.len() as u32;
    for (i, t) in tris.iter().enumerate() {
        let tri = i as u32;
        if t.iter().any(|&x| x >= n) || t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return Err(TetError::DegenerateTri { tri });
        }
        let [a, b, c] = t.map(|x| points[x as usize]);
        let o2 = |p: [usize; 2]| sm_math::orient2d([a[p[0]], a[p[1]]], [b[p[0]], b[p[1]]], [c[p[0]], c[p[1]]]);
        if o2([0, 1]).is_zero() && o2([1, 2]).is_zero() && o2([2, 0]).is_zero() {
            return Err(TetError::DegenerateTri { tri });
        }
    }
    // Edge use table: (min, max, forward, tri).
    let mut uses: Vec<(u32, u32, bool, u32)> = Vec::with_capacity(tris.len() * 3);
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            uses.push((a.min(b), a.max(b), a < b, i as u32));
        }
    }
    uses.sort_unstable();
    let mut parent: Vec<u32> = (0..tris.len() as u32).collect();
    let mut i = 0;
    while i < uses.len() {
        let mut j = i + 1;
        while j < uses.len() && uses[j].0 == uses[i].0 && uses[j].1 == uses[i].1 {
            j += 1;
        }
        let edge = [uses[i].0, uses[i].1];
        match j - i {
            1 => return Err(TetError::NotClosed { edge }),
            2 => {
                if uses[i].2 == uses[i + 1].2 {
                    return Err(TetError::NotClosed { edge });
                }
                let (ra, rb) = (find(&mut parent, uses[i].3), find(&mut parent, uses[i + 1].3));
                if ra != rb {
                    parent[ra.max(rb) as usize] = ra.min(rb);
                }
            }
            _ => return Err(TetError::NonManifold { edge }),
        }
        i = j;
    }
    // Shell volumes (divergence theorem about the centroid of the points).
    let mut c = [0.0; 3];
    for p in points {
        for k in 0..3 {
            c[k] += p[k];
        }
    }
    for x in &mut c {
        *x /= points.len().max(1) as f64;
    }
    let mut vol: std::collections::BTreeMap<u32, (f64, f64)> = std::collections::BTreeMap::new();
    for (i, t) in tris.iter().enumerate() {
        let r = find(&mut parent, i as u32);
        let q = |k: usize| crate::geom::sub(points[t[k] as usize], c);
        let v = crate::geom::dot(q(0), crate::geom::cross(q(1), q(2))) / 6.0;
        let e = vol.entry(r).or_insert((0.0, 0.0));
        e.0 += v;
        e.1 += v.abs();
    }
    let mut total = 0.0;
    let mut scale = 0.0;
    for (&root, &(v, a)) in &vol {
        if v.abs() <= 1e-13 * a {
            return Err(TetError::InvertedShell { tri: root });
        }
        total += v;
        scale += a;
    }
    if total <= 1e-13 * scale {
        let tri = vol.keys().next().copied().unwrap_or(0);
        return Err(TetError::InvertedShell { tri });
    }
    Ok(())
}
