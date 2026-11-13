//! Planar geometry of hexagonal lattices (W4-RT-HEX): axial element coordinates, the hexagon of
//! one element and the element candidates of a point.
//!
//! Element `(q, r)` has its centre at `origin + c(q, r)` with, for pitch `p` (flat-to-flat, which
//! is also the centre distance of neighbours):
//!
//! * `XRows` (`LatticeKind::Hex`, pointy-top): `p (q + r/2, sqrt(3)/2 r)`; flats at `x = +-p/2`.
//! * `YRows` (`LatticeKind::HexYRows`, flat-top): `p (sqrt(3)/2 q, r + q/2)`; flats at `y = +-p/2`.
//!
//! The six neighbours are `(q, r) + {(1,0), (-1,0), (0,1), (0,-1), (1,-1), (-1,1)}` in both.

use smallvec::SmallVec;

/// Orientation of the hexagons.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Orient {
    /// Rows along x, pointy-top (`LatticeKind::Hex`).
    XRows,
    /// Rows along y, flat-top (`LatticeKind::HexYRows`).
    YRows,
}

/// `sqrt(3) / 2`, spelled as in `Lattice::element_center` so the centres are bit-equal to it.
fn s32() -> f64 {
    3.0_f64.sqrt() / 2.0
}

/// Axial neighbour offsets.
pub(crate) const NEIGHBOURS: [(i32, i32); 6] = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, -1), (-1, 1)];

/// Centre of element `(q, r)` relative to element `(0, 0)`.
pub(crate) fn center(o: Orient, p: f64, q: i32, r: i32) -> [f64; 2] {
    let (q, r) = (f64::from(q), f64::from(r));
    match o {
        Orient::XRows => [p * (q + r / 2.0), p * s32() * r],
        Orient::YRows => [p * s32() * q, p * (r + q / 2.0)],
    }
}

/// The element whose hexagon holds `u` (relative to the centre of `(0, 0)`), by cube rounding.
pub(crate) fn round_axial(o: Orient, p: f64, u: [f64; 2]) -> (i32, i32) {
    let (qf, rf) = match o {
        Orient::XRows => {
            let rf = u[1] / (p * s32());
            (u[0] / p - rf / 2.0, rf)
        }
        Orient::YRows => {
            let qf = u[0] / (p * s32());
            (qf, u[1] / p - qf / 2.0)
        }
    };
    let (x, z) = (qf, rf);
    let y = -x - z;
    let (mut rx, ry, mut rz) = (x.round(), y.round(), z.round());
    let (dx, dy, dz) = ((rx - x).abs(), (ry - y).abs(), (rz - z).abs());
    if dx > dy && dx > dz {
        rx = -ry - rz;
    } else if dy <= dz {
        rz = -rx - ry;
    }
    // `as` saturates; the caller range-checks.
    (rx as i32, rz as i32)
}

/// Signed distance of `u` (relative to an element centre) to the hexagon boundary: negative
/// inside, `0` on a face, positive outside (the maximum over the three flat pairs).
pub(crate) fn sd(o: Orient, p: f64, u: [f64; 2]) -> f64 {
    let h = s32();
    let n: [[f64; 2]; 3] = match o {
        Orient::XRows => [[1.0, 0.0], [0.5, h], [-0.5, h]],
        Orient::YRows => [[h, 0.5], [0.0, 1.0], [-h, 0.5]],
    };
    n.iter().map(|n| (u[0] * n[0] + u[1] * n[1]).abs()).fold(0.0, f64::max) - 0.5 * p
}

/// Half extents `(x, y)` of the box circumscribing one hexagon.
pub(crate) fn half_box(o: Orient, p: f64) -> [f64; 2] {
    let v = p / 3.0_f64.sqrt();
    match o {
        Orient::XRows => [0.5 * p, v],
        Orient::YRows => [v, 0.5 * p],
    }
}

/// The `(q, r)` offsets from the origin element of the candidates for the point `u` (relative to
/// the centre of `(0, 0)`), in the order they are tried.
///
/// Candidates are the elements whose hexagon holds `u` within `tol`: one strictly inside, two
/// within `tol` of a face, three within `tol` of a vertex. The vertex-tie rule: candidates are
/// ordered by the distance of `u + 4 tol dir` to their centres, so the element the direction
/// enters comes first; elements within `1e-12 p^2` of each other (a direction along a shared
/// face) are ordered by descending `(r, q)`, which is descending linear index. With a zero `dir`
/// the order is by the distance of `u` itself, then the same tie rule: deterministic.
pub(crate) fn candidates(o: Orient, p: f64, u: [f64; 2], dir: [f64; 2], tol: f64) -> SmallVec<[(i32, i32); 3]> {
    let (q0, r0) = round_axial(o, p, u);
    let mut out: SmallVec<[(i32, i32); 3]> = SmallVec::new();
    let c0 = center(o, p, q0, r0);
    let s0 = sd(o, p, [u[0] - c0[0], u[1] - c0[1]]);
    out.push((q0, r0));
    if s0 < -tol {
        return out;
    }
    for (dq, dr) in NEIGHBOURS {
        let (q, r) = (q0.saturating_add(dq), r0.saturating_add(dr));
        let c = center(o, p, q, r);
        if sd(o, p, [u[0] - c[0], u[1] - c[1]]) <= tol {
            out.push((q, r));
        }
    }
    if out.len() > 1 {
        let sh = [u[0] + 4.0 * tol * dir[0], u[1] + 4.0 * tol * dir[1]];
        let key = |&(q, r): &(i32, i32)| {
            let c = center(o, p, q, r);
            (sh[0] - c[0]).powi(2) + (sh[1] - c[1]).powi(2)
        };
        let eps = 1e-12 * p * p;
        out.sort_by(|a, b| {
            let (ka, kb) = (key(a), key(b));
            if (ka - kb).abs() <= eps { (b.1, b.0).cmp(&(a.1, a.0)) } else { ka.total_cmp(&kb) }
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORIENTS: [Orient; 2] = [Orient::XRows, Orient::YRows];

    #[test]
    fn rounding_inverts_the_centres() {
        for o in ORIENTS {
            for q in -6..=6 {
                for r in -6..=6 {
                    let c = center(o, 1.7, q, r);
                    assert_eq!(round_axial(o, 1.7, c), (q, r), "{o:?} ({q}, {r})");
                    let u = [c[0] + 0.3, c[1] - 0.2];
                    assert_eq!(round_axial(o, 1.7, u), (q, r), "{o:?} off-centre");
                }
            }
        }
    }

    #[test]
    fn the_hexagon_has_apothem_half_pitch_and_circumradius_pitch_over_sqrt3() {
        let p = 2.0;
        for o in ORIENTS {
            // Neighbour centres sit on the flats: distance p, and the midpoint is on the face.
            for (dq, dr) in NEIGHBOURS {
                let c = center(o, p, dq, dr);
                assert!((c[0].hypot(c[1]) - p).abs() < 1e-14);
                assert!(sd(o, p, [0.5 * c[0], 0.5 * c[1]]).abs() < 1e-14, "{o:?}");
            }
            // Vertices at radius p/sqrt(3) lie on the boundary and on the circumscribing box.
            let hb = half_box(o, p);
            let long = hb[0].max(hb[1]);
            assert!((long - p / 3.0_f64.sqrt()).abs() < 1e-14);
            let v = match o {
                Orient::XRows => [0.0, long],
                Orient::YRows => [long, 0.0],
            };
            assert!(sd(o, p, v).abs() < 1e-14, "{o:?} vertex");
            assert!(sd(o, p, [0.0, 0.0]) < -0.99 * p / 2.0);
        }
    }

    #[test]
    fn a_vertex_has_three_candidates_and_an_edge_two() {
        let (p, tol) = (1.0, 1e-9);
        for o in ORIENTS {
            // The vertex shared by (0,0), (1,0), (0,1) in both orientations is the circumcentre of
            // the three centres.
            let (a, b, c) = (center(o, p, 0, 0), center(o, p, 1, 0), center(o, p, 0, 1));
            let v = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0];
            let all = candidates(o, p, v, [0.0, 0.0], tol);
            let mut s: Vec<_> = all.iter().copied().collect();
            s.sort_unstable();
            assert_eq!(s, vec![(0, 0), (0, 1), (1, 0)], "{o:?}");
            // The direction decides: heading toward a centre enters that element first.
            for (e, ce) in [((0, 0), a), ((1, 0), b), ((0, 1), c)] {
                let d = [ce[0] - v[0], ce[1] - v[1]];
                let n = d[0].hypot(d[1]);
                let first = candidates(o, p, v, [d[0] / n, d[1] / n], tol)[0];
                assert_eq!(first, e, "{o:?}");
            }
            // An edge midpoint: two candidates, the one entered first leads.
            let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
            let two = candidates(o, p, m, [b[0] - a[0], b[1] - a[1]], tol);
            assert_eq!(two.len(), 2);
            assert_eq!(two[0], (1, 0));
            let back = candidates(o, p, m, [a[0] - b[0], a[1] - b[1]], tol);
            assert_eq!(back[0], (0, 0));
            // A direction along the shared face: a tie, resolved to the higher `(r, q)`.
            let along = [-(b[1] - a[1]), b[0] - a[0]];
            let tie = candidates(o, p, m, along, tol);
            assert_eq!(tie[0], (1, 0));
            // Strictly inside: one candidate.
            assert_eq!(candidates(o, p, [0.1, 0.05], [1.0, 0.0], tol).len(), 1);
        }
    }
}
