use super::*;
use sm_math::{Sign, orient2d};
use sm_testkit::loops2d::{self, Region2};

fn id_metric(_: [f64; 2]) -> [f64; 3] {
    [1.0, 0.0, 1.0]
}

/// Subdivide every segment of the loop into `ceil(len / h)` equal pieces.
fn resample(l: &[(f64, f64)], h: f64) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    for i in 0..l.len() {
        let (a, b) = (l[i], l[(i + 1) % l.len()]);
        let len = (b.0 - a.0).hypot(b.1 - a.1);
        let k = ((len / h).ceil() as usize).max(1);
        for s in 0..k {
            let t = s as f64 / k as f64;
            out.push([a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)]);
        }
    }
    out
}

#[test]
fn preflight_errors() {
    let o = PaveOptions::default();
    let sq = |n: usize| -> Vec<[f64; 2]> { resample(&loops2d::square().outer, 1.0 / n as f64) };
    assert_eq!(pave_region(&[vec![[0.0, 0.0], [1.0, 0.0]]], &id_metric, None, &o).err(), Some(PaveError::LoopTooSmall { ring: 0, nodes: 2 }));
    let mut odd = sq(2);
    odd.insert(1, [0.25, 0.0]);
    assert_eq!(pave_region(&[odd], &id_metric, None, &o).err(), Some(PaveError::OddBoundary { total: 9 }));
    let bow = vec![[0.0, 0.0], [2.0, 2.0], [2.0, 0.0], [0.0, 2.0]];
    assert_eq!(pave_region(&[bow], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
}

#[test]
fn square_with_eight_boundary_nodes_is_four_quads() {
    let l = vec![resample(&loops2d::square().outer, 0.5)];
    assert_eq!(l[0].len(), 8);
    let m = pave_region(&l, &id_metric, None, &PaveOptions::default()).expect("pave");
    assert_eq!(m.points.len(), 9);
    assert_eq!(m.quads.len(), 4);
}

fn loops_at(r: &Region2, h: f64) -> Vec<Vec<[f64; 2]>> {
    let mut ls: Vec<Vec<[f64; 2]>> = r.loops().map(|l| resample(l, h)).collect();
    let total: usize = ls.iter().map(Vec::len).sum();
    if total % 2 == 1 {
        let (a, b) = (ls[0][0], ls[0][1]);
        ls[0].insert(1, [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
    }
    ls
}

/// Corner-sine scaled Jacobian of a quad (minimum over its corners), sign-corrected for the
/// winding `sgn` (+1 counter-clockwise, -1 clockwise).
fn sj(p: &[[f64; 2]], q: &[u32; 4], sgn: f64) -> f64 {
    let mut m = f64::INFINITY;
    for k in 0..4 {
        let (a, b, c) = (p[q[(k + 3) % 4] as usize], p[q[k] as usize], p[q[(k + 1) % 4] as usize]);
        let (e1, e2) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
        m = m.min(sgn * (e1[0] * e2[1] - e1[1] * e2[0]) / (e1[0].hypot(e1[1]) * e2[0].hypot(e2[1])));
    }
    m
}

struct Stats {
    min: f64,
    mean: f64,
}

/// The acceptance-1 checks; returns the quality statistics.
fn check(region: &Region2, loops: &[Vec<[f64; 2]>], m: &QuadMesh2) -> Stats {
    let nb: usize = loops.iter().map(Vec::len).sum();
    // Boundary points bit-exact, in loop-concatenated order.
    let flat: Vec<[f64; 2]> = loops.iter().flatten().copied().collect();
    for (i, p) in flat.iter().enumerate() {
        assert_eq!(m.points[i].map(f64::to_bits), p.map(f64::to_bits), "boundary point {i}");
    }
    // Directed edge counts.
    let mut dir = std::collections::BTreeMap::<(u32, u32), u32>::new();
    for q in &m.quads {
        for k in 0..4 {
            *dir.entry((q[k], q[(k + 1) % 4])).or_default() += 1;
        }
    }
    assert!(dir.values().all(|c| *c == 1), "an edge is used twice in one direction");
    let mut at = 0;
    let mut seg = std::collections::BTreeSet::new();
    for l in loops {
        for k in 0..l.len() {
            seg.insert((at + k, at + (k + 1) % l.len()));
        }
        at += l.len();
    }
    for (a, b) in &seg {
        assert!(dir.contains_key(&(*a as u32, *b as u32)), "segment {a}->{b} has no owner in the right direction");
        assert!(!dir.contains_key(&(*b as u32, *a as u32)), "segment {a}->{b} is owned from both sides");
    }
    let mut edges = std::collections::BTreeSet::new();
    for (a, b) in dir.keys() {
        let e = (*a.min(b), *a.max(b));
        let n = dir.contains_key(&(*a, *b)) as u32 + dir.contains_key(&(*b, *a)) as u32;
        assert!(n == 2 || seg.contains(&(*a as usize, *b as usize)) || seg.contains(&(*b as usize, *a as usize)), "edge {e:?} is used once but is not a boundary segment");
        edges.insert(e);
    }
    let v = m.points.len() as i64;
    assert_eq!(v - edges.len() as i64 + m.quads.len() as i64, 1 - (loops.len() as i64 - 1), "Euler");
    // Winding like the input outer loop, strictly convex corners.
    let ccw = loops2d::signed_area(&region.outer) > 0.0;
    let sgn = if ccw { 1.0 } else { -1.0 };
    let mut area = 0.0;
    let (mut min, mut sum) = (f64::INFINITY, 0.0);
    for q in &m.quads {
        for k in 0..4 {
            let s = orient2d(m.points[q[(k + 3) % 4] as usize], m.points[q[k] as usize], m.points[q[(k + 1) % 4] as usize]);
            assert_eq!(s, if ccw { Sign::Positive } else { Sign::Negative }, "quad {q:?} corner {k}");
        }
        let p: [[f64; 2]; 4] = core::array::from_fn(|k| m.points[q[k] as usize]);
        area += 0.5 * ((p[2][0] - p[0][0]) * (p[3][1] - p[1][1]) - (p[2][1] - p[0][1]) * (p[3][0] - p[1][0])).abs();
        let s = sj(&m.points, q, sgn);
        min = min.min(s);
        sum += s;
    }
    assert!((area - region.area).abs() <= 1e-12 * region.area, "area {area} vs {}", region.area);
    for p in &m.points[nb..] {
        assert!(region.contains((p[0], p[1])), "interior point {p:?} outside the region");
    }
    Stats { min, mean: sum / m.quads.len() as f64 }
}

#[test]
fn survey_acceptance_one_and_two() {
    let fixtures = [loops2d::square(), loops2d::rectangle(3.0, 2.0), loops2d::circle(1.5, 32), loops2d::l_shape()];
    for r in &fixtures {
        for h in [0.125, 0.25, 0.5] {
            let ls = loops_at(r, h);
            let m = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
            let s = check(r, &ls, &m);
            eprintln!("{:>10} h={h:<6} nb={:<4} quads={:<5} rounds={} min={:.3} mean={:.3}", r.name, ls.iter().map(Vec::len).sum::<usize>(), m.quads.len(), m.rounds, s.min, s.mean);
        }
    }
}

#[test]
#[ignore]
fn dump_circle() {
    let ls = loops_at(&loops2d::circle(1.5, 32), 0.125);
    let _ = pave_region(&ls, &id_metric, None, &PaveOptions::default());
}
