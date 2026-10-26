#![allow(clippy::single_range_in_vec_init)]
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

const SPACINGS: [f64; 3] = [0.125, 0.25, 0.5];

#[test]
fn acceptance_1_conformity_ownership_area_euler_on_the_fixtures() {
    for r in [loops2d::square(), loops2d::rectangle(3.0, 2.0), loops2d::circle(1.5, 32), loops2d::l_shape()] {
        for h in SPACINGS {
            let ls = loops_at(&r, h);
            let m = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
            check(&r, &ls, &m);
        }
    }
}

#[test]
fn acceptance_2_quality_floors() {
    for (r, min_floor) in [(loops2d::square(), 0.40), (loops2d::rectangle(3.0, 2.0), 0.40), (loops2d::circle(1.5, 32), 0.40), (loops2d::l_shape(), 0.35)] {
        for h in SPACINGS {
            let ls = loops_at(&r, h);
            let m = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
            let s = check(&r, &ls, &m);
            eprintln!("{:>10} h={h:<6} nb={:<4} quads={:<5} rounds={} min={:.3} mean={:.3}", r.name, ls.iter().map(Vec::len).sum::<usize>(), m.quads.len(), m.rounds, s.min, s.mean);
            assert!(s.min >= min_floor, "{} h={h}: min {:.3}", r.name, s.min);
            assert!(s.mean >= 0.85, "{} h={h}: mean {:.3}", r.name, s.mean);
        }
    }
}

#[test]
fn acceptance_3_even_rule_for_every_mandatory_fixture() {
    let o = PaveOptions::default();
    for r in [loops2d::square(), loops2d::rectangle(3.0, 2.0), loops2d::circle(1.5, 32), loops2d::l_shape()] {
        for h in SPACINGS {
            let mut ls = loops_at(&r, h);
            // One more node: the total turns odd.
            let (a, b) = (ls[0][0], ls[0][1]);
            ls[0].insert(1, [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
            let total: usize = ls.iter().map(Vec::len).sum();
            assert_eq!(pave_region(&ls, &id_metric, None, &o).err(), Some(PaveError::OddBoundary { total }), "{} h={h}", r.name);
        }
    }
    assert_eq!(pave_region(&[vec![[0.0, 0.0], [1.0, 0.0]]], &id_metric, None, &o).err(), Some(PaveError::LoopTooSmall { ring: 0, nodes: 2 }));
    let square = resample(&loops2d::square().outer, 0.5);
    assert_eq!(pave_region(&[square, vec![[0.2, 0.2], [0.4, 0.2]]], &id_metric, None, &o).err(), Some(PaveError::LoopTooSmall { ring: 1, nodes: 2 }));
    // A hand-built self-crossing loop (bow tie), a loop that touches itself, and a hole that
    // pokes through the outer loop.
    let bow = vec![[0.0, 0.0], [2.0, 2.0], [2.0, 0.0], [0.0, 2.0]];
    assert_eq!(pave_region(&[bow], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
    let touching = vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [1.0, 0.0], [0.0, 2.0], [0.0, 1.0]];
    assert_eq!(pave_region(&[touching], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
    let outer = resample(&loops2d::square().outer, 0.25);
    let poke: Vec<[f64; 2]> = vec![[0.5, 0.5], [0.5, 1.5], [0.7, 1.5], [0.7, 0.5]];
    assert_eq!(pave_region(&[outer.clone(), poke], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
    // Duplicate nodes and non-finite coordinates.
    let mut dup = outer.clone();
    dup[3] = dup[2];
    assert_eq!(pave_region(&[dup], &id_metric, None, &o).err(), Some(PaveError::BoundaryIntersects));
    let mut nan = outer.clone();
    nan[0][0] = f64::NAN;
    assert_eq!(pave_region(&[nan], &id_metric, None, &o).err(), Some(PaveError::NonFinite));
    // A hole wound like the outer loop.
    let hole: Vec<[f64; 2]> = vec![[0.4, 0.4], [0.6, 0.4], [0.6, 0.6], [0.4, 0.6]];
    assert_eq!(pave_region(&[outer, hole], &id_metric, None, &o).err(), Some(PaveError::BadOrientation { ring: 1 }));
    // Bad metrics.
    let bad = |_: [f64; 2]| [1.0, 2.0, 1.0];
    assert_eq!(pave_region(&[resample(&loops2d::square().outer, 0.5)], &bad, None, &o).err(), Some(PaveError::BadMetric));
}

#[test]
fn acceptance_4a_constant_metric_meshes_like_a_2_by_1_rectangle() {
    // [0,1]^2 with E = 4 is a 2 x 1 rectangle: segments of 1/16 in u are 0.125 long, 1/8 in v.
    let (nu, nv) = (16usize, 8usize);
    let mut l: Vec<[f64; 2]> = Vec::new();
    for i in 0..nu {
        l.push([i as f64 / nu as f64, 0.0]);
    }
    for j in 0..nv {
        l.push([1.0, j as f64 / nv as f64]);
    }
    for i in 0..nu {
        l.push([1.0 - i as f64 / nu as f64, 1.0]);
    }
    for j in 0..nv {
        l.push([0.0, 1.0 - j as f64 / nv as f64]);
    }
    let metric = |_: [f64; 2]| [4.0, 0.0, 1.0];
    let m = pave_region(&[l.clone()], &metric, None, &PaveOptions::default()).expect("pave");
    let h = 0.125;
    // The same mesh, measured in the metric: scale u by 2.
    let q: Vec<[f64; 2]> = m.points.iter().map(|p| [2.0 * p[0], p[1]]).collect();
    let mut min_sj = f64::INFINITY;
    for quad in &m.quads {
        for k in 0..4 {
            let (a, b) = (q[quad[k] as usize], q[quad[(k + 1) % 4] as usize]);
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            assert!((0.5 * h..=2.0 * h).contains(&len), "edge length {len} outside [{}, {}]", 0.5 * h, 2.0 * h);
        }
        min_sj = min_sj.min(sj(&q, quad, 1.0));
    }
    assert!(min_sj >= 0.40, "min scaled Jacobian {min_sj}");
    assert_eq!(m.quads.len(), 16 * 8, "the lattice is regular");
    let region = Region2 { name: "unit", outer: loops2d::square().outer, holes: vec![], area: 1.0, outer_types: None };
    check(&region, &[l], &m);
}

#[test]
fn kernel_is_deterministic_and_reflection_invariant() {
    let ls = loops_at(&loops2d::circle(1.5, 32), 0.125);
    let a = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
    let b = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
    assert_eq!(a.quads, b.quads);
    assert!(a.points.iter().zip(&b.points).all(|(p, q)| p.map(f64::to_bits) == q.map(f64::to_bits)));
    // The mirror image (clockwise outer loop) meshes to the mirrored mesh with the same
    // connectivity: quads keep the winding of the input loop.
    let cw: Vec<Vec<[f64; 2]>> = ls.iter().map(|l| l.iter().rev().copied().collect()).collect();
    let c = pave_region(&cw, &id_metric, None, &PaveOptions::default()).expect("pave cw");
    let region = loops2d::circle(1.5, 32);
    let mut flipped = region.clone();
    flipped.outer.reverse();
    let s = check(&flipped, &cw, &c);
    assert!(s.min >= 0.40);
}

#[test]
fn acceptance_11_performance_100_by_100() {
    // 400 boundary points, about 10^4 quads; the gate only binds in release builds.
    let n = 100;
    let mut l: Vec<[f64; 2]> = Vec::new();
    for i in 0..n {
        l.push([i as f64, 0.0]);
    }
    for i in 0..n {
        l.push([n as f64, i as f64]);
    }
    for i in 0..n {
        l.push([(n - i) as f64, n as f64]);
    }
    for i in 0..n {
        l.push([0.0, (n - i) as f64]);
    }
    let t0 = std::time::Instant::now();
    let m = pave_region(&[l], &id_metric, None, &PaveOptions::default()).expect("pave");
    let dt = t0.elapsed().as_secs_f64();
    eprintln!("100 x 100 square: {} quads in {dt:.3} s", m.quads.len());
    assert_eq!(m.quads.len(), n * n);
    if !cfg!(debug_assertions) {
        assert!(dt < 2.0, "{dt} s");
    }
}

// ---------------------------------------------------------------------------- surveys
// Ignored by default: they print robustness statistics (see W3_REPORT.md).

/// Deterministic xorshift for the surveys.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Rectangle `w x h` with `a, b, c, d` intervals on bottom, right, top, left.
fn counted_rect(w: f64, h: f64, c: [usize; 4]) -> Vec<[f64; 2]> {
    let mut l = Vec::new();
    for i in 0..c[0] {
        l.push([w * i as f64 / c[0] as f64, 0.0]);
    }
    for j in 0..c[1] {
        l.push([w, h * j as f64 / c[1] as f64]);
    }
    for i in 0..c[2] {
        l.push([w - w * i as f64 / c[2] as f64, h]);
    }
    for j in 0..c[3] {
        l.push([0.0, h - h * j as f64 / c[3] as f64]);
    }
    l
}

/// Polygon through `corners` with `counts[i]` equal intervals on side `i`.
fn counted_polygon(corners: &[[f64; 2]], counts: &[usize]) -> Vec<[f64; 2]> {
    let n = corners.len();
    let mut out = Vec::new();
    for i in 0..n {
        let (a, b) = (corners[i], corners[(i + 1) % n]);
        for s in 0..counts[i] {
            let t = s as f64 / counts[i] as f64;
            out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
        }
    }
    out
}

/// Polygon through `corners` with `round(len / h)` intervals per side (at least 1); one side
/// gets an extra interval if the total is odd.
fn rounded_polygon(corners: &[[f64; 2]], h: f64, rng: &mut Rng) -> Vec<[f64; 2]> {
    let n = corners.len();
    let mut counts: Vec<usize> = (0..n).map(|i| {
        let (a, b) = (corners[i], corners[(i + 1) % n]);
        (((b[0] - a[0]).hypot(b[1] - a[1]) / h).round() as usize).max(1)
    }).collect();
    if counts.iter().sum::<usize>() % 2 == 1 {
        let k = (rng.next() % n as u64) as usize;
        counts[k] += 1;
    }
    counted_polygon(corners, &counts)
}

/// Rectangles with unequal counts (ratio at most 2), named by their counts.
fn corpus_small_rectangles() -> Vec<(String, Vec<[f64; 2]>)> {
    let mut v = Vec::new();
    for a in 1..=8usize {
        for b in 1..=8usize {
            for c in 1..=8usize {
                for d in 1..=8usize {
                    let (lo, hi) = ([a, b, c, d].into_iter().min().unwrap_or(1), [a, b, c, d].into_iter().max().unwrap_or(1));
                    if (a + b + c + d) % 2 == 1 || a + b + c + d < 4 || hi > 2 * lo {
                        continue;
                    }
                    v.push((format!("rect {a},{b},{c},{d}"), counted_rect(1.0, 1.0, [a, b, c, d])));
                }
            }
        }
    }
    v
}

/// Random rectangles, L, U and triangle polygons with rounded uniform spacing.
fn corpus_polygons(shapes: core::ops::Range<u64>) -> Vec<(String, Vec<[f64; 2]>)> {
    let mut rng = Rng(0x1234_5678_9abc_def1);
    let mut v = Vec::new();
    for case in 0..600u64 {
        let (w, hh) = (1.0 + 5.0 * rng.unit(), 1.0 + 5.0 * rng.unit());
        let h = 0.15 + 0.5 * rng.unit();
        let (name, corners): (&str, Vec<[f64; 2]>) = match case % 4 {
            0 => ("rectangle", vec![[0.0, 0.0], [w, 0.0], [w, hh], [0.0, hh]]),
            1 => ("L", vec![[0.0, 0.0], [w, 0.0], [w, 0.5 * hh], [0.5 * w, 0.5 * hh], [0.5 * w, hh], [0.0, hh]]),
            2 => ("U", vec![[0.0, 0.0], [w, 0.0], [w, hh], [0.7 * w, hh], [0.7 * w, 0.4 * hh], [0.3 * w, 0.4 * hh], [0.3 * w, hh], [0.0, hh]]),
            _ => ("triangle", vec![[0.0, 0.0], [w, 0.0], [0.4 * w, hh]]),
        };
        let l = rounded_polygon(&corners, h, &mut rng);
        if l.len() >= 4 && shapes.contains(&(case % 4)) {
            v.push((format!("{name} case {case} (h={h:.2})"), l));
        }
    }
    v
}

/// Circles, ellipses and five-lobed stars with 24 to 160 nodes.
fn corpus_curves() -> Vec<(String, Vec<[f64; 2]>)> {
    let tau = core::f64::consts::TAU;
    let mut v = Vec::new();
    for n in (24..=160).step_by(2) {
        let r = 1.0 + 0.01 * n as f64;
        v.push((format!("circle n={n}"), (0..n).map(|i| [r * (tau * i as f64 / n as f64).cos(), r * (tau * i as f64 / n as f64).sin()]).collect()));
    }
    for k in 0..60 {
        let n = 40 + 2 * k;
        v.push((format!("ellipse n={n}"), (0..n).map(|i| [3.0 * (tau * i as f64 / n as f64).cos(), (tau * i as f64 / n as f64).sin()]).collect()));
    }
    for k in 0..40 {
        let n = 60 + 4 * k;
        v.push((format!("star n={n}"), (0..n).map(|i| { let a = tau * i as f64 / n as f64; let r = 1.0 + 0.35 * (5.0 * a).cos(); [r * a.cos(), r * a.sin()] }).collect()));
    }
    v
}

/// Isosceles triangles (apex angle 20 to 140 degrees, 4 to 16 intervals on the legs).
fn corpus_isoceles() -> Vec<(String, Vec<[f64; 2]>)> {
    let mut v = Vec::new();
    for apex in [20.0f64, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 120.0, 140.0] {
        for n in [4usize, 6, 8, 10, 12, 16] {
            let a = apex.to_radians();
            let corners = [[0.0, 0.0], [1.0, 0.0], [a.cos(), a.sin()]];
            let base = (corners[2][0] - 1.0).hypot(corners[2][1]);
            let mut counts = [n, ((base * n as f64).round() as usize).max(1), n];
            if counts.iter().sum::<usize>() % 2 == 1 {
                counts[1] += 1;
            }
            v.push((format!("apex {apex:.0} n={n}"), counted_polygon(&corners, &counts)));
        }
    }
    v
}

fn min_sj(m: &QuadMesh2) -> f64 {
    m.quads.iter().map(|q| sj(&m.points, q, 1.0)).fold(f64::INFINITY, f64::min)
}

/// Run the production ladder over a corpus; print failures and the worst minimum SJ.
fn survey(title: &str, corpus: &[(String, Vec<[f64; 2]>)]) {
    let o = PaveOptions::default();
    let (mut fails, mut worst, mut below, mut worst_mean) = (0, f64::INFINITY, 0, f64::INFINITY);
    let t0 = std::time::Instant::now();
    for (name, l) in corpus {
        match pave_region(std::slice::from_ref(l), &id_metric, None, &o) {
            Ok(m) => {
                let q = min_sj(&m);
                worst = worst.min(q);
                below += usize::from(q < 0.2);
                worst_mean = worst_mean.min(m.quads.iter().map(|q| sj(&m.points, q, 1.0)).sum::<f64>() / m.quads.len() as f64);
            }
            Err(e) => {
                fails += 1;
                if fails <= 5 {
                    eprintln!("{title}: {name}: {e}");
                }
            }
        }
    }
    eprintln!("{title}: {fails} failures of {}; worst min SJ {worst:.3}, worst mean {worst_mean:.3}, {below} below 0.2; {:.1} s", corpus.len(), t0.elapsed().as_secs_f64());
}

#[test]
#[ignore]
fn robustness_survey() {
    survey("curves", &corpus_curves());
}

#[test]
#[ignore]
fn small_rectangles_with_unequal_counts() {
    survey("small rectangles", &corpus_small_rectangles());
}

#[test]
#[ignore]
fn survey_realistic_polygons() {
    survey("rectangles, L, U", &corpus_polygons(0..3));
    survey("triangles", &corpus_polygons(3..4));
}

#[test]
#[ignore]
fn survey_isoceles_triangles() {
    survey("isoceles triangles", &corpus_isoceles());
    // The per-case table: minimum scaled Jacobian, `--` for a failure.
    let o = PaveOptions::default();
    for (i, apex) in [20.0f64, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 120.0, 140.0].into_iter().enumerate() {
        let row: String = corpus_isoceles()[6 * i..6 * i + 6].iter().map(|(_, l)| match pave_region(std::slice::from_ref(l), &id_metric, None, &o) {
            Ok(m) => format!(" {:>5.2}", min_sj(&m)),
            Err(_) => "    --".to_string(),
        }).collect();
        eprintln!("apex {apex:>5.0}:{row}");
    }
}

/// Which candidate strategies recover the regions that the first strategy fails on, and a
/// greedy cover: the order in which `ladder()` should list them.
#[test]
#[ignore]
fn survey_cover() {
    let o = PaveOptions::default();
    let cands = candidates();
    let mut corpus = corpus_small_rectangles();
    corpus.extend(corpus_polygons(0..4));
    corpus.extend(corpus_curves());
    corpus.extend(corpus_isoceles());
    let mut hard: Vec<(String, Vec<usize>)> = Vec::new();
    for (name, l) in &corpus {
        if pave_with(std::slice::from_ref(l), &id_metric, None, &o, &cands[..1]).is_ok() {
            continue;
        }
        let ok: Vec<usize> = (1..cands.len()).filter(|k| pave_with(std::slice::from_ref(l), &id_metric, None, &o, &cands[*k..*k + 1]).is_ok()).collect();
        hard.push((name.clone(), ok));
    }
    eprintln!("{} of {} cases fail the first strategy; {} of them no candidate recovers", hard.len(), corpus.len(), hard.iter().filter(|h| h.1.is_empty()).count());
    let mut left: Vec<&(String, Vec<usize>)> = hard.iter().filter(|h| !h.1.is_empty()).collect();
    let mut order = vec![0usize];
    while !left.is_empty() {
        let best = (1..cands.len()).filter(|k| !order.contains(k)).max_by_key(|k| (left.iter().filter(|h| h.1.contains(k)).count(), core::cmp::Reverse(*k)));
        let Some(k) = best else { break };
        let n = left.iter().filter(|h| h.1.contains(&k)).count();
        if n == 0 {
            break;
        }
        order.push(k);
        left.retain(|h| !h.1.contains(&k));
        eprintln!("  + candidate {k}: covers {n}, {} left", left.len());
    }
    eprintln!("ladder order: {order:?}");
}

#[test]
fn stretch_s1_regions_with_holes() {
    for r in [loops2d::rectangle_with_holes(), loops2d::annulus(1.0, 2.0, 32)] {
        for h in SPACINGS {
            let ls = loops_at(&r, h);
            let m = pave_region(&ls, &id_metric, None, &PaveOptions::default()).expect("pave");
            let s = check(&r, &ls, &m);
            eprintln!("{} h={h}: {} quads, min {:.3} mean {:.3}", r.name, m.quads.len(), s.min, s.mean);
            assert!(s.min >= 0.40, "{} h={h}: min {:.3}", r.name, s.min);
            assert!(s.mean >= 0.85, "{} h={h}: mean {:.3}", r.name, s.mean);
        }
        // Even rule and the loop count of the hole loops.
        let mut ls = loops_at(&r, 0.25);
        let (a, b) = (ls[1][0], ls[1][1]);
        ls[1].insert(1, [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
        let total: usize = ls.iter().map(Vec::len).sum();
        assert_eq!(pave_region(&ls, &id_metric, None, &PaveOptions::default()).err(), Some(PaveError::OddBoundary { total }));
    }
}

/// Boundary nodes along `a -> b` (without `b`) whose spacing grows geometrically from about
/// `s_a` to about `s_b`.
fn graded_segment(a: [f64; 2], b: [f64; 2], s_a: f64, s_b: f64) -> Vec<[f64; 2]> {
    let len = (b[0] - a[0]).hypot(b[1] - a[1]);
    let n = if (s_a - s_b).abs() < 1e-12 { len / s_a } else { len * (s_b / s_a).ln() / (s_b - s_a) }.round().max(1.0) as usize;
    // x_i = len * (q^i - 1) / (q^n - 1), q solved so that the first spacing is s_a.
    let first = |q: f64| len * (q - 1.0) / (q.powi(n as i32) - 1.0);
    let q = if n == 1 || (s_a - s_b).abs() < 1e-12 {
        1.0
    } else {
        let (mut lo, mut hi) = if s_b > s_a { (1.0 + 1e-9, 4.0) } else { (0.25, 1.0 - 1e-9) };
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            // `first` decreases with q.
            if first(mid) > s_a { lo = mid } else { hi = mid }
        }
        0.5 * (lo + hi)
    };
    (0..n)
        .map(|i| {
            let x = if q == 1.0 { i as f64 / n as f64 } else { (q.powi(i as i32) - 1.0) / (q.powi(n as i32) - 1.0) };
            [a[0] + x * (b[0] - a[0]), a[1] + x * (b[1] - a[1])]
        })
        .collect()
}

#[test]
fn stretch_s2_graded_boundary() {
    // [0, 3] x [0, 1.5]: spacing 0.05 on the left growing to 0.2 on the right.
    let (w, h) = (3.0, 1.5);
    let mut l: Vec<[f64; 2]> = Vec::new();
    l.extend(graded_segment([0.0, 0.0], [w, 0.0], 0.05, 0.2));
    l.extend(graded_segment([w, 0.0], [w, h], 0.2, 0.2));
    l.extend(graded_segment([w, h], [0.0, h], 0.2, 0.05));
    l.extend(graded_segment([0.0, h], [0.0, 0.0], 0.05, 0.05));
    if l.len() % 2 == 1 {
        // One more node on the short right edge.
        let k = l.iter().position(|p| p[0] == w && p[1] == 0.0).unwrap_or(0);
        let (a, b) = (l[k], l[k + 1]);
        l.insert(k + 1, [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
    }
    let m = pave_region(&[l.clone()], &id_metric, None, &PaveOptions::default()).expect("pave");
    let region = Region2 { name: "graded", outer: vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)], holes: vec![], area: w * h, outer_types: None };
    let s = check(&region, &[l], &m);
    let area = |q: &[u32; 4]| {
        let p: [[f64; 2]; 4] = core::array::from_fn(|k| m.points[q[k] as usize]);
        0.5 * ((p[2][0] - p[0][0]) * (p[3][1] - p[1][1]) - (p[2][1] - p[0][1]) * (p[3][0] - p[1][0])).abs()
    };
    let centroid_x = |q: &[u32; 4]| q.iter().map(|v| m.points[*v as usize][0]).sum::<f64>() / 4.0;
    let (mut left, mut right) = ((0.0, 0usize), (0.0, 0usize));
    for q in &m.quads {
        let (cx, a) = (centroid_x(q), area(q));
        if cx < 1.0 {
            left = (left.0 + a, left.1 + 1);
        } else if cx > 2.0 {
            right = (right.0 + a, right.1 + 1);
        }
    }
    let ratio = (left.0 / left.1 as f64) / (right.0 / right.1 as f64);
    eprintln!("graded: {} quads, min {:.3} mean {:.3}, mean area ratio left/right {ratio:.4}", m.quads.len(), s.min, s.mean);
    assert!(ratio <= 1.0 / 3.0, "ratio {ratio}");
}


/// S3 (reported, not asserted): quality on a spatially varying metric. The metric
/// `[(1 + u)^2, 0, 1]` is the pull-back of the map `x = u + u^2 / 2, y = v`, so the physical
/// mesh is the image of the quads; quality is measured there.
#[test]
fn stretch_s3_varying_metric_quality_is_reported() {
    let metric = |uv: [f64; 2]| [(1.0 + uv[0]) * (1.0 + uv[0]), 0.0, 1.0];
    for (label, nu, nv) in [("coarse", 12usize, 6usize), ("fine", 32, 16)] {
        let l = counted_rect(2.0, 1.0, [nu, nv, nu, nv]);
        let m = pave_region(std::slice::from_ref(&l), &metric, None, &PaveOptions::default()).expect("pave");
        let phys: Vec<[f64; 2]> = m.points.iter().map(|p| [p[0] + 0.5 * p[0] * p[0], p[1]]).collect();
        let sjs: Vec<f64> = m.quads.iter().map(|q| sj(&phys, q, 1.0)).collect();
        let (min, mean) = (sjs.iter().copied().fold(f64::INFINITY, f64::min), sjs.iter().sum::<f64>() / sjs.len() as f64);
        eprintln!("S3 {label}: {} quads, physical scaled Jacobian min {min:.3} mean {mean:.3}", m.quads.len());
        assert!(min > 0.0, "inverted quad in the physical space");
    }
}
