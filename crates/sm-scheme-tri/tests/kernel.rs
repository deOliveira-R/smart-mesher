//! Acceptance 1 and the exactness of the pure kernel: `Cdt::new`.

mod common;

use common::*;
use sm_testkit::loops2d;
use sm_scheme_tri::cdt::{Cdt, Pslg, Segment, SegmentKind};

fn lattice(n: usize, scale: f64) -> Pslg {
    let mut p = Pslg::default();
    for j in 0..n {
        for i in 0..n {
            p.points.push([1.0 + scale * i as f64, 1.0 + scale * j as f64]);
        }
    }
    let id = |i: usize, j: usize| (j * n + i) as u32;
    let b = SegmentKind::Boundary;
    for i in 0..n - 1 {
        p.segments.push(Segment { a: id(i, 0), b: id(i + 1, 0), kind: b });
        p.segments.push(Segment { a: id(n - 1, i), b: id(n - 1, i + 1), kind: b });
        p.segments.push(Segment { a: id(i + 1, n - 1), b: id(i, n - 1), kind: b });
        p.segments.push(Segment { a: id(0, i + 1), b: id(0, i), kind: b });
    }
    p
}

#[test]
fn cocircular_lattice_is_exact_and_order_independent() {
    let p = lattice(11, 1.0);
    let c = Cdt::new(&p).expect("cdt");
    assert!(all_positive(&c));
    assert_eq!(c.triangles().len(), 2 * 100);
    assert!((area(&c) - 100.0).abs() < 1e-12);
    assert_eq!(delaunay_violations(&c), 0);
    let base = tri_set(&c);
    for seed in [1u64, 2, 3] {
        let c2 = Cdt::new(&shuffled(&p, seed)).expect("cdt shuffled");
        assert!(all_positive(&c2));
        assert_eq!(tri_set(&c2), base, "shuffle {seed}");
    }
}

#[test]
fn square_with_a_hundred_collinear_boundary_points() {
    let n = 100usize;
    let mut p = Pslg::default();
    let mut ring: Vec<[f64; 2]> = Vec::new();
    for k in 0..n {
        ring.push([k as f64 / n as f64, 0.0]);
    }
    for k in 0..n {
        ring.push([1.0, k as f64 / n as f64]);
    }
    for k in 0..n {
        ring.push([1.0 - k as f64 / n as f64, 1.0]);
    }
    for k in 0..n {
        ring.push([0.0, 1.0 - k as f64 / n as f64]);
    }
    let m = ring.len() as u32;
    p.points = ring;
    for i in 0..m {
        p.segments.push(Segment { a: i, b: (i + 1) % m, kind: SegmentKind::Boundary });
    }
    let c = Cdt::new(&p).expect("cdt");
    assert!(all_positive(&c));
    assert_eq!(c.triangles().len(), 4 * n - 2);
    assert!((area(&c) - 1.0).abs() < 1e-14);
    let base = tri_set(&c);
    for seed in [7u64, 8, 9] {
        let c2 = Cdt::new(&shuffled(&p, seed)).expect("cdt");
        assert_eq!(tri_set(&c2), base);
    }
}

#[test]
fn scaled_by_two_to_the_sixty_both_ways() {
    for e in [60i32, -60] {
        let s = 2f64.powi(e);
        let r = loops2d::rectangle_with_holes();
        let mut p = pslg_of(&r);
        for q in &mut p.points {
            q[0] *= s;
            q[1] *= s;
        }
        let c = Cdt::new(&p).expect("cdt");
        assert!(all_positive(&c), "scale 2^{e}");
        let want = r.area * s * s;
        assert!((area(&c) - want).abs() <= 1e-12 * want, "scale 2^{e}");
        assert_eq!(delaunay_violations(&c), 0);
        // Same triangulation as the unscaled one (powers of two do not change any predicate).
        let c1 = Cdt::new(&pslg_of(&r)).expect("cdt");
        let t1: Vec<_> = c1.triangles();
        let t2: Vec<_> = c.triangles();
        assert_eq!(t1, t2, "scale 2^{e}");
    }
}

#[test]
fn regions_triangulate_with_holes_and_conform() {
    for r in loops2d::all_regions() {
        let p = pslg_of(&r);
        let c = Cdt::new(&p).expect(r.name);
        assert!(all_positive(&c), "{}", r.name);
        assert!((area(&c) - r.area).abs() <= 1e-12 * r.area, "{}: {} vs {}", r.name, area(&c), r.area);
        assert_eq!(delaunay_violations(&c), 0, "{}", r.name);
        // Boundary edges of the output equal the input segments.
        let mut want: Vec<(u32, u32)> =
            p.segments.iter().map(|s| (s.a.min(s.b), s.a.max(s.b))).collect();
        want.sort();
        assert_eq!(boundary_edges(&c), want, "{}", r.name);
        let (v, f) = (c.points().len() as i64, c.triangles().len() as i64);
        let e = (3 * f + want.len() as i64) / 2;
        assert_eq!(v - e + f, 1 - r.holes.len() as i64, "{}", r.name);
    }
}

#[test]
fn embedded_chain_and_isolated_vertex_are_conformed() {
    let mut p = pslg_of(&loops2d::square());
    // Horizontal chain across the square at y = 0.5 through 5 interior points.
    let base = p.points.len() as u32;
    for k in 0..=6 {
        p.points.push([k as f64 / 6.0, 0.5]);
    }
    for k in 0..6 {
        p.segments.push(Segment { a: base + k, b: base + k + 1, kind: SegmentKind::Embedded });
    }
    let iso = p.points.len() as u32;
    p.points.push([0.3, 0.2]);
    let c = Cdt::new(&p).expect("cdt");
    assert!(all_positive(&c));
    for k in 0..6 {
        assert!(c.is_constrained(base + k, base + k + 1));
    }
    let used: std::collections::BTreeSet<u32> = c.triangles().iter().flatten().copied().collect();
    assert!(used.contains(&iso));
    for k in 0..=6 {
        assert!(used.contains(&(base + k)));
    }
    // The embedded segments are mesh edges.
    let tris = c.triangles();
    for k in 0..6u32 {
        let (a, b) = (base + k, base + k + 1);
        assert!(tris.iter().any(|t| (0..3).any(|i| t[i] == a && t[(i + 1) % 3] == b) || (0..3).any(|i| t[i] == b && t[(i + 1) % 3] == a)));
    }
    assert!((area(&c) - 1.0).abs() < 1e-14);
}

#[test]
fn crossing_segments_are_reported() {
    let mut p = pslg_of(&loops2d::square());
    let base = p.points.len() as u32;
    p.points.push([0.2, 0.2]);
    p.points.push([0.8, 0.8]);
    p.points.push([0.2, 0.8]);
    p.points.push([0.8, 0.2]);
    p.segments.push(Segment { a: base, b: base + 1, kind: SegmentKind::Embedded });
    p.segments.push(Segment { a: base + 2, b: base + 3, kind: SegmentKind::Embedded });
    assert!(matches!(
        Cdt::new(&p),
        Err(sm_scheme_tri::TriError::SelfIntersection { .. })
    ));
}

#[test]
fn bad_input_is_rejected_not_panicking() {
    let mut p = pslg_of(&loops2d::square());
    p.points[1][0] = f64::NAN;
    assert!(Cdt::new(&p).is_err());
    let mut p = pslg_of(&loops2d::square());
    p.points[1][0] = 1e40; // outside 2^100
    assert!(matches!(Cdt::new(&p), Err(sm_scheme_tri::TriError::OutOfRange { .. })));
    let mut p = pslg_of(&loops2d::square());
    p.points.push(p.points[0]);
    assert!(matches!(Cdt::new(&p), Err(sm_scheme_tri::TriError::DuplicatePoint { .. })));
    let mut p = pslg_of(&loops2d::square());
    p.segments.push(Segment { a: 0, b: 99, kind: SegmentKind::Boundary });
    assert!(matches!(Cdt::new(&p), Err(sm_scheme_tri::TriError::BadInput(_))));
}

#[test]
fn collinear_overlapping_segments_are_split() {
    // A long embedded segment through intermediate vertices, plus overlapping duplicates.
    let mut p = pslg_of(&loops2d::square());
    let base = p.points.len() as u32;
    for k in 0..5 {
        p.points.push([0.0 + 0.25 * k as f64, 0.5]);
    }
    p.segments.push(Segment { a: base, b: base + 4, kind: SegmentKind::Embedded });
    p.segments.push(Segment { a: base + 1, b: base + 3, kind: SegmentKind::Embedded });
    let c = Cdt::new(&p).expect("cdt");
    assert!(all_positive(&c));
    for k in 0..4 {
        assert!(c.is_constrained(base + k, base + k + 1));
    }
}
