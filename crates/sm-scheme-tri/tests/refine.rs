//! Refinement on the pure kernel: angles, counts, Delaunay property, conformity.

mod common;

use common::*;
use sm_scheme_tri::cdt::{Cdt, OptimizeOptions, Pslg, RefineOptions, Segment, SegmentKind};
use sm_testkit::loops2d::{self, Region2};

fn subdivided(r: &Region2, h: f64) -> Pslg {
    // Split every boundary segment into pieces no longer than ~h (to mimic curve schemes).
    let mut p = Pslg::default();
    for l in r.loops() {
        let base = p.points.len() as u32;
        let mut ring: Vec<[f64; 2]> = Vec::new();
        for i in 0..l.len() {
            let (a, b) = (l[i], l[(i + 1) % l.len()]);
            let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            let n = (len / h).round().max(1.0) as usize;
            for k in 0..n {
                let s = k as f64 / n as f64;
                ring.push([a.0 + s * (b.0 - a.0), a.1 + s * (b.1 - a.1)]);
            }
        }
        let m = ring.len() as u32;
        p.points.extend(ring);
        for i in 0..m {
            p.segments.push(Segment { a: base + i, b: base + (i + 1) % m, kind: SegmentKind::Boundary });
        }
    }
    p
}

fn run(r: &Region2, h: f64, optimize: bool) -> (Cdt, f64, usize) {
    let p = subdivided(r, h);
    let mut c = Cdt::new(&p).expect("cdt");
    let g = Flat(h);
    let rep = c.refine(&g, &RefineOptions::default());
    if optimize {
        c.optimize(&g, &OptimizeOptions::default());
    }
    let (ang, _) = c.min_angle(&g).unwrap();
    (c, ang, rep.unfixable.len())
}

#[test]
fn unit_square_meets_twenty_degrees() {
    let (c, ang, unfixable) = run(&loops2d::square(), 0.1, true);
    let n = c.triangles().len();
    println!("square: N={n} min angle {ang:.2} unfixable {unfixable}");
    assert!(ang >= 20.0, "{ang}");
    assert!((180..=330).contains(&n), "{n}");
    assert_eq!(unfixable, 0);
    assert!(all_positive(&c));
    assert!((area(&c) - 1.0).abs() < 1e-12);
}

#[test]
fn regions_meet_twenty_degrees() {
    let cases: [(Region2, f64); 4] = [
        (loops2d::circle(1.0, 64), 2.0 * std::f64::consts::PI / 64.0),
        (loops2d::annulus(1.0, 3.0, 48), 2.0 * std::f64::consts::PI / 48.0),
        (loops2d::l_shape(), 0.1),
        (loops2d::rectangle_with_holes(), 0.5),
    ];
    for (r, h) in cases {
        for opt in [false, true] {
            let (c, ang, unfixable) = run(&r, h, opt);
            println!("{} opt={opt}: N={} angle {ang:.2} unfixable {unfixable}", r.name, c.triangles().len());
            assert!(all_positive(&c), "{}", r.name);
            assert!(ang >= 20.0, "{} opt={opt}: {ang}", r.name);
            if !opt {
                assert_eq!(delaunay_violations(&c), 0, "{}", r.name);
            }
        }
    }
}
