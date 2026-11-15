//! W4-BOOL-KFIX: kernel defects found by the BEAVRS import. A webcut by a cylinder of a slab
//! between two bodies it has already cut must split the slab; `locate_point` on a thin annulus
//! must not report a false `In`.
mod common;

use core::f64::consts::PI;
use sm_boolean::{CutTool, WebcutOptions, webcut};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Cylinder, Frame, SurfaceDef};
use sm_topo_api::{Dim, OpId, TopologyEdit, TopologyView, fixtures::brick};

fn vol(t: &sm_topo::TopoStore, b: EntityId) -> f64 {
    let v = t
        .boundary(b)
        .iter()
        .map(|i| i.cell)
        .find(|c| t.dim_of(*c) == Some(Dim::Volume))
        .unwrap();
    t.volume_moments(v).unwrap().volume
}

fn plane(o: [f64; 3], n: [f64; 3]) -> CutTool {
    CutTool::Plane {
        origin: Point3::new(o[0], o[1], o[2]),
        normal: Vector3::new(n[0], n[1], n[2]),
    }
}

fn cylinder(r: f64, seam_angle: f64) -> CutTool {
    let f0 = Frame::at(Point3::new(0.0, 0.0, 0.0));
    let x = f0.x * seam_angle.cos() + f0.y * seam_angle.sin();
    CutTool::Surface {
        surface: SurfaceDef::Cylinder(
            Cylinder::new(Frame::from_zx(f0.origin, f0.z, x).unwrap(), r).unwrap(),
        ),
        window: None,
    }
}

/// Box `[-h,h]^2 x [0,10]` cut by z = 3 and z = 4, the outer pieces cut by the cylinder, then the
/// slab. With `trim` the slab is first cut to a box by four planes. Returns slab (neg, pos) volumes.
fn slab_after_neighbours(trim: bool, r: f64, seam_angle: f64) -> (Vec<f64>, Vec<f64>) {
    let mut t = sm_topo::TopoStore::new();
    t.begin_op(OpId(1)).unwrap();
    let (h, g) = (0.62992, 0.61015);
    let b = brick(&mut t, Point3::new(-h, -h, 0.0), [2.0 * h, 2.0 * h, 10.0]).unwrap();
    let cyl = cylinder(r, seam_angle);
    let cut = |t: &mut sm_topo::TopoStore, body: EntityId, tool: &CutTool| {
        webcut(t, &[body], tool, &WebcutOptions::default()).unwrap()
    };
    let below = cut(&mut t, b.body, &plane([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]));
    let above = cut(
        &mut t,
        below.positive[0],
        &plane([0.0, 0.0, 4.0], [0.0, 0.0, 1.0]),
    );
    let (mut slab, above) = (above.negative[0], above.positive[0]);
    cut(&mut t, below.negative[0], &cyl);
    cut(&mut t, above, &cyl);
    if trim {
        for tool in [
            plane([0.0, g, 0.0], [0.0, -1.0, 0.0]),
            plane([0.0, -g, 0.0], [0.0, 1.0, 0.0]),
            plane([g, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            plane([-g, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ] {
            let c = cut(&mut t, slab, &tool);
            slab = c
                .negative
                .into_iter()
                .chain(c.positive)
                .max_by(|a, b| vol(&t, *a).total_cmp(&vol(&t, *b)))
                .unwrap();
        }
    }
    let c = cut(&mut t, slab, &cyl);
    (
        c.negative.iter().map(|&b| vol(&t, b)).collect(),
        c.positive.iter().map(|&b| vol(&t, b)).collect(),
    )
}

#[test]
fn a_cylinder_splits_a_slab_between_two_bodies_it_already_cut() {
    for r in [0.39218, 0.4572] {
        let want = PI * r * r;
        for trim in [false, true] {
            for seam in [0.0, 1.0] {
                let (neg, pos) = slab_after_neighbours(trim, r, seam);
                assert!(
                    neg.len() == 1 && (neg[0] - want).abs() < 1e-9 && pos.len() == 1,
                    "r {r} trim {trim} seam {seam}: neg {neg:?} pos {pos:?} want {want}"
                );
                let total = if trim {
                    1.2203 * 1.2203
                } else {
                    2.0 * 0.62992_f64 * 2.0 * 0.62992
                };
                assert!(
                    (neg[0] + pos[0] - total).abs() < 1e-9,
                    "sum {}",
                    neg[0] + pos[0]
                );
            }
        }
    }
}

/// A thin annulus `r0 < r < r1`, `0 < z < 1` made by two cylinder webcuts of a brick.
fn thin_annulus(r0: f64, r1: f64, seam: f64) -> (sm_topo::TopoStore, EntityId) {
    let mut t = sm_topo::TopoStore::new();
    t.begin_op(OpId(1)).unwrap();
    let b = brick(&mut t, Point3::new(-1.0, -1.0, 0.0), [2.0, 2.0, 1.0]).unwrap();
    let c0 = webcut(
        &mut t,
        &[b.body],
        &cylinder(r0, seam),
        &WebcutOptions::default(),
    )
    .unwrap();
    let c1 = webcut(
        &mut t,
        &[c0.positive[0]],
        &cylinder(r1, seam + 0.5),
        &WebcutOptions::default(),
    )
    .unwrap();
    let body = c1.negative[0];
    let v = t
        .boundary(body)
        .iter()
        .map(|i| i.cell)
        .find(|c| t.dim_of(*c) == Some(Dim::Volume))
        .unwrap();
    let want = PI * (r1 * r1 - r0 * r0);
    assert!(
        (t.volume_moments(v).unwrap().volume - want).abs() < 1e-9,
        "annulus volume"
    );
    (t, v)
}

/// Seeded probe of a thin annulus along many directions: `locate_point` never says `In` outside
/// it and never `Out` inside it (points within 1e-3 of a boundary surface are skipped).
#[test]
fn locate_point_on_a_thin_annulus_matches_the_analytic_answer() {
    use sm_boolean::{PointLocation, locate_point};
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let (r0, r1) = (0.4, 0.414);
    let mut bad = Vec::new();
    let mut n = 0;
    for seam in [0.0, 1.0] {
        let (t, v) = thin_annulus(r0, r1, seam);
        for _ in 0..1500 {
            let th = next() * 2.0 * PI;
            let r = 0.15 + next() * 0.5;
            let z = -0.3 + next() * 1.6;
            if [r0, r1].iter().any(|b| (r - b).abs() < 1e-3)
                || z.abs() < 1e-3
                || (z - 1.0).abs() < 1e-3
            {
                continue;
            }
            let p = Point3::new(r * th.cos(), r * th.sin(), z);
            let inside = r > r0 && r < r1 && z > 0.0 && z < 1.0;
            n += 1;
            match locate_point(&t, v, p) {
                Ok(PointLocation::In) if inside => {}
                Ok(PointLocation::Out) if !inside => {}
                other => bad.push((p, inside, other.map_err(|e| e.to_string()))),
            }
        }
    }
    assert!(
        bad.is_empty(),
        "{} of {n} wrong, first {:?}",
        bad.len(),
        &bad[..bad.len().min(5)]
    );
}

/// Points whose first probe ray leaves through the top face within a few 1e-4 of the rim of a
/// plain disk (the inner piece of a brick cut by an ordinary pin) and of a thin annulus: the hit
/// lies between the sampled chord and the arc, and must still be counted.
#[test]
fn locate_point_with_a_rim_exit_hit_is_exact_on_a_disk_and_an_annulus() {
    use sm_boolean::{PointLocation, locate_point};
    let d: [f64; 3] = [
        0.577_350_269_189_625_8,
        0.707_106_781_186_547_5 + 0.131_578_947_368_421,
        0.418_879_020_478_639_1,
    ];
    let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let d = [d[0] / n, d[1] / n, d[2] / n];
    let phi0 = d[1].atan2(d[0]);
    let r1 = 0.414;
    // Disk: inner piece of the brick cut by a default-frame cylinder.
    let (disk, dv) = {
        let mut t = sm_topo::TopoStore::new();
        t.begin_op(OpId(1)).unwrap();
        let b = brick(&mut t, Point3::new(-1.0, -1.0, 0.0), [2.0, 2.0, 1.0]).unwrap();
        let c = webcut(
            &mut t,
            &[b.body],
            &cylinder(r1, 0.0),
            &WebcutOptions::default(),
        )
        .unwrap();
        let v = t
            .boundary(c.negative[0])
            .iter()
            .map(|i| i.cell)
            .find(|c| t.dim_of(*c) == Some(Dim::Volume))
            .unwrap();
        (t, v)
    };
    let (ann, av) = thin_annulus(0.4, r1, 0.0);
    let mut bad = Vec::new();
    let mut tot = 0;
    for (t, v, r0) in [(&disk, dv, 0.0), (&ann, av, 0.4)] {
        for i in 0..400 {
            let phi = phi0 + (i as f64 / 400.0 - 0.5) * 0.6;
            for k in 1..8 {
                let rq = r1 - k as f64 * 2.5e-4;
                let tt = 0.05;
                let p = Point3::new(
                    rq * phi.cos() - tt * d[0],
                    rq * phi.sin() - tt * d[1],
                    1.0 - tt * d[2],
                );
                let rp = p.x.hypot(p.y);
                if rp > r1 - 2e-3 || (r0 > 0.0 && (rp - r0).abs() < 2e-3) {
                    continue;
                }
                tot += 1;
                let inside = rp > r0 && rp < r1;
                match locate_point(t, v, p) {
                    Ok(PointLocation::In) if inside => {}
                    Ok(PointLocation::Out) if !inside => {}
                    o => bad.push((p, inside, o.map_err(|e| e.to_string()))),
                }
            }
        }
    }
    assert!(tot > 1000);
    assert!(
        bad.is_empty(),
        "{} of {tot} wrong; first {:?}",
        bad.len(),
        &bad[..bad.len().min(5)]
    );
    // The reported repro point.
    let p = Point3::new(0.2986258284603378, 0.21816330603587458, 0.9809772257938587);
    assert_eq!(locate_point(&disk, dv, p).unwrap(), PointLocation::In);
}
