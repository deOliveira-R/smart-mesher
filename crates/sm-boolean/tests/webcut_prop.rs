//! Acceptance 6: random planes split a solid into two with analytic volumes and cap area; and the
//! stacked-bricks case at the op level.

mod common;

use common::*;
use proptest::prelude::*;
use sm_boolean::ops::{PlaneSpec, PointNormalPlane, Subtract, Unite, WebcutPlane};
use sm_boolean::{CutTool, WebcutOptions, webcut};
use sm_core::{Point3, Vector3};
use sm_topo_api::{CellComplex, TopologyView};

/// Volume of the unit cube below the plane and the area of the section, by clipping.
fn cube_section(o: Vector3, n: Vector3) -> (f64, f64) {
    let corners: Vec<Vector3> = (0..8).map(|i| Vector3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64)).collect();
    let d: Vec<f64> = corners.iter().map(|c| n.dot(*c - o)).collect();
    // Section polygon: edge crossings, ordered around their centroid.
    let mut pts: Vec<Vector3> = Vec::new();
    for i in 0..8usize {
        for bit in [1usize, 2, 4] {
            let j = i | bit;
            if j != i && d[i] * d[j] < 0.0 {
                let t = d[i] / (d[i] - d[j]);
                pts.push(corners[i] + (corners[j] - corners[i]) * t);
            }
        }
    }
    for i in 0..8 {
        if d[i] == 0.0 {
            pts.push(corners[i]);
        }
    }
    let area = if pts.len() < 3 {
        0.0
    } else {
        let c = pts.iter().fold(Vector3::new(0.0, 0.0, 0.0), |a, p| a + *p) * (1.0 / pts.len() as f64);
        let nn = n * (1.0 / n.norm());
        let u = if nn.x.abs() < 0.9 { Vector3::new(1.0, 0.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
        let e1 = nn.cross(u);
        let e1 = e1 * (1.0 / e1.norm());
        let e2 = nn.cross(e1);
        let mut v: Vec<(f64, Vector3)> = pts.iter().map(|p| ((*p - c).dot(e2).atan2((*p - c).dot(e1)), *p)).collect();
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut s = Vector3::new(0.0, 0.0, 0.0);
        for k in 0..v.len() {
            s = s + (v[k].1 - c).cross(v[(k + 1) % v.len()].1 - c);
        }
        0.5 * s.norm()
    };
    (volume_below(&corners, &d, o, n, area), area)
}

/// Volume of the cube on the negative side, by cone decomposition about the origin point `o`.
fn volume_below(corners: &[Vector3], d: &[f64], o: Vector3, n: Vector3, section: f64) -> f64 {
    let faces: [([usize; 4], Vector3); 6] = [
        ([0, 2, 3, 1], Vector3::new(0.0, 0.0, -1.0)),
        ([4, 5, 7, 6], Vector3::new(0.0, 0.0, 1.0)),
        ([0, 1, 5, 4], Vector3::new(0.0, -1.0, 0.0)),
        ([2, 6, 7, 3], Vector3::new(0.0, 1.0, 0.0)),
        ([0, 4, 6, 2], Vector3::new(-1.0, 0.0, 0.0)),
        ([1, 3, 7, 5], Vector3::new(1.0, 0.0, 0.0)),
    ];
    // V = (1/3) * sum over boundary faces of (n . (x - o)) * area, clipped to the negative side;
    // the cap contributes zero since o lies on it.
    let mut v = 0.0;
    for (f, nf) in faces {
        let mut poly: Vec<Vector3> = Vec::new();
        for k in 0..4 {
            let (a, b) = (f[k], f[(k + 1) % 4]);
            if d[a] <= 0.0 {
                poly.push(corners[a]);
            }
            if d[a] * d[b] < 0.0 {
                let t = d[a] / (d[a] - d[b]);
                poly.push(corners[a] + (corners[b] - corners[a]) * t);
            }
        }
        if poly.len() < 3 {
            continue;
        }
        let mut a2 = Vector3::new(0.0, 0.0, 0.0);
        for k in 1..poly.len() - 1 {
            a2 = a2 + (poly[k] - poly[0]).cross(poly[k + 1] - poly[0]);
        }
        let area = 0.5 * a2.dot(nf).abs();
        v += nf.dot(poly[0] - o) * area / 3.0;
    }
    let _ = (n, section);
    v
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(200))]

    #[test]
    fn random_planes_split_a_cube(
        nx in -1.0f64..1.0, ny in -1.0f64..1.0, nz in -1.0f64..1.0,
        ox in 0.2f64..0.8, oy in 0.2f64..0.8, oz in 0.2f64..0.8,
    ) {
        let n = Vector3::new(nx, ny, nz);
        prop_assume!(n.norm() > 0.2);
        let n = n * (1.0 / n.norm());
        let o = Vector3::new(ox, oy, oz);
        // Stay away from vertex/edge-degenerate cuts.
        let corners: Vec<f64> = (0..8).map(|i| n.dot(Vector3::new((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64) - o)).collect();
        prop_assume!(corners.iter().all(|c| c.abs() > 1e-3));
        let (v_neg, area) = cube_section(o, n);

        let mut t = CellComplex::new();
        let b = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
        let tool = CutTool::Plane { origin: Point3::new(o.x, o.y, o.z), normal: n };
        let out = in_op(&mut t, 2, |t| webcut(t, &[b.body], &tool, &WebcutOptions::default())).unwrap();
        assert_valid(&t);
        prop_assert_eq!(out.negative.len(), 1);
        prop_assert_eq!(out.positive.len(), 1);
        let vol_of = |body| t.boundary(body)[0].cell;
        let (vn, vp) = (volume_of(&t, vol_of(out.negative[0])), volume_of(&t, vol_of(out.positive[0])));
        prop_assert!((vn + vp - 1.0).abs() < 1e-9, "V1+V2 = {}", vn + vp);
        prop_assert!((vn - v_neg).abs() < 1e-9, "negative {} vs {}", vn, v_neg);
        let cap: f64 = out.cut_faces.iter().map(|&f| t.face_moments(f).unwrap().area).sum();
        prop_assert!((cap - area).abs() < 1e-9, "cap {} vs {}", cap, area);
    }
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(12))]

    #[test]
    fn random_planes_split_a_cylinder(tilt in 0.0f64..0.6, az in 0.0f64..6.2, d in -0.8f64..0.8) {
        // r = 1, h = 4 centered at the origin: a plane tilted by `tilt` rad about a horizontal
        // axis through (0,0,d) stays inside the lateral surface (|d| + tan(tilt) < 2).
        let mut r = common::rig::Rig::cc();
        let c = r.cylinder(4.0, 1.0, [0.0; 3]);
        let n = [tilt.sin() * az.cos(), tilt.sin() * az.sin(), tilt.cos()];
        let spec = PlaneSpec::PointNormal(PointNormalPlane { origin: [0.0, 0.0, d], normal: n });
        let del = r.run(&WebcutPlane { targets: vec![c.body], plane: spec, keep: None, merge: None, imprint: None }).unwrap();
        let t = r.topo();
        assert_valid(t);
        let vol = |b: sm_core::EntityId| volume_of(t, t.boundary(b)[0].cell);
        let (vn, vp) = (vol(del.outputs["negative"][0]), vol(del.outputs["positive"][0]));
        let total = 4.0 * std::f64::consts::PI;
        prop_assert!((vn + vp - total).abs() < 1e-7);
        prop_assert!((vn - std::f64::consts::PI * (2.0 + d)).abs() < 1e-7, "negative {} vs {}", vn, std::f64::consts::PI * (2.0 + d));
        let cap = t.face_moments(del.outputs["cut_faces"][0]).unwrap().area;
        prop_assert!((cap - std::f64::consts::PI / tilt.cos()).abs() < 1e-7, "cap {}", cap);
    }
}

#[test]
fn three_bricks_minus_the_middle_slab_leave_two_bodies() {
    let mut r = common::rig::Rig::cc();
    let a = r.brick([1.0; 3], [0.5, 0.5, 0.5]);
    let b = r.brick([1.0; 3], [1.5, 0.5, 0.5]);
    let c = r.brick([1.0; 3], [2.5, 0.5, 0.5]);
    let u = r.run(&Unite { bodies: vec![a.body, b.body, c.body], keep: None, regularize: None, tol: None }).unwrap();
    assert_eq!(u.outputs["bodies"].len(), 1);
    assert_eq!(u.outputs["volumes"].len(), 1);
    let slab = r.brick([1.0, 3.0, 3.0], [1.5, 0.5, 0.5]);
    let d = r.run(&Subtract { targets: u.outputs["bodies"].clone(), tools: vec![slab.body], keep: None, regularize: None, tol: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(d.outputs["bodies"].len(), 2);
    assert_eq!(d.outputs["volumes"].len(), 2);
    for &v in &d.outputs["volumes"] {
        assert!((volume_of(t, v) - 1.0).abs() < 1e-12);
    }
    assert_eq!(counts(t)[3], 2);
}
