//! The W3 reference fixtures (torus, cone, brick with a void, nested spheres) against analytic
//! results: counts, Euler characteristic, closed-form volumes and areas, pcurve consistency
//! (the surface at the pcurve point is the curve point), shell structure and region sides.

use core::f64::consts::PI;
use sm_core::{EntityId, Point3};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::*;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    Dim::ALL.map(|d| v.cells(d).len())
}

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
    let bad = validate_geometry(v);
    assert!(bad.is_empty(), "geometry violations: {bad:#?}");
}

/// At every sampled parameter, the surface at the pcurve point is the curve point.
fn assert_pcurves_agree(v: &dyn TopologyView) {
    let mut checked = 0;
    for &face in v.cells(Dim::Face) {
        let Carrier::Surface(sc) = v.carrier(face) else { continue };
        let surf = v.shapes().surface(sc.surface).expect("surface");
        for ce in v.coedges(face) {
            let Carrier::Curve(cc) = v.carrier(ce.key.edge) else { continue };
            let Some(pc) = v.pcurve(ce.key) else { continue };
            let curve = v.shapes().curve(cc.curve).expect("curve");
            for k in 0..=8 {
                let t = cc.range.lo + (cc.range.hi - cc.range.lo) * f64::from(k) / 8.0;
                let on_surface = surf.eval(pc.eval(t));
                let on_curve = curve.eval(t);
                assert!((on_surface - on_curve).norm() < 1e-9, "face {face:?} edge {:?} {:?} t={t}: {on_surface:?} vs {on_curve:?}", ce.key.edge, ce.key.side);
                checked += 1;
            }
        }
    }
    assert!(checked > 0, "no pcurve was checked");
}

fn build<T>(f: impl FnOnce(&mut CellComplex) -> Result<T, FixtureError>) -> (CellComplex, T) {
    let mut t = CellComplex::new();
    let ids = with_op(&mut t, OpId(1), f).unwrap();
    (t, ids)
}

#[test]
fn torus_is_one_vertex_two_seam_edges_and_one_face() {
    let (major, minor) = (5.0, 1.0);
    let (t, ids) = build(|t| fixtures::torus(t, major, minor));
    assert_eq!(counts(&t), [1, 2, 1, 1, 1]);
    assert_eq!(euler_characteristic(&t), 0);
    assert_valid(&t);
    assert_pcurves_agree(&t);
    // The vertex is the (u, v) = (0, 0) point of the surface.
    let Carrier::Surface(sc) = t.carrier(ids.face) else { panic!() };
    let p = t.shapes().surface(sc.surface).unwrap().eval(sm_geom_api::Uv::new(0.0, 0.0));
    let Carrier::Point(v) = t.carrier(ids.vertex) else { panic!() };
    assert!((p - *v).norm() < 1e-12);
    // The loop uses each edge once on each side.
    let loops: Vec<_> = t.loops(ids.face).collect();
    assert_eq!(loops.len(), 1);
    assert_eq!(loops[0].len(), 4);
    for e in ids.edges {
        let sides: Vec<Side> = t.coedges(ids.face).filter(|c| c.key.edge == e).map(|c| c.key.side).collect();
        assert_eq!(sides.len(), 2);
        assert!(sides.contains(&Side::Left) && sides.contains(&Side::Right));
        assert_eq!(t.edge_ends(e), Some((ids.vertex, ids.vertex)));
    }
    let s = t.face_sides(ids.face).unwrap();
    assert_eq!((s.left, s.right), (Region::Volume(ids.volume), Region::Exterior));
    let fm = t.face_moments(ids.face).unwrap();
    assert!(close(fm.area, 4.0 * PI * PI * major * minor, 1e-9), "area {}", fm.area);
    let vm = t.volume_moments(ids.volume).unwrap();
    assert!(close(vm.volume, 2.0 * PI * PI * major * minor * minor, 1e-9), "volume {}", vm.volume);
    assert!(vm.first.norm() < 1e-9, "centroid at the origin: {:?}", vm.first);
    // Moment about the axis: int (x^2 + y^2) dV = V (R^2 + 3/4 r^2).
    assert!(close(vm.second[0][0] + vm.second[1][1], vm.volume * (major * major + 0.75 * minor * minor), 1e-8), "{:?}", vm.second);
}

#[test]
fn cone_has_an_apex_pole_and_the_closed_forms() {
    let (r, h) = (2.0, 3.0);
    let (t, ids) = build(|t| fixtures::cone(t, r, h));
    assert_eq!(counts(&t), [2, 3, 2, 1, 1]);
    assert_eq!(euler_characteristic(&t), 2);
    assert_valid(&t);
    assert_pcurves_agree(&t);
    assert!(t.flags(ids.edges[2]).contains(CellFlags::DEGENERATE));
    let Carrier::Point(apex) = t.carrier(ids.vertices[0]) else { panic!() };
    assert_eq!(*apex, Point3::new(0.0, 0.0, h));
    let Carrier::Surface(sc) = t.carrier(ids.faces[0]) else { panic!() };
    let sm_geom_api::SurfaceDef::Cone(cone) = t.shapes().surface_def(sc.surface).unwrap().clone() else { panic!() };
    assert!((cone.apex() - *apex).norm() < 1e-12, "the surface apex is the vertex");
    for f in ids.faces {
        let s = t.face_sides(f).unwrap();
        assert_eq!((s.left, s.right), (Region::Volume(ids.volume), Region::Exterior));
    }
    let slant = r.hypot(h);
    assert!(close(t.face_moments(ids.faces[0]).unwrap().area, PI * r * slant, 1e-9));
    assert!(close(t.face_moments(ids.faces[1]).unwrap().area, PI * r * r, 1e-9));
    let vm = t.volume_moments(ids.volume).unwrap();
    assert!(close(vm.volume, PI * r * r * h / 3.0, 1e-9), "volume {}", vm.volume);
    let c = vm.centroid().unwrap();
    assert!(c.x.abs() < 1e-9 && c.y.abs() < 1e-9 && close(c.z, h / 4.0, 1e-9), "centroid {c:?}");
}

#[test]
fn brick_with_void_has_an_outer_and_a_void_shell() {
    let (min, size) = (Point3::new(0.0, 0.0, 0.0), [10.0, 10.0, 10.0]);
    let (vmin, vsize) = (Point3::new(4.0, 3.0, 2.0), [2.0, 3.0, 4.0]);
    let (t, ids) = build(|t| fixtures::brick_with_void(t, min, size, vmin, vsize));
    assert_eq!(counts(&t), [16, 24, 12, 1, 1]);
    assert_eq!(euler_characteristic(&t), 4);
    assert_valid(&t);
    assert_eq!(ids.outer.volume, ids.void.volume);
    assert_eq!(t.shell_kind(ids.outer.volume, 0), Some(ShellKind::Outer));
    assert_eq!(t.shell_kind(ids.outer.volume, 1), Some(ShellKind::Void));
    assert_eq!(t.shell_kind(ids.outer.volume, 2), None);
    let shell_faces = |ring: u16| -> Vec<EntityId> { t.boundary(ids.outer.volume).iter().filter(|e| e.ring == ring).map(|e| e.cell).collect() };
    let mut outer = shell_faces(0);
    outer.sort();
    let mut want = ids.outer.faces.to_vec();
    want.sort();
    assert_eq!(outer, want);
    let mut cavity = shell_faces(1);
    cavity.sort();
    let mut want = ids.void.faces.to_vec();
    want.sort();
    assert_eq!(cavity, want);
    for f in ids.outer.faces.iter().chain(&ids.void.faces) {
        let s = t.face_sides(*f).unwrap();
        assert_eq!((s.left, s.right), (Region::Volume(ids.outer.volume), Region::Exterior));
    }
    // Volume is the difference; the first moment is the difference of the box moments.
    let vm = t.volume_moments(ids.outer.volume).unwrap();
    let (vo, vc) = (size.iter().product::<f64>(), vsize.iter().product::<f64>());
    assert!(close(vm.volume, vo - vc, 1e-12), "volume {}", vm.volume);
    let cx = 0.5 * vo * size[0] * 1.0 - vc * (vmin.x + 0.5 * vsize[0]);
    assert!(close(vm.first.x, cx, 1e-12), "first moment x {} vs {cx}", vm.first.x);
    let area: f64 = t.cells(Dim::Face).iter().map(|f| t.face_moments(*f).unwrap().area).sum();
    let a = |s: [f64; 3]| 2.0 * (s[0] * s[1] + s[1] * s[2] + s[0] * s[2]);
    assert!(close(area, a(size) + a(vsize), 1e-12));
    // Cavity faces are outward from the solid: their normals point into the cavity.
    for f in ids.void.faces {
        let fm = t.face_moments(f).unwrap();
        let c = fm.centroid().unwrap();
        let inward = Point3::new(vmin.x + 0.5 * vsize[0], vmin.y + 0.5 * vsize[1], vmin.z + 0.5 * vsize[2]) - c;
        assert!(fm.vector_area.dot(inward) > 0.0, "cavity face normal points into the cavity");
    }
}

#[test]
fn nested_spheres_share_two_sided_spherical_faces() {
    let radii = [1.0, 2.0, 3.5];
    let (t, ids) = build(|t| fixtures::nested_spheres(t, &radii));
    let n = radii.len();
    assert_eq!(counts(&t), [2 * n, 3 * n, n, n, 1]);
    assert_eq!(euler_characteristic(&t), 2 * n as i64);
    assert_valid(&t);
    assert_pcurves_agree(&t);
    let cube = |r: f64| 4.0 / 3.0 * PI * r * r * r;
    for i in 0..n {
        let want = cube(radii[i]) - if i == 0 { 0.0 } else { cube(radii[i - 1]) };
        let got = t.volume_moments(ids.volumes[i]).unwrap().volume;
        assert!(close(got, want, 1e-9), "shell {i}: {got} vs {want}");
        let s = t.face_sides(ids.spheres[i].face).unwrap();
        let outside = if i + 1 < n { Region::Volume(ids.volumes[i + 1]) } else { Region::Exterior };
        assert_eq!((s.left, s.right), (Region::Volume(ids.volumes[i]), outside), "sphere {i}");
        assert!(close(t.face_moments(ids.spheres[i].face).unwrap().area, 4.0 * PI * radii[i] * radii[i], 1e-9));
        assert_eq!(t.shell_kind(ids.volumes[i], 0), Some(ShellKind::Outer));
        assert_eq!(t.shell_kind(ids.volumes[i], 1), if i == 0 { None } else { Some(ShellKind::Void) });
    }
    // The shells tile the ball: their volumes add to the outer ball.
    let total: f64 = ids.volumes.iter().map(|v| t.volume_moments(*v).unwrap().volume).sum();
    assert!(close(total, cube(radii[n - 1]), 1e-9));
    // One body holds every volume.
    assert_eq!(t.boundary(ids.body).len(), n);
}

#[test]
fn nested_spheres_reject_bad_radii() {
    for radii in [&[][..], &[2.0, 1.0], &[1.0, 1.0], &[-1.0, 2.0], &[0.0], &[1.0, f64::NAN], &[f64::INFINITY]] {
        let mut t = CellComplex::new();
        let r = with_op(&mut t, OpId(1), |t| fixtures::nested_spheres(t, radii));
        assert!(matches!(r, Err(FixtureError::Topo(TopoError::InvalidParameter(_)))), "{radii:?}: {r:?}");
        assert!(t.cells(Dim::Face).is_empty(), "the failed fixture was rolled back");
    }
}

#[test]
fn torus_rejects_a_spindle() {
    let mut t = CellComplex::new();
    assert!(matches!(with_op(&mut t, OpId(1), |t| fixtures::torus(t, 1.0, 2.0)), Err(FixtureError::Geom(_))));
}

#[test]
fn the_fixtures_are_deterministic() {
    let hash = || {
        let (t, _) = build(|t| {
            fixtures::torus(t, 5.0, 1.0)?;
            fixtures::cone(t, 2.0, 3.0)?;
            fixtures::brick_with_void(t, Point3::new(0.0, 0.0, 0.0), [10.0; 3], Point3::new(4.0, 4.0, 4.0), [2.0; 3])?;
            fixtures::nested_spheres(t, &[1.0, 2.0, 3.0])?;
            Ok(())
        });
        t.canonical_hash()
    };
    assert_eq!(hash(), hash());
}
