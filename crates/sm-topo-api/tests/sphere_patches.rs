//! Analytic area/volume of sphere patches that reach a pole at a vertex without a degenerate
//! coedge (an octant, as a webcut produces) and of a band between two latitudes (no pole).

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use sm_core::EntityId;
use sm_core::{Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Uv};
use sm_topo_api::fixtures::{FixtureError, with_op};
use sm_topo_api::*;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

fn line2(o: (f64, f64), d: (f64, f64), range: Interval) -> PCurve {
    PCurve::Line2(Line2 { origin: Uv::new(o.0, o.1), dir: Uv::new(d.0, d.1), range })
}

fn circ(t: &mut CellComplex, frame: Frame, r: f64) -> Result<EntityId, FixtureError> {
    Ok(t.shapes_mut()?.add_curve(CurveDef::Circle(sm_geom_api::Circle::new(frame, r)?))?)
}

fn plane_surface(t: &mut CellComplex, origin: Point3, normal: Vector3, x: Vector3) -> Result<EntityId, FixtureError> {
    let frame = Frame::from_zx(origin, normal, x)?;
    Ok(t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame }))?)
}

fn cs(edge: EntityId, sense: Sense) -> CoedgeSpec {
    CoedgeSpec { edge, sense }
}

fn carrier(curve: EntityId, range: Interval) -> Option<CurveCarrier> {
    Some(CurveCarrier { curve, range, sense: Sense::Forward })
}

fn fwd(surface: EntityId) -> Option<SurfaceCarrier> {
    Some(SurfaceCarrier { surface, sense: Sense::Forward })
}

fn line_edge(t: &mut CellComplex, a: EntityId, b: EntityId, d: Vector3, len: f64) -> Result<EntityId, FixtureError> {
    let l = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), d)?))?;
    Ok(t.make_edge(a, b, carrier(l, Interval::new(0.0, len)))?)
}

/// Sphere octant `x, y, z >= 0` (webcut style): the spherical face reaches the pole at a vertex
/// with no degenerate coedge; closed by three quarter discs.
#[test]
fn sphere_octant_without_degenerate_coedge_is_analytic() {
    let r = 1.7;
    let mut t = CellComplex::new();
    let (sph, vol) = with_op(&mut t, OpId(1), |t| -> Result<_, FixtureError> {
        let o = t.make_vertex(Point3::new(0.0, 0.0, 0.0))?;
        let a = t.make_vertex(Point3::new(r, 0.0, 0.0))?;
        let b = t.make_vertex(Point3::new(0.0, r, 0.0))?;
        let n = t.make_vertex(Point3::new(0.0, 0.0, r))?;
        let p0 = Point3::new(0.0, 0.0, 0.0);
        let c_eq = circ(t, Frame::at(p0), r)?;
        let c_xz = circ(t, Frame::from_zx(p0, Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?, r)?;
        let c_yz = circ(t, Frame::from_zx(p0, Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0))?, r)?;
        let q = Interval::new(0.0, FRAC_PI_2);
        let e_ab = t.make_edge(a, b, carrier(c_eq, q))?;
        let e_an = t.make_edge(a, n, carrier(c_xz, q))?;
        let e_bn = t.make_edge(b, n, carrier(c_yz, q))?;
        let e_oa = line_edge(t, o, a, Vector3::new(1.0, 0.0, 0.0), r)?;
        let e_ob = line_edge(t, o, b, Vector3::new(0.0, 1.0, 0.0), r)?;
        let e_on = line_edge(t, o, n, Vector3::new(0.0, 0.0, 1.0), r)?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Sphere(sm_geom_api::Sphere::new(Frame::at(p0), r)?))?;
        let sph = t.make_face(&[&[cs(e_ab, Sense::Forward), cs(e_bn, Sense::Forward), cs(e_an, Sense::Reversed)]], fwd(surface))?;
        let key = |edge, side| CoedgeKey { face: sph, edge, side };
        t.set_pcurve(key(e_ab, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), q)))?;
        t.set_pcurve(key(e_bn, Side::Left), Some(line2((FRAC_PI_2, 0.0), (0.0, 1.0), q)))?;
        t.set_pcurve(key(e_an, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), q)))?;
        let s_xy = plane_surface(t, p0, Vector3::new(0.0, 0.0, -1.0), Vector3::new(1.0, 0.0, 0.0))?;
        let s_xz = plane_surface(t, p0, Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?;
        let s_yz = plane_surface(t, p0, Vector3::new(-1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0))?;
        let f_xy = t.make_face(&[&[cs(e_ob, Sense::Forward), cs(e_ab, Sense::Reversed), cs(e_oa, Sense::Reversed)]], fwd(s_xy))?;
        let f_xz = t.make_face(&[&[cs(e_oa, Sense::Forward), cs(e_an, Sense::Forward), cs(e_on, Sense::Reversed)]], fwd(s_xz))?;
        let f_yz = t.make_face(&[&[cs(e_on, Sense::Forward), cs(e_bn, Sense::Reversed), cs(e_ob, Sense::Reversed)]], fwd(s_yz))?;
        let uses = [sph, f_xy, f_xz, f_yz].map(|face| FaceUseSpec { face, side: Side::Left });
        let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
        Ok((sph, volume))
    })
    .unwrap();
    let fm = t.face_moments(sph).unwrap();
    assert!(close(fm.area, PI * r * r / 2.0, 1e-9), "area {}", fm.area);
    // int x dA over the octant = pi r^3 / 4, and likewise for y and z.
    let want = PI * r.powi(3) / 4.0;
    for c in [fm.first.x, fm.first.y, fm.first.z] {
        assert!(close(c, want, 1e-9), "first moment {c} vs {want}");
    }
    let vm = t.volume_moments(vol).unwrap();
    assert!(close(vm.volume, PI * r.powi(3) / 6.0, 1e-9), "volume {}", vm.volume);
    // The centroid of a solid octant is 3 r / 8 on each axis.
    let vc = vm.centroid().unwrap();
    for c in [vc.x, vc.y, vc.z] {
        assert!(close(c, 3.0 * r / 8.0, 1e-9), "centroid {vc:?}");
    }
}

/// Sphere band between latitudes `v0 < v1` (no pole), closed by two discs.
#[test]
fn sphere_band_between_latitudes_is_analytic() {
    let (r, v0, v1) = (1.3f64, -0.3f64, 0.5f64);
    let (z0, z1) = (r * v0.sin(), r * v1.sin());
    let mut t = CellComplex::new();
    let (band, vol) = with_op(&mut t, OpId(1), |t| -> Result<_, FixtureError> {
        let p0 = t.make_vertex(Point3::new(r * v0.cos(), 0.0, z0))?;
        let p1 = t.make_vertex(Point3::new(r * v1.cos(), 0.0, z1))?;
        let full = Interval::new(0.0, TAU);
        let arc = Interval::new(v0, v1);
        let o = Point3::new(0.0, 0.0, 0.0);
        let c0 = circ(t, Frame::at(Point3::new(0.0, 0.0, z0)), r * v0.cos())?;
        let c1 = circ(t, Frame::at(Point3::new(0.0, 0.0, z1)), r * v1.cos())?;
        let mer = circ(t, Frame::from_zx(o, Vector3::new(0.0, -1.0, 0.0), Vector3::new(1.0, 0.0, 0.0))?, r)?;
        let e0 = t.make_edge(p0, p0, carrier(c0, full))?;
        let e1 = t.make_edge(p1, p1, carrier(c1, full))?;
        let es = t.make_edge(p0, p1, carrier(mer, arc))?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Sphere(sm_geom_api::Sphere::new(Frame::at(o), r)?))?;
        let band = t.make_face(&[&[cs(e0, Sense::Forward), cs(es, Sense::Forward), cs(e1, Sense::Reversed), cs(es, Sense::Reversed)]], fwd(surface))?;
        let key = |edge, side| CoedgeKey { face: band, edge, side };
        t.set_pcurve(key(e0, Side::Left), Some(line2((0.0, v0), (1.0, 0.0), full)))?;
        t.set_pcurve(key(es, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), arc)))?;
        t.set_pcurve(key(e1, Side::Right), Some(line2((0.0, v1), (1.0, 0.0), full)))?;
        t.set_pcurve(key(es, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), arc)))?;
        let s0 = plane_surface(t, Point3::new(0.0, 0.0, z0), Vector3::new(0.0, 0.0, -1.0), Vector3::new(1.0, 0.0, 0.0))?;
        let s1 = plane_surface(t, Point3::new(0.0, 0.0, z1), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0))?;
        let d0 = t.make_face(&[&[cs(e0, Sense::Reversed)]], fwd(s0))?;
        let d1 = t.make_face(&[&[cs(e1, Sense::Forward)]], fwd(s1))?;
        let uses = [band, d0, d1].map(|face| FaceUseSpec { face, side: Side::Left });
        let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
        Ok((band, volume))
    })
    .unwrap();
    let fm = t.face_moments(band).unwrap();
    assert!(close(fm.area, TAU * r * (z1 - z0), 1e-9), "area {}", fm.area);
    let vm = t.volume_moments(vol).unwrap();
    let want = PI * (r * r * (z1 - z0) - (z1.powi(3) - z0.powi(3)) / 3.0);
    assert!(close(vm.volume, want, 1e-9), "volume {} vs {want}", vm.volume);
}
