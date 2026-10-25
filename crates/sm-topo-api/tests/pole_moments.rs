//! Face/volume moments on faces with pole/apex coedges (CR-1) and planarity far from the origin
//! (CR-2), against analytic results.

use core::f64::consts::{PI, TAU};
use sm_core::{Point3, Vector3};
use sm_geom_api::{Cone, CurveDef, Frame, Interval, Line, Line2, PCurve, SurfaceDef, Uv};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::*;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs().max(1.0)
}

#[test]
fn sphere_face_and_volume_moments_are_analytic() {
    let r = 1.5;
    let mut t = CellComplex::new();
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, r)).unwrap();
    let fm = t.face_moments(s.face).unwrap();
    assert!(close(fm.area, 4.0 * PI * r * r, 1e-9), "area {}", fm.area);
    assert!(fm.vector_area.norm() < 1e-9 && fm.first.norm() < 1e-9);
    // int x^2 dA = 4 pi r^4 / 3
    assert!(close(fm.second[0][0], 4.0 * PI * r.powi(4) / 3.0, 1e-9));
    let vm = t.volume_moments(s.volume).unwrap();
    assert!(close(vm.volume, 4.0 / 3.0 * PI * r.powi(3), 1e-9), "volume {}", vm.volume);
    assert!(vm.first.norm() < 1e-9);
    assert!(close(vm.second[2][2], 4.0 * PI * r.powi(5) / 15.0, 1e-9));
    assert!(fm.plane(1e-3).is_none(), "a sphere is not planar");
}

#[test]
fn cone_lateral_face_with_apex_has_analytic_area() {
    let (r, a) = (2.0f64, 0.4f64);
    let slant = r / a.sin();
    let mut t = CellComplex::new();
    let face = with_op(&mut t, OpId(1), |t| -> Result<_, FixtureError> {
        let vb = t.make_vertex(Point3::new(r, 0.0, 0.0))?;
        let cone = Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r, a)?;
        let va = t.make_vertex(cone.apex())?;
        let base = t.shapes_mut()?.add_curve(CurveDef::Circle(sm_geom_api::Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?;
        let gen_dir = Vector3::new(-a.sin(), 0.0, -a.cos());
        let gen_curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(Point3::new(r, 0.0, 0.0), gen_dir)?))?;
        let full = Interval::new(0.0, TAU);
        let e_base = t.make_edge(vb, vb, Some(CurveCarrier { curve: base, range: full, sense: Sense::Forward }))?;
        let e_seam = t.make_edge(vb, va, Some(CurveCarrier { curve: gen_curve, range: Interval::new(0.0, slant), sense: Sense::Forward }))?;
        let e_apex = t.make_edge(va, va, None)?;
        t.set_flags(e_apex, CellFlags::DEGENERATE)?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Cone(cone))?;
        let lp = [
            CoedgeSpec { edge: e_base, sense: Sense::Forward },
            CoedgeSpec { edge: e_seam, sense: Sense::Forward },
            CoedgeSpec { edge: e_apex, sense: Sense::Reversed },
            CoedgeSpec { edge: e_seam, sense: Sense::Reversed },
        ];
        let face = t.make_face(&[&lp], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?;
        let line2 = |o: (f64, f64), d: (f64, f64), range| PCurve::Line2(Line2 { origin: Uv::new(o.0, o.1), dir: Uv::new(d.0, d.1), range });
        let key = |edge, side| CoedgeKey { face, edge, side };
        let av = cone.apex_v();
        t.set_pcurve(key(e_base, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
        t.set_pcurve(key(e_seam, Side::Left), Some(line2((TAU, 0.0), (0.0, -1.0), Interval::new(0.0, slant))))?;
        t.set_pcurve(key(e_apex, Side::Right), Some(line2((0.0, av), (1.0, 0.0), full)))?;
        t.set_pcurve(key(e_seam, Side::Right), Some(line2((0.0, 0.0), (0.0, -1.0), Interval::new(0.0, slant))))?;
        Ok(face)
    })
    .unwrap();
    let fm = t.face_moments(face).unwrap();
    assert!(close(fm.area.abs(), PI * r * slant, 1e-9), "area {} vs {}", fm.area, PI * r * slant);
}

#[test]
fn translated_unit_square_stays_planar_far_from_the_origin() {
    for (off, tol) in [(0.0, 1e-9), (1e6, 1e-8), (1e9, 1e-6)] {
        let mut t = CellComplex::new();
        let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(off, off, off), [1.0; 3])).unwrap();
        for f in b.faces {
            let fm = t.face_moments(f).unwrap();
            assert!(close(fm.area, 1.0, 1e-9), "offset {off}: area {}", fm.area);
            assert!(fm.plane(tol).is_some(), "offset {off}: planar face rejected at {tol}");
            if off == 1e6 {
                // The origin-based formula cannot do this.
                let old = FaceMoments { central: None, ..fm };
                assert!(old.plane(tol).is_none(), "offset {off}: the uncentred form is expected to cancel");
            }
        }
    }
}
