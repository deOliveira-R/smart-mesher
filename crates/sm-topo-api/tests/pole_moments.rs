//! Face/volume moments on faces with pole/apex coedges (CR-1) and planarity far from the origin
//! (CR-2), against analytic results.

use core::f64::consts::{PI, TAU};
use sm_core::EntityId;
use sm_core::{Point3, Vector3};
use sm_geom_api::{Cone, CurveDef, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Uv};
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
    assert!(
        close(vm.volume, 4.0 / 3.0 * PI * r.powi(3), 1e-9),
        "volume {}",
        vm.volume
    );
    assert!(vm.first.norm() < 1e-9);
    assert!(close(vm.second[2][2], 4.0 * PI * r.powi(5) / 15.0, 1e-9));
    assert!(fm.plane(1e-3).is_none(), "a sphere is not planar");
}

/// Closed cone about z: base disc of radius `r` at z = 0 (normal +z), apex below at z = -h.
/// Returns `(lateral face, volume)`.
fn closed_cone(t: &mut CellComplex, r: f64, a: f64) -> (EntityId, EntityId) {
    let slant = r / a.sin();
    with_op(t, OpId(1), |t| -> Result<_, FixtureError> {
        let vb = t.make_vertex(Point3::new(r, 0.0, 0.0))?;
        let cone = Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r, a)?;
        let va = t.make_vertex(cone.apex())?;
        let base = t
            .shapes_mut()?
            .add_curve(CurveDef::Circle(sm_geom_api::Circle::new(
                Frame::at(Point3::new(0.0, 0.0, 0.0)),
                r,
            )?))?;
        let gen_dir = Vector3::new(-a.sin(), 0.0, -a.cos());
        let gen_curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(
            Point3::new(r, 0.0, 0.0),
            gen_dir,
        )?))?;
        let full = Interval::new(0.0, TAU);
        let e_base = t.make_edge(
            vb,
            vb,
            Some(CurveCarrier {
                curve: base,
                range: full,
                sense: Sense::Forward,
            }),
        )?;
        let e_seam = t.make_edge(
            vb,
            va,
            Some(CurveCarrier {
                curve: gen_curve,
                range: Interval::new(0.0, slant),
                sense: Sense::Forward,
            }),
        )?;
        let e_apex = t.make_edge(va, va, None)?;
        t.set_flags(e_apex, CellFlags::DEGENERATE)?;
        let surface = t.shapes_mut()?.add_surface(SurfaceDef::Cone(cone))?;
        // Loop counter-clockwise in (u, v): apex row, seam up at u = 2 pi, base back, seam down at u = 0.
        let lp = [
            CoedgeSpec {
                edge: e_apex,
                sense: Sense::Forward,
            },
            CoedgeSpec {
                edge: e_seam,
                sense: Sense::Reversed,
            },
            CoedgeSpec {
                edge: e_base,
                sense: Sense::Reversed,
            },
            CoedgeSpec {
                edge: e_seam,
                sense: Sense::Forward,
            },
        ];
        let face = t.make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface,
                sense: Sense::Forward,
            }),
        )?;
        let line2 = |o: (f64, f64), d: (f64, f64), range| {
            PCurve::Line2(Line2 {
                origin: Uv::new(o.0, o.1),
                dir: Uv::new(d.0, d.1),
                range,
            })
        };
        let key = |edge, side| CoedgeKey { face, edge, side };
        let av = cone.apex_v();
        t.set_pcurve(
            key(e_apex, Side::Left),
            Some(line2((0.0, av), (1.0, 0.0), full)),
        )?;
        t.set_pcurve(
            key(e_seam, Side::Right),
            Some(line2((TAU, 0.0), (0.0, -1.0), Interval::new(0.0, slant))),
        )?;
        t.set_pcurve(
            key(e_base, Side::Right),
            Some(line2((0.0, 0.0), (1.0, 0.0), full)),
        )?;
        t.set_pcurve(
            key(e_seam, Side::Left),
            Some(line2((0.0, 0.0), (0.0, -1.0), Interval::new(0.0, slant))),
        )?;
        let disc_surface = t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane {
            frame: Frame::from_zx(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
                Vector3::new(1.0, 0.0, 0.0),
            )?,
        }))?;
        let disc = t.make_face(
            &[&[CoedgeSpec {
                edge: e_base,
                sense: Sense::Forward,
            }]],
            Some(SurfaceCarrier {
                surface: disc_surface,
                sense: Sense::Forward,
            }),
        )?;
        let uses = [face, disc].map(|face| FaceUseSpec {
            face,
            side: Side::Left,
        });
        let volume = t.make_volume(&[ShellSpec {
            kind: ShellKind::Outer,
            uses: &uses,
        }])?;
        Ok((face, volume))
    })
    .unwrap()
}

#[test]
fn cone_lateral_face_and_closed_volume_with_apex_are_analytic() {
    let (r, a) = (2.0f64, 0.4f64);
    let slant = r / a.sin();
    let h = r / a.tan();
    let mut t = CellComplex::new();
    let (face, volume) = closed_cone(&mut t, r, a);
    let fm = t.face_moments(face).unwrap();
    // Signed area: positive, so the face normal agrees with the stored (outward) sense.
    assert!(
        close(fm.area, PI * r * slant, 1e-9),
        "area {} vs {}",
        fm.area,
        PI * r * slant
    );
    // Outward lateral vector area balances the base disc (+z normal): -pi r^2 along z.
    assert!(fm.vector_area.x.abs() < 1e-9 && fm.vector_area.y.abs() < 1e-9);
    assert!(
        close(fm.vector_area.z, -PI * r * r, 1e-9),
        "{:?}",
        fm.vector_area
    );
    // Lateral centroid on the axis at h/3; int z^2 dA = A h^2 / 6; int (x^2+y^2) dA = A r^2 / 2.
    let c = fm.centroid().unwrap();
    assert!(
        c.x.abs() < 1e-9 && c.y.abs() < 1e-9 && close(c.z, -h / 3.0, 1e-9),
        "centroid {c:?}"
    );
    assert!(close(fm.second[2][2], fm.area * h * h / 6.0, 1e-9));
    assert!(close(
        fm.second[0][0] + fm.second[1][1],
        fm.area * r * r / 2.0,
        1e-9
    ));
    // Closed volume: V = pi r^2 h / 3, centroid at h/4.
    let vm = t.volume_moments(volume).unwrap();
    assert!(
        close(vm.volume, PI * r * r * h / 3.0, 1e-9),
        "volume {}",
        vm.volume
    );
    let vc = vm.centroid().unwrap();
    assert!(
        vc.x.abs() < 1e-9 && vc.y.abs() < 1e-9 && close(vc.z, -h / 4.0, 1e-9),
        "{vc:?}"
    );
}

#[test]
fn translated_unit_square_stays_planar_far_from_the_origin() {
    for (off, tol) in [(0.0, 1e-9), (1e6, 1e-8), (1e9, 1e-6)] {
        let mut t = CellComplex::new();
        let b = with_op(&mut t, OpId(1), |t| {
            fixtures::brick(t, Point3::new(off, off, off), [1.0; 3])
        })
        .unwrap();
        for f in b.faces {
            let fm = t.face_moments(f).unwrap();
            assert!(close(fm.area, 1.0, 1e-9), "offset {off}: area {}", fm.area);
            let (n, d) = t
                .face_plane(f, tol)
                .unwrap()
                .unwrap_or_else(|| panic!("offset {off}: planar face rejected at {tol}"));
            // (n, d) are exactly those of the origin-based form.
            let (n0, d0) = fm.plane(f64::INFINITY).unwrap();
            assert!(n == n0 && d == d0);
            let fc = t.face_central(f).unwrap();
            assert!(close(fc.area, 1.0, 1e-9));
            // A unit square has central second moment 1/12 on its two in-plane axes, 0 along n.
            let tr = fc.central[0][0] + fc.central[1][1] + fc.central[2][2];
            assert!(close(tr, 1.0 / 6.0, 1e-6), "offset {off}: trace {tr}");
            if off == 1e6 {
                // The origin-based formula cannot do this.
                assert!(
                    fm.plane(tol).is_none(),
                    "offset {off}: the uncentred form is expected to cancel"
                );
            }
        }
    }
}

#[test]
fn central_moments_combine_by_the_parallel_axis_theorem() {
    // Two unit squares side by side in z = 0: [0,1]^2 and [1,2] x [0,1] -> a 2 x 1 rectangle.
    let sq = |x0: f64| FaceCentral {
        area: 1.0,
        centroid: Point3::new(x0 + 0.5, 0.5, 0.0),
        central: [[1.0 / 12.0, 0.0, 0.0], [0.0, 1.0 / 12.0, 0.0], [0.0; 3]],
    };
    let c = sq(0.0).combine(&sq(1.0));
    assert!(close(c.area, 2.0, 1e-15) && close(c.centroid.x, 1.0, 1e-15));
    assert!(
        close(c.central[0][0], 2.0 * 2.0 * 2.0 / 12.0, 1e-14),
        "{}",
        c.central[0][0]
    ); // A w^2/12 = 2*4/12
    assert!(close(c.central[1][1], 2.0 / 12.0, 1e-14));
}

#[test]
fn composite_face_planarity_uses_combined_central_moments() {
    use std::sync::Arc;
    let off = 1e6;
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| {
        fixtures::brick(t, Point3::new(off, off, off), [1.0; 3])
    })
    .unwrap();
    let (halves, _chord) = with_op(&mut t, OpId(2), |t| {
        let line = t
            .shapes_mut()
            .unwrap()
            .add_curve(CurveDef::Line(
                Line::new(
                    Point3::new(off, off, off + 1.0),
                    Vector3::new(1.0, 1.0, 0.0),
                )
                .unwrap(),
            ))
            .unwrap();
        let chord = t.make_edge(
            b.vertices[4],
            b.vertices[7],
            Some(CurveCarrier {
                curve: line,
                range: Interval::new(0.0, 2f64.sqrt()),
                sense: Sense::Forward,
            }),
        )?;
        Ok((
            t.split_face(
                b.faces[5],
                FaceCut::Chord { edge: chord },
                InnerLoopAssign::None,
            )?,
            chord,
        ))
    })
    .unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t.clone());
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&halves).unwrap();
    let fc = ov.face_central(vf).unwrap();
    assert!(close(fc.area, 1.0, 1e-9));
    assert!(
        (fc.centroid.x - (off + 0.5)).abs() < 1e-6 && (fc.centroid.z - (off + 1.0)).abs() < 1e-6
    );
    // Unit square: tr(C) = 1/6, no out-of-plane spread.
    assert!(
        close(
            fc.central[0][0] + fc.central[1][1] + fc.central[2][2],
            1.0 / 6.0,
            1e-6
        ),
        "{:?}",
        fc.central
    );
    assert!(
        ov.face_plane(vf, 1e-8).unwrap().is_some(),
        "planar composite face rejected far from the origin"
    );
    // The origin-based form on the composite moments cannot do this.
    assert!(ov.face_moments(vf).unwrap().plane(1e-8).is_none());
}
