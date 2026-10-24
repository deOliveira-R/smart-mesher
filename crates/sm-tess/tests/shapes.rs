//! Faces the reference fixtures do not cover: a planar face with a hole, a cone with an apex
//! (pole row, unequal ring counts), a frustum (merge triangulation of unequal rings) and a torus
//! (two periodic directions, interior rows).

mod common;

use core::f64::consts::{PI, TAU};

use common::*;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, Cone, CurveDef, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Torus, Uv};
use sm_tess::{TessOptions, tessellate};
use sm_topo_api::fixtures::FixtureError;
use sm_topo_api::{CellFlags, CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, Sense, Side, SurfaceCarrier, TopologyEdit};

fn add_curve(t: &mut CellComplex, d: CurveDef) -> Result<EntityId, FixtureError> {
    Ok(t.shapes_mut()?.add_curve(d)?)
}

fn add_surface(t: &mut CellComplex, d: SurfaceDef) -> Result<EntityId, FixtureError> {
    Ok(t.shapes_mut()?.add_surface(d)?)
}

fn line2(origin: (f64, f64), dir: (f64, f64), range: Interval) -> PCurve {
    PCurve::Line2(Line2 { origin: Uv::new(origin.0, origin.1), dir: Uv::new(dir.0, dir.1), range })
}

fn opts(chord: f64) -> TessOptions {
    TessOptions { chord_tol: chord, angle_tol: 0.5, max_edge: f64::INFINITY }
}

fn fwd(edge: EntityId) -> CoedgeSpec {
    CoedgeSpec { edge, sense: Sense::Forward }
}

fn rev(edge: EntityId) -> CoedgeSpec {
    CoedgeSpec { edge, sense: Sense::Reversed }
}

/// A 10 x 10 plate in z = 0 with a circular hole of radius 2 at (5, 5).
fn plate_with_hole() -> (CellComplex, EntityId, EntityId) {
    build(|t| {
        let corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let vs: Vec<EntityId> = corners.iter().map(|c| t.make_vertex(Point3::new(c.0, c.1, 0.0))).collect::<Result<_, _>>()?;
        let mut loop_outer = Vec::new();
        for i in 0..4 {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            let d = Vector3::new(b.0 - a.0, b.1 - a.1, 0.0);
            let c = add_curve(t, CurveDef::Line(Line::new(Point3::new(a.0, a.1, 0.0), d)?))?;
            let e = t.make_edge(vs[i], vs[(i + 1) % 4], Some(CurveCarrier { curve: c, range: Interval::new(0.0, 10.0), sense: Sense::Forward }))?;
            loop_outer.push(fwd(e));
        }
        let vh = t.make_vertex(Point3::new(7.0, 5.0, 0.0))?;
        let hole = add_curve(t, CurveDef::Circle(Circle::new(Frame::at(Point3::new(5.0, 5.0, 0.0)), 2.0)?))?;
        let eh = t.make_edge(vh, vh, Some(CurveCarrier { curve: hole, range: Interval::new(0.0, TAU), sense: Sense::Forward }))?;
        let plane = add_surface(t, SurfaceDef::Plane(Plane { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)) }))?;
        let f = t.make_face(&[&loop_outer, &[rev(eh)]], Some(SurfaceCarrier { surface: plane, sense: Sense::Forward }))?;
        Ok((f, eh))
    })
    .pipe(|(c, (f, eh))| (c, f, eh))
}

trait Pipe: Sized {
    fn pipe<R>(self, f: impl FnOnce(Self) -> R) -> R {
        f(self)
    }
}
impl<T> Pipe for T {}

#[test]
fn planar_face_with_hole() {
    let (c, f, eh) = plate_with_hole();
    let t = tessellate(&c, &[f], &opts(1e-3)).unwrap();
    let n = t.edge_polyline(eh).unwrap().len() - 1;
    let hole_area = 0.5 * n as f64 * 4.0 * (TAU / n as f64).sin();
    assert!((face_area(&t, f) - (100.0 - hole_area)).abs() < 1e-4, "{}", face_area(&t, f));
    check_face_structure(&c, &t, f);
    check_normal_signs(&c, &t, f);
    // Every boundary point is used; no interior points are added.
    assert_eq!(t.vertex_count(), 4 + 1 + (n - 1));
}

/// Cone with its apex at the origin, half angle `a`, base circle at `v = vb`; `top` makes a frustum
/// cut at `v = vt` instead of the apex. Returns the lateral face and the edges `[base, top, seam]`.
fn cone_sheet(a: f64, vb: f64, vt: Option<f64>) -> (CellComplex, EntityId, [Option<EntityId>; 3]) {
    build(|t| {
        let (s, co) = a.sin_cos();
        let at = |v: f64| Point3::new(v * s, 0.0, v * co);
        let vtop = vt.unwrap_or(0.0);
        let v_base = t.make_vertex(at(vb))?;
        let v_top = t.make_vertex(at(vtop))?;
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let base_c = add_curve(t, CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(0.0, 0.0, vb * co), z, x)?, vb * s)?))?;
        let e_base = t.make_edge(v_base, v_base, Some(CurveCarrier { curve: base_c, range: Interval::new(0.0, TAU), sense: Sense::Forward }))?;
        let slant = vb - vtop;
        let seam_c = add_curve(t, CurveDef::Line(Line::new(at(vb), at(vtop) - at(vb))?))?;
        // Line parameter is arc length only for unit directions; Line::new normalises the direction.
        let e_seam = t.make_edge(v_base, v_top, Some(CurveCarrier { curve: seam_c, range: Interval::new(0.0, slant), sense: Sense::Forward }))?;
        let (e_top, top_is_pole) = match vt {
            None => {
                let e = t.make_edge(v_top, v_top, None)?;
                t.set_flags(e, CellFlags::DEGENERATE)?;
                (e, true)
            }
            Some(v) => {
                let c = add_curve(t, CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(0.0, 0.0, v * co), z, x)?, v * s)?))?;
                (t.make_edge(v_top, v_top, Some(CurveCarrier { curve: c, range: Interval::new(0.0, TAU), sense: Sense::Forward }))?, false)
            }
        };
        let surf = add_surface(t, SurfaceDef::Cone(Cone::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 0.0, a)?))?;
        let lp = [fwd(e_base), fwd(e_seam), rev(e_top), rev(e_seam)];
        // The loop is clockwise in (u, v): the face normal is the reverse of the cone's.
        let f = t.make_face(&[&lp], Some(SurfaceCarrier { surface: surf, sense: Sense::Reversed }))?;
        let full = Interval::new(0.0, TAU);
        let key = |edge, side| CoedgeKey { face: f, edge, side };
        t.set_pcurve(key(e_base, Side::Left), Some(line2((0.0, vb), (1.0, 0.0), full)))?;
        t.set_pcurve(key(e_seam, Side::Left), Some(line2((TAU, vb), (0.0, -1.0), Interval::new(0.0, slant))))?;
        t.set_pcurve(key(e_top, Side::Right), Some(line2((0.0, vtop), (1.0, 0.0), full)))?;
        t.set_pcurve(key(e_seam, Side::Right), Some(line2((0.0, vb), (0.0, -1.0), Interval::new(0.0, slant))))?;
        Ok((f, [Some(e_base), (!top_is_pole).then_some(e_top), Some(e_seam)]))
    })
    .pipe(|(c, (f, e))| (c, f, e))
}

#[test]
fn cone_with_apex_collapses_the_pole_row() {
    let (a, vb) = (0.5, 4.0);
    let (c, f, [base, _, _]) = cone_sheet(a, vb, None);
    let t = tessellate(&c, &[f], &opts(1e-3)).unwrap();
    let n = t.edge_polyline(base.unwrap()).unwrap().len() - 1;
    // n triangles (apex, p_i, p_i+1) and nothing else: the pole row is a single vertex.
    assert_eq!(t.triangle_count(), n);
    assert_eq!(t.vertex_count(), 2 + (n - 1));
    let r = vb * a.sin();
    let slant = vb;
    let exact = PI * r * slant;
    let theta = TAU / n as f64;
    let area = face_area(&t, f);
    assert!(area <= exact * (1.0 + 1e-5) && area >= exact * theta.sin() / theta * 0.999, "{area} vs {exact}");
    check_face_structure(&c, &t, f);
    check_normal_signs(&c, &t, f);
}

#[test]
fn frustum_with_unequal_rings_is_merge_triangulated() {
    let (a, vb, vt) = (0.4, 6.0, 1.5);
    let (c, f, [base, top, _]) = cone_sheet(a, vb, Some(vt));
    let o = opts(2e-3);
    let t = tessellate(&c, &[f], &o).unwrap();
    let nb = t.edge_polyline(base.unwrap()).unwrap().len() - 1;
    let nt = t.edge_polyline(top.unwrap()).unwrap().len() - 1;
    assert!(nb > nt, "the larger ring gets more segments ({nb} vs {nt})");
    // Strip between two rings with nb and nt segments: nb + nt triangles.
    assert_eq!(t.triangle_count(), nb + nt);
    let (rb, rt) = (vb * a.sin(), vt * a.sin());
    let exact = PI * (rb + rt) * (vb - vt);
    let theta_max = TAU / nt as f64;
    let area = face_area(&t, f);
    assert!(area <= exact * (1.0 + 1e-5) && area >= exact * theta_max.cos(), "{area} vs {exact}");
    check_face_structure(&c, &t, f);
    check_normal_signs(&c, &t, f);
}

#[test]
fn torus_face_with_two_seams_gets_interior_rows() {
    let (rr, r) = (3.0, 1.0);
    let (c, (f, eu, ev)) = build(|t| {
        let v = t.make_vertex(Point3::new(rr + r, 0.0, 0.0))?;
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let cu = add_curve(t, CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(0.0, 0.0, 0.0), z, x)?, rr + r)?))?;
        let cv = add_curve(t, CurveDef::Circle(Circle::new(Frame::from_zx(Point3::new(rr, 0.0, 0.0), Vector3::new(0.0, -1.0, 0.0), x)?, r)?))?;
        let full = Interval::new(0.0, TAU);
        let eu = t.make_edge(v, v, Some(CurveCarrier { curve: cu, range: full, sense: Sense::Forward }))?;
        let ev = t.make_edge(v, v, Some(CurveCarrier { curve: cv, range: full, sense: Sense::Forward }))?;
        let surf = add_surface(t, SurfaceDef::Torus(Torus::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), rr, r)?))?;
        let f = t.make_face(&[&[fwd(eu), fwd(ev), rev(eu), rev(ev)]], Some(SurfaceCarrier { surface: surf, sense: Sense::Forward }))?;
        let key = |edge, side| CoedgeKey { face: f, edge, side };
        t.set_pcurve(key(eu, Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
        t.set_pcurve(key(ev, Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), full)))?;
        t.set_pcurve(key(eu, Side::Right), Some(line2((0.0, TAU), (1.0, 0.0), full)))?;
        t.set_pcurve(key(ev, Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), full)))?;
        Ok((f, eu, ev))
    });
    let t = tessellate(&c, &[f], &opts(5e-3)).unwrap();
    let exact = 4.0 * PI * PI * rr * r;
    let area = face_area(&t, f);
    // Grid of nu x nv cells, 2 triangles each, with per-direction steps from the circle rule.
    let (nu, nv) = (t.edge_polyline(eu).unwrap().len() - 1, t.edge_polyline(ev).unwrap().len() - 1);
    assert_eq!(t.triangle_count(), 2 * nu * nv);
    let theta = (TAU / nu as f64).max(TAU / nv as f64);
    assert!((area - exact).abs() / exact < theta * theta, "area {area} vs {exact}");
    check_face_structure(&c, &t, f);
    check_normal_signs(&c, &t, f);
}
