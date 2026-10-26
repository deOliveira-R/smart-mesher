//! Topology fixtures for sweeping (extruded, twisted and revolved solids with a source and a
//! target face joined by linking faces). Each returns a [`SweepFixture`]: the [`TopoFixture`]
//! with its closed-form expectations plus the ids a sweep scheme needs.

use super::{TopoFixture, run};
use core::f64::consts::{PI, TAU};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, Cylinder, CurveDef, Frame, Interval, Line, Line2, NurbsSurface, PCurve, Plane, SurfaceDef, Uv};
use sm_topo_api::fixtures::FixtureError;
use sm_topo_api::{CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier, TopoError, TopologyEdit};

/// A sweepable solid: the fixture plus the ids of its source, target and linking faces.
#[derive(Clone, Debug)]
pub struct SweepFixture {
    /// The topology and its expectations (counts `[V, E, F, 1, 1]`, Euler, volume, area).
    pub topo: TopoFixture,
    /// The source face (at `z = 0` for prisms; the half-plane rectangle at angle 0 for the
    /// revolved sector).
    pub source: EntityId,
    /// The target face.
    pub target: EntityId,
    /// Linking faces, in the order documented by each fixture.
    pub linking: Vec<EntityId>,
    /// The edges that join source vertices to target vertices (polygon fixtures: in source loop
    /// order; the revolved sector: arcs; the hollow cylinder: seams). Documented per fixture.
    pub columns: Vec<EntityId>,
    /// The volume.
    pub volume: EntityId,
}

fn bad(msg: &'static str) -> FixtureError {
    FixtureError::Topo(TopoError::InvalidParameter(msg))
}

type Fwd = (EntityId, Sense);

fn lp(c: &[Fwd]) -> Vec<CoedgeSpec> {
    c.iter().map(|(edge, sense)| CoedgeSpec { edge: *edge, sense: *sense }).collect()
}

const F: Sense = Sense::Forward;
const R: Sense = Sense::Reversed;

fn line_edge(t: &mut CellComplex, a: (EntityId, Point3), b: (EntityId, Point3)) -> Result<EntityId, FixtureError> {
    let d = b.1 - a.1;
    let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(a.1, d)?))?;
    Ok(t.make_edge(a.0, b.0, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: F }))?)
}

fn plane(t: &mut CellComplex, origin: Point3, normal: Vector3) -> Result<EntityId, FixtureError> {
    Ok(t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame: Frame::from_z(origin, normal)? }))?)
}

fn face(t: &mut CellComplex, loops: &[&[CoedgeSpec]], surface: EntityId, sense: Sense) -> Result<EntityId, FixtureError> {
    Ok(t.make_face(loops, Some(SurfaceCarrier { surface, sense }))?)
}

fn close(t: &mut CellComplex, faces: &[EntityId]) -> Result<(EntityId, EntityId), FixtureError> {
    let uses: Vec<FaceUseSpec> = faces.iter().map(|f| FaceUseSpec { face: *f, side: Side::Left }).collect();
    let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
    let body = t.make_body(&[volume])?;
    Ok((volume, body))
}

/// Gauss-Legendre nodes and weights on `[0, 1]`.
fn gauss01(n: usize) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let mut x = (PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                    p0 = p1;
                    p1 = p2;
                }
                dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
                let dx = p1 / dp;
                x -= dx;
                if dx.abs() < 1e-16 {
                    break;
                }
            }
            (0.5 * (1.0 - x), 1.0 / ((1.0 - x * x) * dp * dp))
        })
        .collect()
}

/// Area of the bilinear patch through `a_b, b_b` (bottom) and `a_t, b_t` (top).
fn bilinear_area(a_b: Point3, b_b: Point3, a_t: Point3, b_t: Point3) -> f64 {
    let g = gauss01(16);
    let mut s = 0.0;
    for (u, wu) in &g {
        for (t, wt) in &g {
            let pu = (b_b - a_b) * (1.0 - t) + (b_t - a_t) * *t;
            let pt = (a_t - a_b) * (1.0 - u) + (b_t - b_b) * *u;
            s += wu * wt * pu.cross(pt).norm();
        }
    }
    s
}

/// Prism-like solid over matching bottom and top polygons (counter-clockwise seen from `+z`,
/// bottom in `z = 0`... any height): `n + 2` faces, `2 n` vertices, `3 n` edges. Linking face `i`
/// joins bottom edge `i` and top edge `i`; it is planar, or an exact bilinear patch when
/// `bilinear`. Column `i` runs from bottom vertex `i` to top vertex `i`.
fn polygon_sweep(name: &'static str, bottom: &[Point3], top: &[Point3], bilinear: bool, volume: f64, area: f64) -> Result<SweepFixture, FixtureError> {
    let n = bottom.len();
    let mut t = CellComplex::new();
    let (source, target, linking, columns, vol) = run(&mut t, 1, |t| {
        let vb: Vec<EntityId> = bottom.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
        let vt: Vec<EntityId> = top.iter().map(|p| t.make_vertex(*p)).collect::<Result<_, _>>()?;
        let mut eb = Vec::new();
        let mut et = Vec::new();
        let mut ec = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            eb.push(line_edge(t, (vb[i], bottom[i]), (vb[j], bottom[j]))?);
            et.push(line_edge(t, (vt[i], top[i]), (vt[j], top[j]))?);
            ec.push(line_edge(t, (vb[i], bottom[i]), (vt[i], top[i]))?);
        }
        let src_loop = lp(&(0..n).rev().map(|i| (eb[i], R)).collect::<Vec<_>>());
        let tgt_loop = lp(&(0..n).map(|i| (et[i], F)).collect::<Vec<_>>());
        let s_src = plane(t, bottom[0], Vector3::new(0.0, 0.0, -1.0))?;
        let s_tgt = plane(t, top[0], Vector3::new(0.0, 0.0, 1.0))?;
        let source = face(t, &[&src_loop], s_src, F)?;
        let target = face(t, &[&tgt_loop], s_tgt, F)?;
        let mut linking = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let l = lp(&[(eb[i], F), (ec[j], F), (et[i], R), (ec[i], R)]);
            let surf = if bilinear {
                // u along the bottom edge, v up; control points are u-major.
                let ctrl = vec![bottom[i], top[i], bottom[j], top[j]];
                let s = NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, ctrl, None)?;
                t.shapes_mut()?.add_surface(SurfaceDef::Nurbs(s))?
            } else {
                let d = bottom[j] - bottom[i];
                plane(t, bottom[i], Vector3::new(d.y, -d.x, 0.0))?
            };
            linking.push(face(t, &[&l], surf, F)?);
        }
        let mut all = vec![source, target];
        all.extend(&linking);
        let (vol, _) = close(t, &all)?;
        Ok((source, target, linking, ec, vol))
    })?;
    let topo = TopoFixture { name, complex: t, counts: [2 * n, 3 * n, n + 2, 1, 1], euler: 2, volumes: vec![volume], area };
    Ok(SweepFixture { topo, source, target, linking, columns, volume: vol })
}

fn lifted(poly: &[(f64, f64)], z: f64) -> Vec<Point3> {
    poly.iter().map(|p| Point3::new(p.0, p.1, z)).collect()
}

/// The L-shaped prism `[0, a]^2 \ [t, a]^2` extruded to `z = h` (needs `0 < t < a`).
///
/// Source area `a^2 - (a - t)^2`, volume `h` times that (`extruded_l(2, 1, 2)`: area 3, volume
/// 6), total face area `2 (a^2 - (a-t)^2) + 4 a h`. 8 faces, 12 vertices, 18 edges, Euler 2.
/// The six linking faces are planar quadrilaterals in loop order from the bottom edge
/// `(0,0)-(a,0)`; the column at the re-entrant vertex `(t, t)` (index 3) is one straight edge
/// shared by two linking faces. Columns are in source loop order.
pub fn extruded_l(a: f64, t: f64, h: f64) -> Result<SweepFixture, FixtureError> {
    if !(0.0 < t && t < a && h > 0.0) {
        return Err(bad("extruded_l needs 0 < t < a and h > 0"));
    }
    let poly = [(0.0, 0.0), (a, 0.0), (a, t), (t, t), (t, a), (0.0, a)];
    let base = a * a - (a - t) * (a - t);
    polygon_sweep("extruded_l", &lifted(&poly, 0.0), &lifted(&poly, h), false, base * h, 2.0 * base + 4.0 * a * h)
}

/// A regular `n`-gon prism of circumradius `r` and height `h` (`n >= 3`), first vertex at
/// `(r, 0)`, counter-clockwise: `n + 2` faces, base area `(n/2) r^2 sin(2 pi / n)`, volume
/// `h` times that (`regular_prism(6, 1, 3)`: 7.794228634059948), side faces of width
/// `2 r sin(pi / n)`. Source and target are the `n`-gons; linking faces follow the edges.
pub fn regular_prism(n: usize, r: f64, h: f64) -> Result<SweepFixture, FixtureError> {
    if n < 3 || !(r > 0.0 && h > 0.0) {
        return Err(bad("regular_prism needs n >= 3, r > 0, h > 0"));
    }
    let poly: Vec<(f64, f64)> = (0..n).map(|i| (r * (TAU * i as f64 / n as f64).cos(), r * (TAU * i as f64 / n as f64).sin())).collect();
    let base = 0.5 * n as f64 * r * r * (TAU / n as f64).sin();
    let side = 2.0 * r * (PI / n as f64).sin();
    polygon_sweep("regular_prism", &lifted(&poly, 0.0), &lifted(&poly, h), false, base * h, 2.0 * base + n as f64 * side * h)
}

/// A twisted square prism: the square of side `side` centred on the `z` axis at `z = 0`, and its
/// rotation by `twist` (radians, about `z`) at `z = h`. The four linking faces are exact
/// bilinear `NurbsSurface` patches `(1-t)((1-u)A + uB) + t((1-u)A' + uB')`; column edges are
/// straight chords.
///
/// Volume `side^2 h (2 + cos twist) / 3` (the cross-section at height fraction `s` is a square of
/// area `side^2 |(1-s) + s e^{i twist}|^2`); `twisted_prism(2, 3, pi/2)` has volume 8. The area is
/// the two squares plus four patches integrated by 16-point Gauss quadrature (exact to
/// rounding for the smooth integrand).
pub fn twisted_prism(side: f64, h: f64, twist: f64) -> Result<SweepFixture, FixtureError> {
    if !(side > 0.0 && h > 0.0 && twist.is_finite()) {
        return Err(bad("twisted_prism needs side > 0, h > 0 and a finite twist"));
    }
    let q = 0.5 * side;
    let sq = [(-q, -q), (q, -q), (q, q), (-q, q)];
    let (c, s) = (twist.cos(), twist.sin());
    let top: Vec<Point3> = sq.iter().map(|p| Point3::new(c * p.0 - s * p.1, s * p.0 + c * p.1, h)).collect();
    let bottom = lifted(&sq, 0.0);
    let patches: f64 = (0..4).map(|i| bilinear_area(bottom[i], bottom[(i + 1) % 4], top[i], top[(i + 1) % 4])).sum();
    let volume = side * side * h * (2.0 + twist.cos()) / 3.0;
    polygon_sweep("twisted_prism", &bottom, &top, true, volume, 2.0 * side * side + patches)
}

/// The revolved rectangle `[r_in, r_out] x [0, h]` of the `xz` half-plane (angle 0) swept about
/// `z` by `sweep` radians (`0 < sweep < 2 pi`; target = `Rz(sweep)` source). 8 vertices, 12 edges,
/// 6 faces. Linking faces in the order `[inner cylinder, outer cylinder, bottom, top]`: the
/// cylinders (`Cylinder` carriers; the inner one has a `Reversed` carrier so its normal points
/// out of the solid) and the planar annular sectors, each 4-sided with two `Circle` arcs.
/// Columns are the four arcs `[bottom inner, bottom outer, top inner, top outer]`.
///
/// Volume `sweep/2 (r_out^2 - r_in^2) h`; area is two rectangles plus both cylinders plus both
/// sectors.
pub fn annular_sector(r_in: f64, r_out: f64, h: f64, sweep: f64) -> Result<SweepFixture, FixtureError> {
    if !(0.0 < r_in && r_in < r_out && h > 0.0 && 0.0 < sweep && sweep < TAU) {
        return Err(bad("annular_sector needs 0 < r_in < r_out, h > 0, 0 < sweep < 2 pi"));
    }
    let pt = |a: f64, r: f64, z: f64| Point3::new(r * a.cos(), r * a.sin(), z);
    let ang = [0.0, sweep];
    let mut t = CellComplex::new();
    let (source, target, linking, columns, vol) = run(&mut t, 1, |t| {
        let radii = [r_in, r_out];
        let mut vs = Vec::new();
        for a in ang {
            for zi in 0..2 {
                for r in radii {
                    vs.push(t.make_vertex(pt(a, r, zi as f64 * h))?);
                }
            }
        }
        let v = |ai: usize, zi: usize, ri: usize| vs[ai * 4 + zi * 2 + ri];
        let p = |ai: usize, zi: usize, ri: usize| pt(ang[ai], radii[ri], zi as f64 * h);
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let mut arcs = Vec::new();
        for zi in 0..2 {
            for r in radii {
                let frame = Frame::from_zx(Point3::new(0.0, 0.0, zi as f64 * h), z, x)?;
                let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
                arcs.push(t.make_edge(v(0, zi, arcs.len() % 2), v(1, zi, arcs.len() % 2), Some(CurveCarrier { curve, range: Interval::new(0.0, sweep), sense: F }))?);
            }
        }
        let arc = |zi: usize, ri: usize| arcs[zi * 2 + ri];
        let mut radials = Vec::new();
        let mut cols = Vec::new();
        for ai in 0..2 {
            for zi in 0..2 {
                radials.push(line_edge(t, (v(ai, zi, 0), p(ai, zi, 0)), (v(ai, zi, 1), p(ai, zi, 1)))?);
            }
            for ri in 0..2 {
                cols.push(line_edge(t, (v(ai, 0, ri), p(ai, 0, ri)), (v(ai, 1, ri), p(ai, 1, ri)))?);
            }
        }
        let radial = |ai: usize, zi: usize| radials[ai * 2 + zi];
        let col = |ai: usize, ri: usize| cols[ai * 2 + ri];
        let s_src = plane(t, p(0, 0, 0), Vector3::new(0.0, -1.0, 0.0))?;
        let s_tgt = plane(t, p(1, 0, 0), Vector3::new(-sweep.sin(), sweep.cos(), 0.0))?;
        let s_bot = plane(t, p(0, 0, 0), Vector3::new(0.0, 0.0, -1.0))?;
        let s_top = plane(t, p(0, 1, 0), z)?;
        let cyl = |t: &mut CellComplex, r: f64| -> Result<EntityId, FixtureError> { Ok(t.shapes_mut()?.add_surface(SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?) };
        let (s_in, s_out) = (cyl(t, r_in)?, cyl(t, r_out)?);
        let source = face(t, &[&lp(&[(radial(0, 0), F), (col(0, 1), F), (radial(0, 1), R), (col(0, 0), R)])], s_src, F)?;
        let target = face(t, &[&lp(&[(col(1, 0), F), (radial(1, 1), F), (col(1, 1), R), (radial(1, 0), R)])], s_tgt, F)?;
        let inner = face(t, &[&lp(&[(col(0, 0), F), (arc(1, 0), F), (col(1, 0), R), (arc(0, 0), R)])], s_in, R)?;
        let outer = face(t, &[&lp(&[(arc(0, 1), F), (col(1, 1), F), (arc(1, 1), R), (col(0, 1), R)])], s_out, F)?;
        let bottom = face(t, &[&lp(&[(arc(0, 0), F), (radial(1, 0), F), (arc(0, 1), R), (radial(0, 0), R)])], s_bot, F)?;
        let top = face(t, &[&lp(&[(radial(0, 1), F), (arc(1, 1), F), (radial(1, 1), R), (arc(1, 0), R)])], s_top, F)?;
        let (vol, _) = close(t, &[source, target, inner, outer, bottom, top])?;
        Ok((source, target, vec![inner, outer, bottom, top], vec![arc(0, 0), arc(0, 1), arc(1, 0), arc(1, 1)], vol))
    })?;
    let rect = (r_out - r_in) * h;
    let sector = 0.5 * sweep * (r_out * r_out - r_in * r_in);
    let area = 2.0 * rect + sweep * (r_in + r_out) * h + 2.0 * sector;
    let topo = TopoFixture { name: "annular_sector", complex: t, counts: [8, 12, 6, 1, 1], euler: 2, volumes: vec![sector * h], area };
    Ok(SweepFixture { topo, source, target, linking, columns, volume: vol })
}

/// The hollow cylinder `r_in <= r <= r_out`, `0 <= z <= h` about `z`. Source (bottom) and target
/// (top) are planar annuli with two loops, one circle edge and one vertex per loop. The linking
/// faces `[outer, inner]` are `Cylinder` faces whose seam edge is used twice; the inner one has a
/// `Reversed` carrier. 4 vertices, 6 edges (4 circles, 2 seams), 4 faces, Euler 2. Columns are
/// the seams `[inner, outer]`.
///
/// Volume `pi (r_out^2 - r_in^2) h` (`hollow_cylinder(1, 2, 2)`: `6 pi`), area
/// `2 pi (r_out^2 - r_in^2) + 2 pi (r_in + r_out) h`.
pub fn hollow_cylinder(r_in: f64, r_out: f64, h: f64) -> Result<SweepFixture, FixtureError> {
    if !(0.0 < r_in && r_in < r_out && h > 0.0) {
        return Err(bad("hollow_cylinder needs 0 < r_in < r_out and h > 0"));
    }
    let mut t = CellComplex::new();
    let (source, target, linking, columns, vol) = run(&mut t, 1, |t| {
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let radii = [r_in, r_out];
        let full = Interval::new(0.0, TAU);
        let mut vs = Vec::new();
        let mut cs = Vec::new();
        for zi in 0..2 {
            for r in radii {
                let v = t.make_vertex(Point3::new(r, 0.0, zi as f64 * h))?;
                let frame = Frame::from_zx(Point3::new(0.0, 0.0, zi as f64 * h), z, x)?;
                let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
                cs.push(t.make_edge(v, v, Some(CurveCarrier { curve, range: full, sense: F }))?);
                vs.push(v);
            }
        }
        let c = |zi: usize, ri: usize| cs[zi * 2 + ri];
        let mut seam = Vec::new();
        for (ri, r) in radii.iter().enumerate() {
            seam.push(line_edge(t, (vs[ri], Point3::new(*r, 0.0, 0.0)), (vs[2 + ri], Point3::new(*r, 0.0, h)))?);
        }
        let s_bot = plane(t, Point3::new(0.0, 0.0, 0.0), -z)?;
        let s_top = plane(t, Point3::new(0.0, 0.0, h), z)?;
        let cyl = |t: &mut CellComplex, r: f64| -> Result<EntityId, FixtureError> { Ok(t.shapes_mut()?.add_surface(SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), r)?))?) };
        let (s_in, s_out) = (cyl(t, r_in)?, cyl(t, r_out)?);
        let source = face(t, &[&lp(&[(c(0, 1), R)]), &lp(&[(c(0, 0), F)])], s_bot, F)?;
        let target = face(t, &[&lp(&[(c(1, 1), F)]), &lp(&[(c(1, 0), R)])], s_top, F)?;
        let outer = face(t, &[&lp(&[(c(0, 1), F), (seam[1], F), (c(1, 1), R), (seam[1], R)])], s_out, F)?;
        let inner = face(t, &[&lp(&[(seam[0], F), (c(1, 0), F), (seam[0], R), (c(0, 0), R)])], s_in, R)?;
        let line2 = |o: (f64, f64), d: (f64, f64), range| PCurve::Line2(Line2 { origin: Uv::new(o.0, o.1), dir: Uv::new(d.0, d.1), range });
        let hr = Interval::new(0.0, h);
        let key = |face, edge, side| CoedgeKey { face, edge, side };
        t.set_pcurve(key(outer, c(0, 1), Side::Left), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
        t.set_pcurve(key(outer, seam[1], Side::Left), Some(line2((TAU, 0.0), (0.0, 1.0), hr)))?;
        t.set_pcurve(key(outer, c(1, 1), Side::Right), Some(line2((0.0, h), (1.0, 0.0), full)))?;
        t.set_pcurve(key(outer, seam[1], Side::Right), Some(line2((0.0, 0.0), (0.0, 1.0), hr)))?;
        t.set_pcurve(key(inner, seam[0], Side::Left), Some(line2((0.0, 0.0), (0.0, 1.0), hr)))?;
        t.set_pcurve(key(inner, c(1, 0), Side::Left), Some(line2((0.0, h), (1.0, 0.0), full)))?;
        t.set_pcurve(key(inner, seam[0], Side::Right), Some(line2((TAU, 0.0), (0.0, 1.0), hr)))?;
        t.set_pcurve(key(inner, c(0, 0), Side::Right), Some(line2((0.0, 0.0), (1.0, 0.0), full)))?;
        let (vol, _) = close(t, &[source, target, outer, inner])?;
        Ok((source, target, vec![outer, inner], vec![seam[0], seam[1]], vol))
    })?;
    let ring = PI * (r_out * r_out - r_in * r_in);
    let area = 2.0 * ring + 2.0 * PI * (r_in + r_out) * h;
    let topo = TopoFixture { name: "hollow_cylinder", complex: t, counts: [4, 6, 4, 1, 1], euler: 2, volumes: vec![ring * h], area };
    Ok(SweepFixture { topo, source, target, linking, columns, volume: vol })
}
