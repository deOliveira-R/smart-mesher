//! `model.create_sphere_part`: a sphere (or hollow sphere) cut by coordinate planes through its
//! centre into a hemisphere, quarter or octant.
//!
//! The kept spherical patch is a `(u, v)` rectangle of the standard parametrization: `u` in
//! `[0, 2 pi]`, `[0, pi]` or `[0, pi / 2]` (the sphere frame's x axis is chosen so that `u` starts
//! at the patch edge) and `v` in `[-pi/2, pi/2]`, `[0, pi/2]` or `[-pi/2, 0]`. Pole sides are
//! `DEGENERATE` edges; a seam exists only when `u` is full. Caps are `Plane` faces that share the
//! meridians, the equator arcs and the axis or diameter edges with the patch and each other.
//!
//! Cell counts `[V, E incl. degenerate, F]` without `inner`: whole `[2,3,1]`; z cut `[2,3,2]`;
//! x or y `[2,4,2]`; xy `[2,5,3]`; xz or yz `[3,5,3]`; xyz `[4,7,4]`. With `inner` every
//! ring (outer, inner) contributes its own vertices and meridians; a whole hollow sphere is one
//! volume with an `Outer` and a `Void` shell (`[4,6,2]`), any cut gives one `Outer` shell whose
//! caps are annular. The inner patch is the same sphere with a `Reversed` carrier, so its normal
//! points into the cavity, out of the solid.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, Interval, Line, Plane, Sphere, SurfaceDef};
use sm_layer_api::{LayerError, TopoHost};
use sm_op_api::{ApplyCtx, OpError, OutputRole, TypedOp};
use sm_topo_api::{CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier};

use crate::prim::{Built, SOLID_OUTPUTS, build_sphere, create, curve, finite3, line2, positive, span, surface};

const F: Sense = Sense::Forward;
const R: Sense = Sense::Reversed;

/// Which side of a coordinate plane through the centre is kept.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum HalfSpace {
    /// The side with coordinate `>= 0`.
    Positive,
    /// The side with coordinate `<= 0`.
    Negative,
}

/// The coordinate planes that cut a sphere part: each given axis keeps one half-space.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Octants {
    /// Keep `x >= 0` or `x <= 0`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x: Option<HalfSpace>,
    /// Keep `y >= 0` or `y <= 0`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub y: Option<HalfSpace>,
    /// Keep `z >= 0` or `z <= 0`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub z: Option<HalfSpace>,
}

/// `model.create_sphere_part`: the general form of `model.create_sphere`.
///
/// `inner` in `(0, radius)` hollows the sphere out (a cavity of that radius). `octants` keeps the
/// half-space on the given side of each listed coordinate plane through `center`: one cut keeps a
/// hemisphere, two a quarter, three an octant. With neither option the cells are exactly those of
/// `model.create_sphere`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateSpherePart {
    /// Outer radius.
    pub radius: f64,
    /// Cavity radius, `0 < inner < radius`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub inner: Option<f64>,
    /// Coordinate-plane cuts.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub octants: Option<Octants>,
    /// Centre of the sphere (default the origin).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub center: Option<[f64; 3]>,
}

impl TypedOp for CreateSpherePart {
    const NAME: &'static str = "model.create_sphere_part";
    const OUTPUTS: &'static [OutputRole] = SOLID_OUTPUTS;
    const POSITIONAL: Option<&'static str> = Some("radius");

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let r = positive("radius", self.radius)?;
        let inner = match self.inner {
            None => None,
            Some(i) => {
                let i = positive("inner", i)?;
                if i >= r {
                    return Err(OpError::Invalid(format!("`inner` {i} must be below `radius` {r}")));
                }
                Some(i)
            }
        };
        let c = finite3("center", self.center.unwrap_or([0.0; 3]))?;
        if !(2.0 * r).is_finite() {
            return Err(OpError::Invalid(format!("`radius` {r:e} is too large: a diameter overflows")));
        }
        span("radius", c.x - r, c.x + r)?;
        span("radius", c.y - r, c.y + r)?;
        span("radius", c.z - r, c.z + r)?;
        let cuts = self.octants.unwrap_or_default();
        let mut smallest = 2.0 * r;
        if let Some(i) = inner {
            smallest = smallest.min(2.0 * i).min(r - i);
        }
        let plan = Plan::from_cuts(&cuts);
        create(ctx, smallest, |topo| {
            if inner.is_none() && plan.is_whole() {
                build_sphere(topo, c, r)
            } else {
                build_part(topo, c, r, inner, &plan)
            }
        })
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Du {
    Full,
    Half,
    Quarter,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Zm {
    Full,
    Up,
    Low,
}

/// The patch layout derived from the cuts: where `u` starts, how far it runs, the z range.
struct Plan {
    e0: Vector3,
    du: Du,
    zm: Zm,
}

impl Plan {
    fn from_cuts(c: &Octants) -> Plan {
        let v = |x: f64, y: f64| Vector3::new(x, y, 0.0);
        use HalfSpace::{Negative as N, Positive as P};
        let (e0, du) = match (c.x, c.y) {
            (None, None) => (v(1.0, 0.0), Du::Full),
            (Some(P), None) => (v(0.0, -1.0), Du::Half),
            (Some(N), None) => (v(0.0, 1.0), Du::Half),
            (None, Some(P)) => (v(1.0, 0.0), Du::Half),
            (None, Some(N)) => (v(-1.0, 0.0), Du::Half),
            (Some(P), Some(P)) => (v(1.0, 0.0), Du::Quarter),
            (Some(N), Some(P)) => (v(0.0, 1.0), Du::Quarter),
            (Some(N), Some(N)) => (v(-1.0, 0.0), Du::Quarter),
            (Some(P), Some(N)) => (v(0.0, -1.0), Du::Quarter),
        };
        let zm = match c.z {
            None => Zm::Full,
            Some(P) => Zm::Up,
            Some(N) => Zm::Low,
        };
        Plan { e0, du, zm }
    }

    fn is_whole(&self) -> bool {
        self.du == Du::Full && self.zm == Zm::Full
    }
}

/// The cells of one radius.
struct Ring {
    rho: f64,
    vn: Option<EntityId>,
    vs: Option<EntityId>,
    /// Equator vertices at `u = 0` and `u = delta`.
    eq: [Option<EntityId>; 2],
    /// Meridians at `u = 0` and `u = delta` (the second only for a partial `u`).
    m: [Option<EntityId>; 2],
    /// Equator arc or circle.
    q: Option<EntityId>,
    pn: Option<EntityId>,
    ps: Option<EntityId>,
}

fn pt(c: Point3, dir: Vector3, d: f64) -> Point3 {
    c + dir * d
}

fn build_part(topo: &mut dyn TopoHost, c: Point3, outer: f64, inner: Option<f64>, plan: &Plan) -> Result<Built, LayerError> {
    let z = Vector3::new(0.0, 0.0, 1.0);
    let (e0, du, zm) = (plan.e0, plan.du, plan.zm);
    let e1 = match du {
        Du::Full => e0,
        Du::Half => -e0,
        Du::Quarter => z.cross(e0),
    };
    let delta = match du {
        Du::Full => TAU,
        Du::Half => PI,
        Du::Quarter => FRAC_PI_2,
    };
    let n0 = z.cross(e0);
    let n1 = z.cross(e1);
    let partial = du != Du::Full;
    let (v0, v1) = match zm {
        Zm::Full => (-FRAC_PI_2, FRAC_PI_2),
        Zm::Up => (0.0, FRAC_PI_2),
        Zm::Low => (-FRAC_PI_2, 0.0),
    };
    let radii: Vec<f64> = std::iter::once(outer).chain(inner).collect();
    let hollow = inner.is_some();
    let dirs = [e0, e1];
    let normals = [n0, n1];
    let frame_u = Frame::from_zx(c, z, e0)?;
    let mut all_vertices: Vec<EntityId> = Vec::new();
    let mut all_edges: Vec<EntityId> = Vec::new();
    let mut rings: Vec<Ring> = Vec::new();
    for &rho in &radii {
        let mut ring = Ring { rho, vn: None, vs: None, eq: [None; 2], m: [None; 2], q: None, pn: None, ps: None };
        if zm != Zm::Low {
            ring.vn = Some(topo.make_vertex(pt(c, z, rho))?);
        }
        if zm != Zm::Up {
            ring.vs = Some(topo.make_vertex(pt(c, -z, rho))?);
        }
        if zm != Zm::Full {
            ring.eq[0] = Some(topo.make_vertex(pt(c, e0, rho))?);
            if partial {
                ring.eq[1] = Some(topo.make_vertex(pt(c, e1, rho))?);
            }
        }
        for k in 0..if partial { 2 } else { 1 } {
            let lo = if zm == Zm::Up { ring.eq[k] } else { ring.vs };
            let hi = if zm == Zm::Low { ring.eq[k] } else { ring.vn };
            let (Some(lo), Some(hi)) = (lo, hi) else { return Err(LayerError::from(sm_topo_api::TopoError::InvalidParameter("sphere part: missing meridian end"))) };
            let frame = Frame::from_zx(c, -normals[k], dirs[k])?;
            let cv = curve(topo, CurveDef::Circle(Circle::new(frame, rho)?))?;
            ring.m[k] = Some(topo.make_edge(lo, hi, Some(CurveCarrier { curve: cv, range: Interval::new(v0, v1), sense: F }))?);
        }
        if zm != Zm::Full {
            let cv = curve(topo, CurveDef::Circle(Circle::new(frame_u, rho)?))?;
            let end = if partial { ring.eq[1] } else { ring.eq[0] };
            let (Some(a), Some(b)) = (ring.eq[0], end) else { return Err(LayerError::from(sm_topo_api::TopoError::InvalidParameter("sphere part: missing equator end"))) };
            ring.q = Some(topo.make_edge(a, b, Some(CurveCarrier { curve: cv, range: Interval::new(0.0, delta), sense: F }))?);
        }
        if let Some(vn) = ring.vn {
            let e = topo.make_edge(vn, vn, None)?;
            topo.set_flags(e, CellFlags::DEGENERATE)?;
            ring.pn = Some(e);
        }
        if let Some(vs) = ring.vs {
            let e = topo.make_edge(vs, vs, None)?;
            topo.set_flags(e, CellFlags::DEGENERATE)?;
            ring.ps = Some(e);
        }
        all_vertices.extend([ring.vn, ring.vs, ring.eq[0], ring.eq[1]].into_iter().flatten());
        all_edges.extend(ring.m.iter().flatten().chain(ring.q.iter()).chain(ring.pn.iter()).chain(ring.ps.iter()));
        rings.push(ring);
    }
    let big = &rings[0];
    let small = rings.get(1);
    let unwrap_e = |e: Option<EntityId>| e.ok_or(LayerError::from(sm_topo_api::TopoError::InvalidParameter("sphere part: missing edge")));

    // Radial segments of the z cap (partial u, z cut) at `u = 0` and `u = delta`.
    let mut rad: [Option<EntityId>; 2] = [None; 2];
    let mut origin_v: Option<EntityId> = None;
    let mut diameter: Option<EntityId> = None;
    if zm != Zm::Full && partial {
        match small {
            Some(s) => {
                for k in 0..2 {
                    let (a, b) = (unwrap_e(s.eq[k])?, unwrap_e(big.eq[k])?);
                    let e = line_edge(topo, (a, pt(c, dirs[k], s.rho)), (b, pt(c, dirs[k], big.rho)))?;
                    all_edges.push(e);
                    rad[k] = Some(e);
                }
            }
            None if du == Du::Half => {
                let (a, b) = (unwrap_e(big.eq[1])?, unwrap_e(big.eq[0])?);
                let e = line_edge(topo, (a, pt(c, e1, big.rho)), (b, pt(c, e0, big.rho)))?;
                all_edges.push(e);
                diameter = Some(e);
            }
            None => {
                let o = topo.make_vertex(c)?;
                all_vertices.push(o);
                origin_v = Some(o);
                for k in 0..2 {
                    let e = line_edge(topo, (o, c), (unwrap_e(big.eq[k])?, pt(c, dirs[k], big.rho)))?;
                    all_edges.push(e);
                    rad[k] = Some(e);
                }
            }
        }
    }
    // Axis edges (quarter only: the two end faces meet along the axis).
    let mut axis_n: Option<EntityId> = None;
    let mut axis_s: Option<EntityId> = None;
    let mut axis: Option<EntityId> = None;
    if du == Du::Quarter {
        match small {
            Some(s) => {
                if zm != Zm::Low {
                    let e = line_edge(topo, (unwrap_e(s.vn)?, pt(c, z, s.rho)), (unwrap_e(big.vn)?, pt(c, z, big.rho)))?;
                    all_edges.push(e);
                    axis_n = Some(e);
                }
                if zm != Zm::Up {
                    let e = line_edge(topo, (unwrap_e(big.vs)?, pt(c, -z, big.rho)), (unwrap_e(s.vs)?, pt(c, -z, s.rho)))?;
                    all_edges.push(e);
                    axis_s = Some(e);
                }
            }
            None => {
                let (lo, lo_p) = if zm == Zm::Up { (unwrap_e(origin_v)?, c) } else { (unwrap_e(big.vs)?, pt(c, -z, big.rho)) };
                let (hi, hi_p) = if zm == Zm::Low { (unwrap_e(origin_v)?, c) } else { (unwrap_e(big.vn)?, pt(c, z, big.rho)) };
                let e = line_edge(topo, (lo, lo_p), (hi, hi_p))?;
                all_edges.push(e);
                axis = Some(e);
            }
        }
    }

    let mut faces: Vec<EntityId> = Vec::new();
    let mut patch_faces: Vec<EntityId> = Vec::new();
    // Spherical patches.
    for (idx, ring) in rings.iter().enumerate() {
        let reversed = idx == 1;
        let right = if partial { 1 } else { 0 };
        let bottom = if v0 == -FRAC_PI_2 { ring.ps } else { ring.q };
        let top = if v1 == FRAC_PI_2 { ring.pn } else { ring.q };
        let mut pieces: Vec<(EntityId, Sense, PcSpec)> = vec![
            (unwrap_e(bottom)?, F, PcSpec { origin: (0.0, v0), dir: (1.0, 0.0), range: Interval::new(0.0, delta) }),
            (unwrap_e(ring.m[right])?, F, PcSpec { origin: (delta, 0.0), dir: (0.0, 1.0), range: Interval::new(v0, v1) }),
            (unwrap_e(top)?, R, PcSpec { origin: (0.0, v1), dir: (1.0, 0.0), range: Interval::new(0.0, delta) }),
            (unwrap_e(ring.m[0])?, R, PcSpec { origin: (0.0, 0.0), dir: (0.0, 1.0), range: Interval::new(v0, v1) }),
        ];
        if reversed {
            pieces.reverse();
            for p in &mut pieces {
                p.1 = p.1.flip();
            }
        }
        let sphere = surface(topo, SurfaceDef::Sphere(Sphere::new(frame_u, ring.rho)?))?;
        let specs: Vec<CoedgeSpec> = pieces.iter().map(|p| CoedgeSpec { edge: p.0, sense: p.1 }).collect();
        let carrier = SurfaceCarrier { surface: sphere, sense: if reversed { R } else { F } };
        let face = topo.make_face(&[&specs], Some(carrier))?;
        for p in &pieces {
            let side = if p.1 == F { Side::Left } else { Side::Right };
            topo.set_pcurve(CoedgeKey { face, edge: p.0, side }, Some(line2(p.2.origin, p.2.dir, p.2.range)))?;
        }
        patch_faces.push(face);
    }
    faces.extend(&patch_faces);

    // Equatorial cap.
    if zm != Zm::Full {
        let nz = if zm == Zm::Up { -z } else { z };
        let mut loops: Vec<Vec<(EntityId, Sense)>> = Vec::new();
        let (q_big, q_small) = (unwrap_e(big.q)?, small.and_then(|s| s.q));
        if !partial {
            loops.push(vec![(q_big, F)]);
            if let Some(qs) = q_small {
                loops.push(vec![(qs, R)]);
            }
        } else {
            let mut lp = vec![(q_big, F)];
            if let Some(qs) = q_small {
                lp.extend([(unwrap_e(rad[1])?, R), (qs, R), (unwrap_e(rad[0])?, F)]);
            } else if let Some(d) = diameter {
                lp.push((d, F));
            } else {
                lp.extend([(unwrap_e(rad[1])?, R), (unwrap_e(rad[0])?, F)]);
            }
            loops.push(lp);
        }
        if zm == Zm::Up {
            for lp in &mut loops {
                reverse_loop(lp);
            }
        }
        let plane = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(c, nz, e0)? }))?;
        faces.push(planar_face(topo, &loops, plane)?);
    }

    // End faces.
    let m = |ring: &Ring, k: usize| unwrap_e(ring.m[k]);
    match du {
        Du::Full => {}
        Du::Quarter => {
            let end_loop = |k: usize| -> Result<Vec<(EntityId, Sense)>, LayerError> {
                let mut lp: Vec<(EntityId, Sense)> = Vec::new();
                match small {
                    None => match zm {
                        Zm::Full => lp.extend([(m(big, k)?, F), (unwrap_e(axis)?, R)]),
                        Zm::Up => lp.extend([(unwrap_e(rad[k])?, F), (m(big, k)?, F), (unwrap_e(axis)?, R)]),
                        Zm::Low => lp.extend([(unwrap_e(axis)?, R), (m(big, k)?, F), (unwrap_e(rad[k])?, R)]),
                    },
                    Some(s) => match zm {
                        Zm::Full => lp.extend([(m(big, k)?, F), (unwrap_e(axis_n)?, R), (m(s, k)?, R), (unwrap_e(axis_s)?, R)]),
                        Zm::Up => lp.extend([(m(big, k)?, F), (unwrap_e(axis_n)?, R), (m(s, k)?, R), (unwrap_e(rad[k])?, F)]),
                        Zm::Low => lp.extend([(unwrap_e(rad[k])?, R), (m(s, k)?, R), (unwrap_e(axis_s)?, R), (m(big, k)?, F)]),
                    },
                }
                Ok(lp)
            };
            let l0 = end_loop(0)?;
            let mut l1 = end_loop(1)?;
            reverse_loop(&mut l1);
            let p0 = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(c, -n0, e0)? }))?;
            let p1 = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(c, n1, e1)? }))?;
            faces.push(planar_face(topo, &[l0], p0)?);
            faces.push(planar_face(topo, &[l1], p1)?);
        }
        Du::Half => {
            let mut loops: Vec<Vec<(EntityId, Sense)>> = Vec::new();
            match small {
                None => {
                    let mut lp = vec![(m(big, 0)?, F), (m(big, 1)?, R)];
                    match zm {
                        Zm::Full => {}
                        Zm::Up => lp.push((unwrap_e(diameter)?, F)),
                        Zm::Low => lp = vec![(unwrap_e(diameter)?, R), (m(big, 1)?, R), (m(big, 0)?, F)],
                    }
                    loops.push(lp);
                }
                Some(s) => match zm {
                    Zm::Full => {
                        loops.push(vec![(m(big, 0)?, F), (m(big, 1)?, R)]);
                        loops.push(vec![(m(s, 1)?, F), (m(s, 0)?, R)]);
                    }
                    Zm::Up => loops.push(vec![(m(big, 0)?, F), (m(big, 1)?, R), (unwrap_e(rad[1])?, R), (m(s, 1)?, F), (m(s, 0)?, R), (unwrap_e(rad[0])?, F)]),
                    Zm::Low => loops.push(vec![(unwrap_e(rad[0])?, R), (m(s, 0)?, R), (m(s, 1)?, F), (unwrap_e(rad[1])?, F), (m(big, 1)?, R), (m(big, 0)?, F)]),
                },
            }
            let p = surface(topo, SurfaceDef::Plane(Plane { frame: Frame::from_zx(c, -n0, e0)? }))?;
            faces.push(planar_face(topo, &loops, p)?);
        }
    }

    let uses = |fs: &[EntityId]| -> Vec<FaceUseSpec> { fs.iter().map(|&face| FaceUseSpec { face, side: Side::Left }).collect() };
    let volume = if hollow && !partial && zm == Zm::Full {
        // The cavity sphere is the only member of the `Void` shell.
        let outer_uses = uses(&patch_faces[..1]);
        let void_uses = uses(&patch_faces[1..]);
        topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &outer_uses }, ShellSpec { kind: ShellKind::Void, uses: &void_uses }])?
    } else {
        let all = uses(&faces);
        topo.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &all }])?
    };
    let body = topo.make_body(&[volume])?;
    Ok(Built { body, volume, faces, edges: all_edges, vertices: all_vertices })
}

/// A pcurve of a spherical patch: a line in `(u, v)` over the edge's parameter range.
struct PcSpec {
    origin: (f64, f64),
    dir: (f64, f64),
    range: Interval,
}

/// Reverse a loop: opposite order, opposite senses.
fn reverse_loop(lp: &mut [(EntityId, Sense)]) {
    lp.reverse();
    for e in lp.iter_mut() {
        e.1 = e.1.flip();
    }
}

fn planar_face(topo: &mut dyn TopoHost, loops: &[Vec<(EntityId, Sense)>], plane: EntityId) -> Result<EntityId, LayerError> {
    let specs: Vec<Vec<CoedgeSpec>> = loops.iter().map(|l| l.iter().map(|&(edge, sense)| CoedgeSpec { edge, sense }).collect()).collect();
    let refs: Vec<&[CoedgeSpec]> = specs.iter().map(Vec::as_slice).collect();
    Ok(topo.make_face(&refs, Some(SurfaceCarrier { surface: plane, sense: F }))?)
}

fn line_edge(topo: &mut dyn TopoHost, a: (EntityId, Point3), b: (EntityId, Point3)) -> Result<EntityId, LayerError> {
    let d = b.1 - a.1;
    let cv = curve(topo, CurveDef::Line(Line::new(a.1, d)?))?;
    Ok(topo.make_edge(a.0, b.0, Some(CurveCarrier { curve: cv, range: Interval::new(0.0, d.norm()), sense: F }))?)
}
