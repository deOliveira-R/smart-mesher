//! Temporary solid bodies for the closed and quadric webcut tools.
//!
//! A bare sphere, torus, cone or elliptic surface cannot be capped loop by loop the way a plane
//! or cylinder can (poles, two seams, nappes). Instead the tool surface is closed into a solid
//! (a cone is cut off by a plane beyond the target, an elliptic cylinder is capped on both
//! ends), and the target is split with the boolean machinery against it, once, so both pieces
//! share the tool-surface fragment. The lateral face always carries the user's surface with
//! `Forward` sense, so the solid lies on the negative side and the surface normal points out of
//! it: the positive side of the tool is the outside of the solid.

use core::f64::consts::{FRAC_PI_2, TAU};

use sm_core::{Aabb, EntityId, Point3};
use sm_geom_api::{
    Circle, CurveDef, Ellipse, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Uv,
};
use sm_topo_api::{
    CellFlags, CoedgeKey, CoedgeSpec, CurveCarrier, FaceUseSpec, Sense, ShellKind, ShellSpec, Side,
    SurfaceCarrier, TopologyEdit,
};

use crate::error::BooleanError;

const F: Sense = Sense::Forward;
const R: Sense = Sense::Reversed;

/// A closed solid built on a tool surface.
pub(crate) struct ToolBody {
    pub volume: EntityId,
    pub body: EntityId,
}

fn curve<E: TopologyEdit + ?Sized>(topo: &mut E, def: CurveDef) -> Result<EntityId, BooleanError> {
    Ok(topo.shapes_mut()?.add_curve(def)?)
}

fn surface<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    def: SurfaceDef,
) -> Result<EntityId, BooleanError> {
    Ok(topo.shapes_mut()?.add_surface(def)?)
}

fn line2(origin: (f64, f64), dir: (f64, f64), range: Interval) -> PCurve {
    PCurve::Line2(Line2 {
        origin: Uv::new(origin.0, origin.1),
        dir: Uv::new(dir.0, dir.1),
        range,
    })
}

fn full() -> Interval {
    Interval::new(0.0, TAU)
}

fn at(f: &Frame, x: f64, y: f64, z: f64) -> Point3 {
    f.origin + f.x * x + f.y * y + f.z * z
}

fn close_solid<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    faces: &[EntityId],
) -> Result<(EntityId, EntityId), BooleanError> {
    let uses: Vec<FaceUseSpec> = faces
        .iter()
        .map(|&face| FaceUseSpec {
            face,
            side: Side::Left,
        })
        .collect();
    let volume = topo.make_volume(&[ShellSpec {
        kind: ShellKind::Outer,
        uses: &uses,
    }])?;
    let body = topo.make_body(&[volume])?;
    Ok((volume, body))
}

/// The extent of `b` along the frame axis: `(zmin, zmax)` of the box corners in local `z`.
fn z_range(f: &Frame, b: &Aabb) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..8 {
        let p = Point3::new(
            if i & 1 == 0 { b.lo.x } else { b.hi.x },
            if i & 2 == 0 { b.lo.y } else { b.hi.y },
            if i & 4 == 0 { b.lo.z } else { b.hi.z },
        );
        let z = f.to_local(p).z;
        lo = lo.min(z);
        hi = hi.max(z);
    }
    (lo, hi)
}

/// Close `def` (stored as `handle`) into a solid that covers `bounds` (the region of interest). `None` when the solid
/// cannot reach `bounds` at all (a cone whose apex lies beyond it): everything there is outside.
pub(crate) fn build<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    def: &SurfaceDef,
    handle: EntityId,
    bounds: &Aabb,
) -> Result<Option<ToolBody>, BooleanError> {
    match def {
        SurfaceDef::Sphere(s) => sphere(topo, handle, &s.frame, s.radius).map(Some),
        SurfaceDef::Torus(t) => torus(topo, handle, &t.frame, t.major, t.minor).map(Some),
        SurfaceDef::Cone(c) => {
            let (_, zmax) = z_range(&c.frame, bounds);
            cone(
                topo,
                handle,
                &c.frame,
                c.radius,
                c.radius,
                c.half_angle,
                c.apex_v(),
                zmax,
            )
        }
        SurfaceDef::EllipticCone(c) => {
            let (_, zmax) = z_range(&c.frame, bounds);
            cone(
                topo,
                handle,
                &c.frame,
                c.a,
                c.b,
                c.half_angle,
                c.apex_v(),
                zmax,
            )
        }
        SurfaceDef::EllipticCylinder(c) => {
            let (zlo, zhi) = z_range(&c.frame, bounds);
            let pad = 0.1 * (zhi - zlo).abs() + 1e-3 * (c.a + c.b);
            elliptic_cylinder(topo, handle, &c.frame, c.a, c.b, zlo - pad, zhi + pad).map(Some)
        }
        _ => Err(BooleanError::Unsupported(
            "webcut supports plane, cylinder, sphere, cone, torus and elliptic tools",
        )),
    }
}

fn sphere<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    handle: EntityId,
    f: &Frame,
    r: f64,
) -> Result<ToolBody, BooleanError> {
    let vs = topo.make_vertex(at(f, 0.0, 0.0, -r))?;
    let vn = topo.make_vertex(at(f, 0.0, 0.0, r))?;
    let mframe = Frame::from_zx(f.origin, -f.y, f.x)?;
    let meridian = curve(topo, CurveDef::Circle(Circle::new(mframe, r)?))?;
    let seam = topo.make_edge(
        vs,
        vn,
        Some(CurveCarrier {
            curve: meridian,
            range: Interval::new(-FRAC_PI_2, FRAC_PI_2),
            sense: F,
        }),
    )?;
    let pole_s = topo.make_edge(vs, vs, None)?;
    let pole_n = topo.make_edge(vn, vn, None)?;
    topo.set_flags(pole_s, CellFlags::DEGENERATE)?;
    topo.set_flags(pole_n, CellFlags::DEGENERATE)?;
    let surf = handle;
    let lp = [
        CoedgeSpec {
            edge: pole_s,
            sense: F,
        },
        CoedgeSpec {
            edge: seam,
            sense: F,
        },
        CoedgeSpec {
            edge: pole_n,
            sense: R,
        },
        CoedgeSpec {
            edge: seam,
            sense: R,
        },
    ];
    let face = topo.make_face(
        &[&lp],
        Some(SurfaceCarrier {
            surface: surf,
            sense: F,
        }),
    )?;
    let key = |edge, side| CoedgeKey { face, edge, side };
    let half = Interval::new(-FRAC_PI_2, FRAC_PI_2);
    topo.set_pcurve(
        key(pole_s, Side::Left),
        Some(line2((0.0, -FRAC_PI_2), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(seam, Side::Left),
        Some(line2((TAU, 0.0), (0.0, 1.0), half)),
    )?;
    topo.set_pcurve(
        key(pole_n, Side::Right),
        Some(line2((0.0, FRAC_PI_2), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(seam, Side::Right),
        Some(line2((0.0, 0.0), (0.0, 1.0), half)),
    )?;
    let (volume, body) = close_solid(topo, &[face])?;
    Ok(ToolBody { volume, body })
}

fn torus<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    handle: EntityId,
    f: &Frame,
    major: f64,
    minor: f64,
) -> Result<ToolBody, BooleanError> {
    let v = topo.make_vertex(at(f, major + minor, 0.0, 0.0))?;
    let c_u = curve(topo, CurveDef::Circle(Circle::new(*f, major + minor)?))?;
    let c_v = curve(
        topo,
        CurveDef::Circle(Circle::new(
            Frame::from_zx(at(f, major, 0.0, 0.0), -f.y, f.x)?,
            minor,
        )?),
    )?;
    let e_u = topo.make_edge(
        v,
        v,
        Some(CurveCarrier {
            curve: c_u,
            range: full(),
            sense: F,
        }),
    )?;
    let e_v = topo.make_edge(
        v,
        v,
        Some(CurveCarrier {
            curve: c_v,
            range: full(),
            sense: F,
        }),
    )?;
    let surf = handle;
    let lp = [
        CoedgeSpec {
            edge: e_u,
            sense: F,
        },
        CoedgeSpec {
            edge: e_v,
            sense: F,
        },
        CoedgeSpec {
            edge: e_u,
            sense: R,
        },
        CoedgeSpec {
            edge: e_v,
            sense: R,
        },
    ];
    let face = topo.make_face(
        &[&lp],
        Some(SurfaceCarrier {
            surface: surf,
            sense: F,
        }),
    )?;
    let key = |edge, side| CoedgeKey { face, edge, side };
    topo.set_pcurve(
        key(e_u, Side::Left),
        Some(line2((0.0, 0.0), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(e_v, Side::Left),
        Some(line2((TAU, 0.0), (0.0, 1.0), full())),
    )?;
    topo.set_pcurve(
        key(e_u, Side::Right),
        Some(line2((0.0, TAU), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(e_v, Side::Right),
        Some(line2((0.0, 0.0), (0.0, 1.0), full())),
    )?;
    let (volume, body) = close_solid(topo, &[face])?;
    Ok(ToolBody { volume, body })
}

fn ellipse_or_circle(frame: Frame, a: f64, b: f64) -> Result<CurveDef, BooleanError> {
    Ok(if a == b {
        CurveDef::Circle(Circle::new(frame, a)?)
    } else {
        CurveDef::Ellipse(Ellipse::new(frame, a, b)?)
    })
}

/// A cone (circular when `a == b`) from its apex to the plane at local height `zmax` (taken a
/// little beyond that). `a` and `b` are the semi-axes at `v = 0`; the circular `Cone` has `a == b
/// == radius`. `None` when the apex is not below the bounds.
#[allow(clippy::too_many_arguments)]
fn cone<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    handle: EntityId,
    f: &Frame,
    a: f64,
    b: f64,
    half: f64,
    apex_v: f64,
    zmax: f64,
) -> Result<Option<ToolBody>, BooleanError> {
    let (sh, ch) = half.sin_cos();
    let z_apex = apex_v * ch;
    if zmax <= z_apex {
        return Ok(None);
    }
    let span = zmax - z_apex;
    let z_hi = zmax + 0.1 * span + 1e-3 * (a + b + 1e-3);
    let v_hi = z_hi / ch;
    let slant = v_hi - apex_v;
    // Semi-axes of the base ellipse at v_hi: the section scales with (1 + v sin / a) for the elliptic
    // cone; for the circular cone a == b == radius gives radius + v sin.
    let k_hi = if a > 0.0 { 1.0 + v_hi * sh / a } else { 0.0 };
    let (ah, bh) = if a > 0.0 {
        (a * k_hi, b * k_hi)
    } else {
        (v_hi * sh, v_hi * sh)
    };
    let vb = topo.make_vertex(at(f, ah, 0.0, z_hi))?;
    let apex = at(f, 0.0, 0.0, z_apex);
    let va = topo.make_vertex(apex)?;
    let big_frame = Frame::from_zx(at(f, 0.0, 0.0, z_hi), f.z, f.x)?;
    let c_big = curve(topo, ellipse_or_circle(big_frame, ah, bh)?)?;
    let dir = apex - at(f, ah, 0.0, z_hi);
    let n = dir.norm();
    let c_gen = curve(
        topo,
        CurveDef::Line(Line::new(at(f, ah, 0.0, z_hi), dir * (1.0 / n))?),
    )?;
    let e_big = topo.make_edge(
        vb,
        vb,
        Some(CurveCarrier {
            curve: c_big,
            range: full(),
            sense: F,
        }),
    )?;
    let e_seam = topo.make_edge(
        vb,
        va,
        Some(CurveCarrier {
            curve: c_gen,
            range: Interval::new(0.0, n),
            sense: F,
        }),
    )?;
    let e_apex = topo.make_edge(va, va, None)?;
    topo.set_flags(e_apex, CellFlags::DEGENERATE)?;
    let s_lat = handle;
    let lp = [
        CoedgeSpec {
            edge: e_apex,
            sense: F,
        },
        CoedgeSpec {
            edge: e_seam,
            sense: R,
        },
        CoedgeSpec {
            edge: e_big,
            sense: R,
        },
        CoedgeSpec {
            edge: e_seam,
            sense: F,
        },
    ];
    let f_lat = topo.make_face(
        &[&lp],
        Some(SurfaceCarrier {
            surface: s_lat,
            sense: F,
        }),
    )?;
    let key = |edge, side| CoedgeKey {
        face: f_lat,
        edge,
        side,
    };
    let sl = Interval::new(0.0, slant);
    topo.set_pcurve(
        key(e_seam, Side::Right),
        Some(line2((TAU, v_hi), (0.0, -1.0), sl)),
    )?;
    topo.set_pcurve(
        key(e_big, Side::Right),
        Some(line2((0.0, v_hi), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(e_seam, Side::Left),
        Some(line2((0.0, v_hi), (0.0, -1.0), sl)),
    )?;
    topo.set_pcurve(
        key(e_apex, Side::Left),
        Some(line2((0.0, apex_v), (1.0, 0.0), full())),
    )?;
    let s_disc = surface(topo, SurfaceDef::Plane(Plane { frame: big_frame }))?;
    let f_big = topo.make_face(
        &[&[CoedgeSpec {
            edge: e_big,
            sense: F,
        }]],
        Some(SurfaceCarrier {
            surface: s_disc,
            sense: F,
        }),
    )?;
    let (volume, body) = close_solid(topo, &[f_lat, f_big])?;
    Ok(Some(ToolBody { volume, body }))
}

fn elliptic_cylinder<E: TopologyEdit + ?Sized>(
    topo: &mut E,
    handle: EntityId,
    f: &Frame,
    a: f64,
    b: f64,
    z_lo: f64,
    z_hi: f64,
) -> Result<ToolBody, BooleanError> {
    let h = z_hi - z_lo;
    let (base, top) = (at(f, 0.0, 0.0, z_lo), at(f, 0.0, 0.0, z_hi));
    let vb = topo.make_vertex(at(f, a, 0.0, z_lo))?;
    let vt = topo.make_vertex(at(f, a, 0.0, z_hi))?;
    let bottom_frame = Frame::from_zx(base, f.z, f.x)?;
    let top_frame = Frame::from_zx(top, f.z, f.x)?;
    let cb = curve(topo, ellipse_or_circle(bottom_frame, a, b)?)?;
    let ct = curve(topo, ellipse_or_circle(top_frame, a, b)?)?;
    let cs = curve(topo, CurveDef::Line(Line::new(at(f, a, 0.0, z_lo), f.z)?))?;
    let e_bot = topo.make_edge(
        vb,
        vb,
        Some(CurveCarrier {
            curve: cb,
            range: full(),
            sense: F,
        }),
    )?;
    let e_top = topo.make_edge(
        vt,
        vt,
        Some(CurveCarrier {
            curve: ct,
            range: full(),
            sense: F,
        }),
    )?;
    let e_seam = topo.make_edge(
        vb,
        vt,
        Some(CurveCarrier {
            curve: cs,
            range: Interval::new(0.0, h),
            sense: F,
        }),
    )?;
    let s_bot = surface(
        topo,
        SurfaceDef::Plane(Plane {
            frame: Frame::from_zx(base, -f.z, f.x)?,
        }),
    )?;
    let s_top = surface(topo, SurfaceDef::Plane(Plane { frame: top_frame }))?;
    let s_lat = handle;
    let f_bot = topo.make_face(
        &[&[CoedgeSpec {
            edge: e_bot,
            sense: R,
        }]],
        Some(SurfaceCarrier {
            surface: s_bot,
            sense: F,
        }),
    )?;
    let f_top = topo.make_face(
        &[&[CoedgeSpec {
            edge: e_top,
            sense: F,
        }]],
        Some(SurfaceCarrier {
            surface: s_top,
            sense: F,
        }),
    )?;
    let lateral = [
        CoedgeSpec {
            edge: e_bot,
            sense: F,
        },
        CoedgeSpec {
            edge: e_seam,
            sense: F,
        },
        CoedgeSpec {
            edge: e_top,
            sense: R,
        },
        CoedgeSpec {
            edge: e_seam,
            sense: R,
        },
    ];
    let f_lat = topo.make_face(
        &[&lateral],
        Some(SurfaceCarrier {
            surface: s_lat,
            sense: F,
        }),
    )?;
    let key = |edge, side| CoedgeKey {
        face: f_lat,
        edge,
        side,
    };
    let hr = Interval::new(0.0, h);
    topo.set_pcurve(
        key(e_bot, Side::Left),
        Some(line2((0.0, z_lo), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(e_seam, Side::Left),
        Some(line2((TAU, z_lo), (0.0, 1.0), hr)),
    )?;
    topo.set_pcurve(
        key(e_top, Side::Right),
        Some(line2((0.0, z_hi), (1.0, 0.0), full())),
    )?;
    topo.set_pcurve(
        key(e_seam, Side::Right),
        Some(line2((0.0, z_lo), (0.0, 1.0), hr)),
    )?;
    let (volume, body) = close_solid(topo, &[f_bot, f_top, f_lat])?;
    Ok(ToolBody { volume, body })
}
