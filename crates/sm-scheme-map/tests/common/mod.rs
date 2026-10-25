//! Fixtures: bare faces built through `TopologyEdit` (no volume), a registry with the stub
//! `equal` curve scheme and the real `map`, and a one-call mesher.
#![allow(dead_code, missing_docs)]

use core::f64::consts::FRAC_PI_2;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{
    Circle, CurveDef, Cylinder, Frame, Interval, Line, Line2, PCurve, Plane, SurfaceDef, Uv,
};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_model_api::Model;
use sm_scheme_api::stubs::Equal;
use sm_scheme_api::{
    Driver, Executor, Firm, IntervalRequest, MeshInput, MeshIntent, Meshed, ReferenceCornerPicker,
    ReferenceIntervalSolver, SchemeRegistry, WholeFaces,
};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::{
    CellComplex, CoedgeKey, CoedgeSpec, CurveCarrier, OpId, Sense, Side, SurfaceCarrier,
    TopologyEdit,
};

/// A bare face with the ids a test needs.
pub struct BareFace {
    pub topo: CellComplex,
    pub model: Model,
    pub face: EntityId,
    pub vertices: Vec<EntityId>,
    /// Edges in loop order (edge `i` runs from vertex `i` to vertex `i + 1`).
    pub edges: Vec<EntityId>,
}

/// `equal` (stub curve scheme) plus the real `map`.
pub fn registry() -> SchemeRegistry {
    let mut reg = SchemeRegistry::new();
    reg.register(Box::new(Equal)).expect("equal");
    sm_scheme_map::register(&mut reg).expect("map");
    reg
}

fn line_edge(t: &mut CellComplex, a: (EntityId, Point3), b: (EntityId, Point3)) -> EntityId {
    let d = b.1 - a.1;
    let len = d.norm();
    let dir = d * (1.0 / len);
    let curve = t
        .shapes_mut()
        .expect("shapes")
        .add_curve(CurveDef::Line(Line::new(a.1, dir).expect("line")))
        .expect("curve");
    t.make_edge(
        a.0,
        b.0,
        Some(CurveCarrier {
            curve,
            range: Interval::new(0.0, len),
            sense: Sense::Forward,
        }),
    )
    .expect("edge")
}

/// Planar polygon with straight edges, no pcurves. `rings[0]` is the outer loop (counter-
/// clockwise about `normal`), further rings are holes (clockwise).
pub fn polygon_with_holes(rings: &[&[[f64; 3]]], normal: [f64; 3]) -> BareFace {
    let mut topo = CellComplex::new();
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let face = with_op(&mut topo, OpId(1), |t| {
        let n = Vector3::new(normal[0], normal[1], normal[2]);
        let frame = Frame::from_z(
            Point3::new(rings[0][0][0], rings[0][0][1], rings[0][0][2]),
            n,
        )?;
        let surface = t
            .shapes_mut()?
            .add_surface(SurfaceDef::Plane(Plane { frame }))?;
        let mut loops: Vec<Vec<CoedgeSpec>> = Vec::new();
        for ring in rings {
            let vs: Vec<(EntityId, Point3)> = ring
                .iter()
                .map(|p| {
                    let pt = Point3::new(p[0], p[1], p[2]);
                    Ok((t.make_vertex(pt)?, pt))
                })
                .collect::<Result<_, sm_topo_api::fixtures::FixtureError>>()?;
            let mut l = Vec::new();
            for i in 0..vs.len() {
                let e = line_edge(t, vs[i], vs[(i + 1) % vs.len()]);
                edges.push(e);
                l.push(CoedgeSpec {
                    edge: e,
                    sense: Sense::Forward,
                });
            }
            vertices.extend(vs.iter().map(|v| v.0));
            loops.push(l);
        }
        let refs: Vec<&[CoedgeSpec]> = loops.iter().map(Vec::as_slice).collect();
        Ok(t.make_face(
            &refs,
            Some(SurfaceCarrier {
                surface,
                sense: Sense::Forward,
            }),
        )?)
    })
    .expect("polygon face");
    BareFace {
        topo,
        model: Model::new(),
        face,
        vertices,
        edges,
    }
}

/// Planar polygon (single loop, counter-clockwise about +z unless `normal` says otherwise).
pub fn polygon(points: &[[f64; 3]], normal: [f64; 3]) -> BareFace {
    polygon_with_holes(&[points], normal)
}

/// `[0, w] x [0, h]` in the z = 0 plane.
pub fn rectangle(w: f64, h: f64) -> BareFace {
    polygon(
        &[[0.0, 0.0, 0.0], [w, 0.0, 0.0], [w, h, 0.0], [0.0, h, 0.0]],
        [0.0, 0.0, 1.0],
    )
}

fn pc_line(origin: (f64, f64), dir: (f64, f64), range: Interval) -> PCurve {
    PCurve::Line2(Line2 {
        origin: Uv::new(origin.0, origin.1),
        dir: Uv::new(dir.0, dir.1),
        range,
    })
}

/// The quarter of a cylinder of radius `r` and height `h` with `u in [0, pi/2]`, with pcurves
/// `(u, v) = (angle, z)`. Edges: `[bottom arc, right line, top arc, left line]` (the top arc is
/// used reversed, as is the left line).
pub fn quarter_cylinder(r: f64, h: f64) -> BareFace {
    let mut topo = CellComplex::new();
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    let face = with_op(&mut topo, OpId(1), |t| {
        let (a, b) = (Point3::new(r, 0.0, 0.0), Point3::new(0.0, r, 0.0));
        let (c, d) = (Point3::new(0.0, r, h), Point3::new(r, 0.0, h));
        let (va, vb, vc, vd) = (
            t.make_vertex(a)?,
            t.make_vertex(b)?,
            t.make_vertex(c)?,
            t.make_vertex(d)?,
        );
        vertices.extend([va, vb, vc, vd]);
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let quarter = Interval::new(0.0, FRAC_PI_2);
        let height = Interval::new(0.0, h);
        let cb = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(
            Frame::from_zx(Point3::new(0.0, 0.0, 0.0), z, x)?,
            r,
        )?))?;
        let ct = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(
            Frame::from_zx(Point3::new(0.0, 0.0, h), z, x)?,
            r,
        )?))?;
        let lr = t
            .shapes_mut()?
            .add_curve(CurveDef::Line(Line::new(b, z)?))?;
        let ll = t
            .shapes_mut()?
            .add_curve(CurveDef::Line(Line::new(a, z)?))?;
        let e_bot = t.make_edge(
            va,
            vb,
            Some(CurveCarrier {
                curve: cb,
                range: quarter,
                sense: Sense::Forward,
            }),
        )?;
        let e_right = t.make_edge(
            vb,
            vc,
            Some(CurveCarrier {
                curve: lr,
                range: height,
                sense: Sense::Forward,
            }),
        )?;
        let e_top = t.make_edge(
            vd,
            vc,
            Some(CurveCarrier {
                curve: ct,
                range: quarter,
                sense: Sense::Forward,
            }),
        )?;
        let e_left = t.make_edge(
            va,
            vd,
            Some(CurveCarrier {
                curve: ll,
                range: height,
                sense: Sense::Forward,
            }),
        )?;
        edges.extend([e_bot, e_right, e_top, e_left]);
        let s = t
            .shapes_mut()?
            .add_surface(SurfaceDef::Cylinder(Cylinder::new(
                Frame::at(Point3::new(0.0, 0.0, 0.0)),
                r,
            )?))?;
        let lp = [
            CoedgeSpec {
                edge: e_bot,
                sense: Sense::Forward,
            },
            CoedgeSpec {
                edge: e_right,
                sense: Sense::Forward,
            },
            CoedgeSpec {
                edge: e_top,
                sense: Sense::Reversed,
            },
            CoedgeSpec {
                edge: e_left,
                sense: Sense::Reversed,
            },
        ];
        let f = t.make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface: s,
                sense: Sense::Forward,
            }),
        )?;
        let key = |edge, side| CoedgeKey {
            face: f,
            edge,
            side,
        };
        t.set_pcurve(
            key(e_bot, Side::Left),
            Some(pc_line((0.0, 0.0), (1.0, 0.0), quarter)),
        )?;
        t.set_pcurve(
            key(e_right, Side::Left),
            Some(pc_line((FRAC_PI_2, 0.0), (0.0, 1.0), height)),
        )?;
        t.set_pcurve(
            key(e_top, Side::Right),
            Some(pc_line((0.0, h), (1.0, 0.0), quarter)),
        )?;
        t.set_pcurve(
            key(e_left, Side::Right),
            Some(pc_line((0.0, 0.0), (0.0, 1.0), height)),
        )?;
        Ok(f)
    })
    .expect("quarter cylinder");
    BareFace {
        topo,
        model: Model::new(),
        face,
        vertices,
        edges,
    }
}

/// The lateral face of the full cylinder fixture (periodic seam used twice).
pub fn cylinder_lateral(
    r: f64,
    h: f64,
) -> (CellComplex, sm_topo_api::fixtures::CylinderIds, Model) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| {
        sm_topo_api::fixtures::cylinder(t, r, h)
    })
    .expect("cylinder");
    (c, ids, Model::new())
}

/// Hard interval counts per edge.
pub fn counts(pairs: &[(EntityId, u32)]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, c) in pairs {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(*c)));
    }
    intent
}

/// Mesh the faces `sel` of `topo` into a dimension-2 mesh with the `map` registry.
pub fn mesh_faces(
    topo: &CellComplex,
    model: &Model,
    sel: &[EntityId],
    intent: &MeshIntent,
    exec: &dyn Executor,
) -> Result<Meshed<Mesh>, sm_scheme_api::SchemeError> {
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let input = MeshInput::new(topo, model, &WholeFaces, sel);
    driver.mesh(
        &input,
        intent,
        exec,
        MeshBuilder::surface(),
        &mut SerialAlloc::new(),
    )
}

/// Every vertex position of a mesh.
pub fn points(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    (0..m.vertex_count())
        .map(|v| {
            let p = m.point(v);
            [p.x, p.y, p.z]
        })
        .collect()
}

/// Total area of a dimension-2 mesh through `sm_quality`.
pub fn area(m: &dyn MeshRead) -> f64 {
    let v = sm_quality::compute(
        m,
        sm_quality::Metric::Volume,
        &sm_quality::Options::default(),
    )
    .expect("quality");
    v.iter().sum()
}

/// A longitude/latitude band of a sphere of radius `r`: longitudes `[0, lon]`, latitudes
/// `[lat0, lat1]`, with pcurves `(u, v) = (longitude, latitude)`. Edges: `[bottom latitude arc,
/// right meridian, top latitude arc (reversed), left meridian (reversed)]`.
pub fn sphere_band(r: f64, lon: f64, lat0: f64, lat1: f64) -> BareFace {
    let mut topo = CellComplex::new();
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    let face = with_op(&mut topo, OpId(1), |t| {
        let at = |lam: f64, phi: f64| {
            Point3::new(
                r * phi.cos() * lam.cos(),
                r * phi.cos() * lam.sin(),
                r * phi.sin(),
            )
        };
        let vs = [at(0.0, lat0), at(lon, lat0), at(lon, lat1), at(0.0, lat1)];
        let v: Vec<EntityId> = vs
            .iter()
            .map(|p| t.make_vertex(*p))
            .collect::<Result<_, _>>()?;
        vertices.extend(v.iter().copied());
        let up = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let lat_arc = |t: &mut CellComplex,
                       phi: f64|
         -> Result<EntityId, sm_topo_api::fixtures::FixtureError> {
            let f = Frame::from_zx(Point3::new(0.0, 0.0, r * phi.sin()), up, x)?;
            Ok(t.shapes_mut()?
                .add_curve(CurveDef::Circle(Circle::new(f, r * phi.cos())?))?)
        };
        let meridian = |t: &mut CellComplex,
                        lam: f64|
         -> Result<EntityId, sm_topo_api::fixtures::FixtureError> {
            let (c, s) = (lam.cos(), lam.sin());
            let f = Frame::from_zx(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::new(s, -c, 0.0),
                Vector3::new(c, s, 0.0),
            )?;
            Ok(t.shapes_mut()?
                .add_curve(CurveDef::Circle(Circle::new(f, r)?))?)
        };
        let (cb, ct) = (lat_arc(t, lat0)?, lat_arc(t, lat1)?);
        let (cr, cl) = (meridian(t, lon)?, meridian(t, 0.0)?);
        let along = Interval::new(0.0, lon);
        let up_range = Interval::new(lat0, lat1);
        let e_bot = t.make_edge(
            v[0],
            v[1],
            Some(CurveCarrier {
                curve: cb,
                range: along,
                sense: Sense::Forward,
            }),
        )?;
        let e_right = t.make_edge(
            v[1],
            v[2],
            Some(CurveCarrier {
                curve: cr,
                range: up_range,
                sense: Sense::Forward,
            }),
        )?;
        let e_top = t.make_edge(
            v[3],
            v[2],
            Some(CurveCarrier {
                curve: ct,
                range: along,
                sense: Sense::Forward,
            }),
        )?;
        let e_left = t.make_edge(
            v[0],
            v[3],
            Some(CurveCarrier {
                curve: cl,
                range: up_range,
                sense: Sense::Forward,
            }),
        )?;
        edges.extend([e_bot, e_right, e_top, e_left]);
        let s = t
            .shapes_mut()?
            .add_surface(SurfaceDef::Sphere(sm_geom_api::Sphere::new(
                Frame::at(Point3::new(0.0, 0.0, 0.0)),
                r,
            )?))?;
        let lp = [
            CoedgeSpec {
                edge: e_bot,
                sense: Sense::Forward,
            },
            CoedgeSpec {
                edge: e_right,
                sense: Sense::Forward,
            },
            CoedgeSpec {
                edge: e_top,
                sense: Sense::Reversed,
            },
            CoedgeSpec {
                edge: e_left,
                sense: Sense::Reversed,
            },
        ];
        let f = t.make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface: s,
                sense: Sense::Forward,
            }),
        )?;
        let key = |edge, side| CoedgeKey {
            face: f,
            edge,
            side,
        };
        t.set_pcurve(
            key(e_bot, Side::Left),
            Some(pc_line((0.0, lat0), (1.0, 0.0), along)),
        )?;
        t.set_pcurve(
            key(e_right, Side::Left),
            Some(pc_line((lon, 0.0), (0.0, 1.0), up_range)),
        )?;
        t.set_pcurve(
            key(e_top, Side::Right),
            Some(pc_line((0.0, lat1), (1.0, 0.0), along)),
        )?;
        t.set_pcurve(
            key(e_left, Side::Right),
            Some(pc_line((0.0, 0.0), (0.0, 1.0), up_range)),
        )?;
        Ok(f)
    })
    .expect("sphere band");
    BareFace {
        topo,
        model: Model::new(),
        face,
        vertices,
        edges,
    }
}

/// A cylinder sector of radius `r`, height `h` and angular extent `sweep` with `u in [0, sweep]`
/// and **no pcurves** (the 3-D fallback path). Edges as in [`quarter_cylinder`].
pub fn cylinder_sector_no_pcurves(r: f64, h: f64, sweep: f64) -> BareFace {
    let mut topo = CellComplex::new();
    let mut edges = Vec::new();
    let mut vertices = Vec::new();
    let face = with_op(&mut topo, OpId(1), |t| {
        let at = |a: f64, z: f64| Point3::new(r * a.cos(), r * a.sin(), z);
        let (pa, pb, pc, pd) = (at(0.0, 0.0), at(sweep, 0.0), at(sweep, h), at(0.0, h));
        let v: Vec<EntityId> = [pa, pb, pc, pd]
            .iter()
            .map(|p| t.make_vertex(*p))
            .collect::<Result<_, _>>()?;
        vertices.extend(v.iter().copied());
        let z = Vector3::new(0.0, 0.0, 1.0);
        let x = Vector3::new(1.0, 0.0, 0.0);
        let arc = Interval::new(0.0, sweep);
        let height = Interval::new(0.0, h);
        let cb = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(
            Frame::from_zx(Point3::new(0.0, 0.0, 0.0), z, x)?,
            r,
        )?))?;
        let ct = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(
            Frame::from_zx(Point3::new(0.0, 0.0, h), z, x)?,
            r,
        )?))?;
        let lr = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pb, z)?))?;
        let ll = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pa, z)?))?;
        let carrier = |curve, range| CurveCarrier {
            curve,
            range,
            sense: Sense::Forward,
        };
        let e_bot = t.make_edge(v[0], v[1], Some(carrier(cb, arc)))?;
        let e_right = t.make_edge(v[1], v[2], Some(carrier(lr, height)))?;
        let e_top = t.make_edge(v[3], v[2], Some(carrier(ct, arc)))?;
        let e_left = t.make_edge(v[0], v[3], Some(carrier(ll, height)))?;
        edges.extend([e_bot, e_right, e_top, e_left]);
        let s = t
            .shapes_mut()?
            .add_surface(SurfaceDef::Cylinder(Cylinder::new(
                Frame::at(Point3::new(0.0, 0.0, 0.0)),
                r,
            )?))?;
        let spec = |edge, sense| CoedgeSpec { edge, sense };
        let lp = [
            spec(e_bot, Sense::Forward),
            spec(e_right, Sense::Forward),
            spec(e_top, Sense::Reversed),
            spec(e_left, Sense::Reversed),
        ];
        Ok(t.make_face(
            &[&lp],
            Some(SurfaceCarrier {
                surface: s,
                sense: Sense::Forward,
            }),
        )?)
    })
    .expect("cylinder sector");
    BareFace {
        topo,
        model: Model::new(),
        face,
        vertices,
        edges,
    }
}
