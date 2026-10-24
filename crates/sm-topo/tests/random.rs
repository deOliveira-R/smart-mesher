//! Randomized differential driver: random operator sequences (valid and invalid) run in lock step
//! on the reference `CellComplex` and on `TopoStore`. Every result, error, rollback and the full
//! observable state must agree after every operation.

mod common;

use common::{Rng, Twin};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, ModelTolerance, PCurve, Plane, SurfaceDef, Uv};
use sm_topo_api::fixtures::{self, FixtureError};
use sm_topo_api::*;

fn fx<T>(r: Result<T, FixtureError>) -> Result<T, TopoError> {
    r.map_err(|e| match e {
        FixtureError::Topo(t) => t,
        FixtureError::Geom(g) => TopoError::Geom(g),
    })
}

fn pick(t: &dyn TopologyEdit, dim: Dim, r: &mut Rng) -> Option<EntityId> {
    r.pick(t.cells(dim))
}

fn point_of(t: &dyn TopologyEdit, v: EntityId) -> Option<Point3> {
    match t.carrier(v) {
        Carrier::Point(p) => Some(*p),
        _ => None,
    }
}

fn line_edge(t: &mut dyn TopologyEdit, a: EntityId, b: EntityId) -> Result<EntityId, TopoError> {
    let (Some(pa), Some(pb)) = (point_of(t, a), point_of(t, b)) else { return Err(TopoError::InvalidParameter("no point")) };
    let d = pb - pa;
    let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pa, d)?))?;
    t.make_edge(a, b, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))
}

/// A closed triangle of fresh vertices and edges near `c`; returns the loop and its edges.
fn triangle(t: &mut dyn TopologyEdit, c: Point3, s: f64) -> Result<Vec<CoedgeSpec>, TopoError> {
    let a = t.make_vertex(Point3::new(c.x, c.y, c.z))?;
    let b = t.make_vertex(Point3::new(c.x + s, c.y, c.z))?;
    let d = t.make_vertex(Point3::new(c.x, c.y + s, c.z))?;
    let e1 = line_edge(t, a, b)?;
    let e2 = line_edge(t, b, d)?;
    let e3 = line_edge(t, d, a)?;
    Ok(vec![CoedgeSpec { edge: e1, sense: Sense::Forward }, CoedgeSpec { edge: e2, sense: Sense::Forward }, CoedgeSpec { edge: e3, sense: Sense::Forward }])
}

fn free_triangle_face(t: &mut dyn TopologyEdit, c: Point3, s: f64) -> Result<EntityId, TopoError> {
    let lp = triangle(t, c, s)?;
    let frame = Frame::from_z(c, Vector3::new(0.0, 0.0, 1.0))?;
    let surface = t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame }))?;
    t.make_face(&[&lp], Some(SurfaceCarrier { surface, sense: Sense::Forward }))
}

fn side(r: &mut Rng) -> Side {
    [Side::Left, Side::Right, Side::Both][r.below(3)]
}

fn sense(r: &mut Rng) -> Sense {
    if r.below(2) == 0 { Sense::Forward } else { Sense::Reversed }
}

/// One random action; returns a description of its result (compared across implementations).
fn act(t: &mut dyn TopologyEdit, r: &mut Rng) -> String {
    let k = r.below(44);
    match k {
        0 | 1 => {
            let min = Point3::new(r.unit() * 6.0, r.unit() * 6.0, r.unit() * 6.0);
            let size = [0.5 + r.unit(), 0.5 + r.unit(), 0.5 + r.unit()];
            format!("brick {:?}", fx(fixtures::brick(t, min, size)).map(|b| b.volume))
        }
        2 => format!("brick+webcut {:?}", fx((|| {
            let b = fixtures::brick(t, Point3::new(r.unit() * 3.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
            let w = fixtures::webcut_brick(t, &b, 0.15 + 0.7 * r.unit())?;
            Ok(w.volumes)
        })())),
        3 => format!("two bricks {:?}", fx((|| {
            let (a, b) = fixtures::two_bricks(t)?;
            if r.below(2) == 0 {
                fixtures::merge_touching_bricks(t, &a, &b)
            } else {
                fixtures::merge_touching_bricks_gap(t, &a, &b, Some(1e-4))
            }
        })())),
        4 => format!("cylinder {:?}", fx(fixtures::cylinder(t, 0.5 + r.unit(), 1.0 + r.unit())).map(|c| c.volume)),
        5 => format!("sphere {:?}", fx(fixtures::sphere(t, 0.5 + r.unit())).map(|c| c.volume)),
        6 => format!("baffle {:?}", fx(fixtures::brick_with_baffle(t)).map(|c| c.1)),
        7 => format!("fin {:?}", fx(fixtures::brick_with_fin(t)).map(|c| c.1)),
        8..=10 => {
            // split an edge at a random parameter
            let Some(e) = pick(t, Dim::Edge, r) else { return "no edge".into() };
            let frac = 0.1 + 0.8 * r.unit();
            let Carrier::Curve(cc) = t.carrier(e).clone() else { return format!("split_edge nocurve {:?}", t.split_edge(e, e, 0.5)) };
            let param = cc.range.lo + frac * (cc.range.hi - cc.range.lo);
            let pt = t.shapes().curve(cc.curve).map(|c| c.eval(param));
            let Some(pt) = pt else { return "no curve".into() };
            let v = match t.make_vertex(pt) {
                Ok(v) => v,
                Err(e) => return format!("make_vertex {e:?}"),
            };
            format!("split_edge {:?}", t.split_edge(e, v, param))
        }
        11..=14 => {
            // chord or chain across a face
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            let vs: Vec<EntityId> = t.loop_vertices(f, 0).collect();
            if vs.len() < 3 {
                return "tiny loop".into();
            }
            let i = r.below(vs.len());
            let j = r.below(vs.len());
            let chain = r.below(3) == 0;
            let r2 = (|| -> Result<[EntityId; 2], TopoError> {
                if chain {
                    let (pa, pb) = (point_of(t, vs[i]), point_of(t, vs[j]));
                    let (Some(pa), Some(pb)) = (pa, pb) else { return Err(TopoError::InvalidParameter("no point")) };
                    let mid = t.make_vertex(Point3::new((pa.x + pb.x) / 2.0 + 0.01, (pa.y + pb.y) / 2.0, (pa.z + pb.z) / 2.0))?;
                    let e1 = line_edge(t, vs[i], mid)?;
                    let e2 = line_edge(t, mid, vs[j])?;
                    let ch = [CoedgeSpec { edge: e1, sense: Sense::Forward }, CoedgeSpec { edge: e2, sense: Sense::Forward }];
                    t.split_face(f, FaceCut::Chain { edges: &ch }, InnerLoopAssign::None)
                } else {
                    let e = line_edge(t, vs[i], vs[j])?;
                    let inner = if r.below(2) == 0 { InnerLoopAssign::None } else { InnerLoopAssign::Explicit(&[(1, 0), (2, 1)]) };
                    t.split_face(f, FaceCut::Chord { edge: e }, inner)
                }
            })();
            format!("split_face {r2:?}")
        }
        15 => {
            // imprint a closed loop in the interior of a face
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            let Some(v) = t.loop_vertices(f, 0).next() else { return "no vertex".into() };
            let Some(p) = point_of(t, v) else { return "no point".into() };
            let r2 = (|| {
                let lp = triangle(t, Point3::new(p.x + 0.1, p.y + 0.1, p.z), 0.05)?;
                let assign = match r.below(3) {
                    0 => InnerLoopAssign::None,
                    1 => InnerLoopAssign::Explicit(&[(1, 0)]),
                    _ => InnerLoopAssign::ByPointTest,
                };
                t.split_face(f, FaceCut::ClosedLoop { edges: &lp }, assign)
            })();
            format!("imprint {r2:?}")
        }
        16 => {
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            let Some(v) = t.loop_vertices(f, 0).next() else { return "no vertex".into() };
            let Some(p) = point_of(t, v) else { return "no point".into() };
            let r2 = triangle(t, Point3::new(p.x + 0.2, p.y + 0.1, p.z), 0.04).and_then(|lp| t.make_inner_loop(f, &lp));
            format!("make_inner_loop {r2:?}")
        }
        17 => {
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            format!("kill_inner_loop {:?}", t.kill_inner_loop(f, 1 + r.below(2) as u16))
        }
        18 => {
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            format!("reverse {:?}", t.reverse_face(f))
        }
        19 => {
            let (Some(v), Some(f)) = (pick(t, Dim::Volume, r), pick(t, Dim::Face, r)) else { return "none".into() };
            let f = if r.below(2) == 0 { f } else { r.pick(t.volume_faces(v)).unwrap_or(f) };
            format!("remove_face_use {:?}", t.remove_face_use(v, f, side(r)))
        }
        20 => {
            let (Some(v), Some(f)) = (pick(t, Dim::Volume, r), pick(t, Dim::Face, r)) else { return "none".into() };
            format!("add_face_use {:?}", t.add_face_use(v, r.below(3) as u16, FaceUseSpec { face: f, side: side(r) }))
        }
        21 => {
            let dim = [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Volume, Dim::Body][r.below(5)];
            let Some(c) = pick(t, dim, r) else { return "none".into() };
            if r.below(2) == 0 { format!("kill {:?}", t.kill(c)) } else { format!("kill_cascade {:?}", t.kill_cascade(c)) }
        }
        22 => {
            let (Some(a), Some(b)) = (pick(t, Dim::Vertex, r), pick(t, Dim::Vertex, r)) else { return "none".into() };
            let (a, b) = (a.min(b), a.max(b));
            let gap = [None, Some(1e-4), Some(1e-3)][r.below(3)];
            format!("merge_vertices {:?}", t.merge_vertices(a, b, gap))
        }
        23 => {
            let (Some(a), Some(b)) = (pick(t, Dim::Edge, r), pick(t, Dim::Edge, r)) else { return "none".into() };
            let (a, b) = (a.min(b), a.max(b));
            format!("merge_edges {:?}", t.merge_edges(a, b, sense(r)))
        }
        24 => {
            let (Some(a), Some(b)) = (pick(t, Dim::Face, r), pick(t, Dim::Face, r)) else { return "none".into() };
            let (a, b) = (a.min(b), a.max(b));
            format!("merge_faces {:?}", t.merge_faces(a, b, sense(r)))
        }
        25 => {
            let Some(v) = pick(t, Dim::Vertex, r) else { return "none".into() };
            let edges: Vec<EntityId> = t.coboundary(v).iter().map(|e| e.cell).filter(|c| t.dim_of(*c) == Some(Dim::Edge)).filter(|_| r.below(2) == 0).collect();
            format!("unmerge_vertex {:?}", t.unmerge_vertex(v, &edges))
        }
        26 => {
            let Some(e) = pick(t, Dim::Edge, r) else { return "none".into() };
            let faces: Vec<EntityId> = t.coboundary(e).iter().map(|x| x.cell).filter(|c| t.dim_of(*c) == Some(Dim::Face)).filter(|_| r.below(2) == 0).collect();
            format!("unmerge_edge {:?}", t.unmerge_edge(e, &faces))
        }
        27 => {
            let Some(f) = pick(t, Dim::Face, r) else { return "none".into() };
            format!("unmerge_face {:?}", t.unmerge_face(f, side(r)))
        }
        28 => {
            let dim = [Dim::Vertex, Dim::Edge, Dim::Face, Dim::Volume][r.below(4)];
            let Some(c) = pick(t, dim, r) else { return "none".into() };
            match r.below(4) {
                0 => format!("set_flags {:?}", t.set_flags(c, if r.below(2) == 0 { CellFlags::DEGENERATE } else { CellFlags::NONE })),
                1 => format!("raise_tolerance {:?}", t.raise_tolerance(c, 1e-6 * (1.0 + 90.0 * r.unit()))),
                2 => format!("scale_tolerances {:?}", t.scale_tolerances(&[c], 0.25 + 3.0 * r.unit())),
                _ => format!("record_created {:?}", t.record_created(c)),
            }
        }
        29 => {
            let (Some(a), Some(b)) = (pick(t, Dim::Face, r), pick(t, Dim::Face, r)) else { return "none".into() };
            let st = [Coincidence::ImprintedNotMerged, Coincidence::Overlapping, Coincidence::Merged][r.below(3)];
            format!("record_coincidence {:?}", t.record_coincidence(a, b, st))
        }
        30 => {
            // kill then declare split/merged
            let Some(f) = pick(t, Dim::Face, r) else { return "none".into() };
            let Some(g) = pick(t, Dim::Face, r) else { return "none".into() };
            let a = t.kill(f);
            if r.below(2) == 0 { format!("kill+record_split {a:?} {:?}", t.record_split(f, &[g])) } else { format!("kill+record_merged {a:?} {:?}", t.record_merged(f, g, 1e-5)) }
        }
        31 => {
            // free sheet, then maybe a shell/embed/volume built from free faces
            let c = Point3::new(r.unit() * 8.0, 9.0 + r.unit(), r.unit());
            let r2 = (|| {
                let f = free_triangle_face(t, c, 0.5)?;
                match r.below(4) {
                    0 => pick(t, Dim::Volume, r).map(|v| t.add_shell(v, ShellSpec { kind: ShellKind::Internal, uses: &[FaceUseSpec { face: f, side: Side::Both }] })).unwrap_or(Ok(0)).map(|_| f),
                    1 => pick(t, Dim::Volume, r).map(|v| t.embed(v, f)).unwrap_or(Ok(())).map(|_| f),
                    2 => t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &[FaceUseSpec { face: f, side: Side::Left }] }]).map(|_| f),
                    _ => Ok(f),
                }
            })();
            format!("sheet {r2:?}")
        }
        32 => {
            let Some(v) = pick(t, Dim::Volume, r) else { return "none".into() };
            format!("remove_shell {:?}", t.remove_shell(v, 1 + r.below(2) as u16))
        }
        33 => {
            // tetra from scratch with Carrier::None everywhere, then a body
            let r2 = (|| -> Result<EntityId, TopoError> {
                let p = |t: &mut dyn TopologyEdit, x, y, z| t.make_vertex(Point3::new(x, y, z));
                let v: Vec<EntityId> = [(0.0, 0.0, 20.0), (1.0, 0.0, 20.0), (0.0, 1.0, 20.0), (0.0, 0.0, 21.0)].iter().map(|&(x, y, z)| p(t, x, y, z)).collect::<Result<_, _>>()?;
                let idx = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];
                let e: Vec<EntityId> = idx.iter().map(|&(a, b)| t.make_edge(v[a], v[b], None)).collect::<Result<_, _>>()?;
                let c = |i: usize, fwd: bool| CoedgeSpec { edge: e[i], sense: if fwd { Sense::Forward } else { Sense::Reversed } };
                let faces = [
                    t.make_face(&[&[c(0, true), c(1, true), c(2, true)]], None)?,
                    t.make_face(&[&[c(0, false), c(3, true), c(4, false)]], None)?,
                    t.make_face(&[&[c(1, false), c(4, true), c(5, false)]], None)?,
                    t.make_face(&[&[c(2, false), c(5, true), c(3, false)]], None)?,
                ];
                let sides = [Side::Left, Side::Right, Side::Right, Side::Right];
                let uses: Vec<FaceUseSpec> = faces.iter().zip(sides).map(|(f, s)| FaceUseSpec { face: *f, side: s }).collect();
                let vol = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
                if r.below(2) == 0 {
                    t.make_body(&[vol])?;
                }
                Ok(vol)
            })();
            format!("tetra {r2:?}")
        }
        34 => {
            // merge a genuinely coincident pair (vertices at one point, edges with equal ends, faces with equal edges)
            let cat = r.below(3);
            let vs = t.cells(Dim::Vertex).to_vec();
            match cat {
                0 => {
                    let mut c: Vec<(EntityId, EntityId)> = Vec::new();
                    for (i, &a) in vs.iter().enumerate() {
                        for &b in &vs[i + 1..] {
                            if let (Some(pa), Some(pb)) = (point_of(t, a), point_of(t, b))
                                && (pa - pb).norm() < 1e-9
                            {
                                c.push((a, b));
                            }
                        }
                    }
                    let Some((a, b)) = r.pick(&c) else { return "merge_any none".into() };
                    format!("merge_vertices {:?}", t.merge_vertices(a, b, None))
                }
                1 => {
                    let es = t.cells(Dim::Edge).to_vec();
                    let mut c: Vec<(EntityId, EntityId, Sense)> = Vec::new();
                    for (i, &a) in es.iter().enumerate() {
                        for &b in &es[i + 1..] {
                            let (Some(x), Some(y)) = (t.edge_ends(a), t.edge_ends(b)) else { continue };
                            if x == y {
                                c.push((a, b, Sense::Forward));
                            } else if x == (y.1, y.0) {
                                c.push((a, b, Sense::Reversed));
                            }
                        }
                    }
                    let Some((a, b, rel)) = r.pick(&c) else { return "merge_any none".into() };
                    format!("merge_edges {:?}", t.merge_edges(a, b, rel))
                }
                _ => {
                    let fs = t.cells(Dim::Face).to_vec();
                    let mut c: Vec<(EntityId, EntityId)> = Vec::new();
                    for (i, &a) in fs.iter().enumerate() {
                        for &b in &fs[i + 1..] {
                            if !t.face_edges(a).is_empty() && t.face_edges(a) == t.face_edges(b) {
                                c.push((a, b));
                            }
                        }
                    }
                    let Some((a, b)) = r.pick(&c) else { return "merge_any none".into() };
                    let rel = if r.below(3) == 0 { Sense::Forward } else { Sense::Reversed };
                    format!("merge_faces {:?}", t.merge_faces(a, b, rel))
                }
            }
        }
        35 => {
            let fs: Vec<EntityId> = t.cells(Dim::Face).iter().copied().filter(|f| t.loops(*f).count() > 1).collect();
            let Some(f) = r.pick(&fs) else { return "kill_inner_loop none".into() };
            let n = t.loops(f).count();
            format!("kill_inner_loop {:?}", t.kill_inner_loop(f, 1 + r.below(n - 1) as u16))
        }
        36 => {
            let free: Vec<EntityId> = t.cells(Dim::Face).iter().copied().filter(|f| t.face_class(*f) == Some(FaceClass::Free)).collect();
            let (Some(f), Some(g)) = (r.pick(&free), pick(t, Dim::Face, r)) else { return "kill+record none".into() };
            let a = t.kill(f);
            if r.below(2) == 0 { format!("kill+record_split {a:?} {:?}", t.record_split(f, &[g])) } else { format!("kill+record_merged {a:?} {:?}", t.record_merged(f, g, 1e-5)) }
        }
        37 => {
            let fs: Vec<EntityId> = t.cells(Dim::Face).iter().copied().filter(|f| t.loops(*f).count() > 1).collect();
            let Some(f) = r.pick(&fs) else { return "bridge none".into() };
            let (Some(a), Some(b)) = (t.loop_vertices(f, 0).next(), t.loop_vertices(f, 1).next()) else { return "bridge none".into() };
            let r2 = line_edge(t, a, b).and_then(|e| t.bridge_loops(f, e));
            format!("bridge_loops {r2:?}")
        }
        38 => {
            let (Some(a), Some(b)) = (pick(t, Dim::Vertex, r), pick(t, Dim::Vertex, r)) else { return "wire none".into() };
            let dim = [Dim::Volume, Dim::Face, Dim::Body][r.below(3)];
            let Some(parent) = pick(t, dim, r) else { return "wire none".into() };
            let r2 = line_edge(t, a, b).and_then(|e| {
                t.embed(parent, e)?;
                if r.below(2) == 0 { t.unembed(parent, e) } else { Ok(()) }
            });
            format!("wire {r2:?}")
        }
        39 => match r.below(7) {
            0 => {
                let dim = [Dim::Edge, Dim::Face, Dim::Volume][r.below(3)];
                let Some(c) = pick(t, dim, r) else { return "set_carrier none".into() };
                format!("set_carrier {:?}", t.set_carrier(c, Carrier::None))
            }
            1 => {
                let Some(f) = pick(t, Dim::Face, r) else { return "set_pcurve none".into() };
                let Some(e) = t.boundary(f).first().copied() else { return "set_pcurve none".into() };
                let pc = PCurve::Line2(sm_geom_api::Line2 { origin: Uv::new(r.unit(), r.unit()), dir: Uv::new(1.0, 0.5), range: Interval::new(0.0, 1.0) });
                let key = CoedgeKey { face: f, edge: e.cell, side: e.side };
                format!("set_pcurve {:?}", t.set_pcurve(key, if r.below(4) == 0 { None } else { Some(pc) }))
            }
            2 => {
                let Some(v) = pick(t, Dim::Vertex, r) else { return "set_vertex_point none".into() };
                format!("set_vertex_point {:?}", t.set_vertex_point(v, Point3::new(r.unit(), r.unit(), r.unit())))
            }
            3 => {
                let n = t.shapes().curve_count();
                let Some(id) = EntityId::new(sm_core::EntityKind::Curve, r.below(n.max(1)) as u64) else { return "replace none".into() };
                let Some(def) = t.shapes().curve_def(id).cloned() else { return "replace none".into() };
                format!("replace_curve {:?}", t.replace_curve(id, def))
            }
            4 => {
                let n = t.shapes().surface_count();
                let Some(id) = EntityId::new(sm_core::EntityKind::Surface, r.below(n.max(1)) as u64) else { return "replace none".into() };
                let Some(def) = t.shapes().surface_def(id).cloned() else { return "replace none".into() };
                format!("replace_surface {:?}", t.replace_surface(id, def))
            }
            5 => {
                let tol = ModelTolerance::new(1e-6 * (1.0 + r.unit()), 1e-9, 1e-9, 1e-3);
                format!("set_model_tolerance {:?}", t.set_model_tolerance(tol))
            }
            _ => {
                let Some(f) = pick(t, Dim::Face, r) else { return "record_generated none".into() };
                let sh = match t.carrier(f) {
                    Carrier::Surface(sc) => sc.surface,
                    _ => f,
                };
                format!("record_generated {:?}", t.record_generated(f, sh))
            }
        },
        40 | 41 => {
            // an internal shell of free sheets, then a baffle use in it
            let c = Point3::new(r.unit() * 8.0, 12.0 + r.unit(), r.unit());
            let Some(v) = pick(t, Dim::Volume, r) else { return "shell none".into() };
            let r2 = (|| -> Result<u16, TopoError> {
                let f1 = free_triangle_face(t, c, 0.5)?;
                let f2 = free_triangle_face(t, Point3::new(c.x + 1.0, c.y, c.z), 0.5)?;
                let ring = t.add_shell(v, ShellSpec { kind: ShellKind::Internal, uses: &[FaceUseSpec { face: f1, side: Side::Both }] })?;
                t.add_face_use(v, ring, FaceUseSpec { face: f2, side: Side::Both })?;
                if r.below(2) == 0 {
                    t.remove_face_use(v, f1, Side::Left)?;
                }
                Ok(ring)
            })();
            format!("shell {r2:?}")
        }
        42 | 43 => {
            // grow the same face by repeated chords so a volume row gets long
            let Some(v) = pick(t, Dim::Volume, r) else { return "grow none".into() };
            let mut n = 0;
            for _ in 0..(24 + r.below(30)) {
                let Some(f) = r.pick(t.volume_faces(v)) else { break };
                let vs: Vec<EntityId> = t.loop_vertices(f, 0).collect();
                if vs.len() < 4 || r.below(2) == 0 {
                    // refine an edge of the face first so chords keep being possible
                    let Some(e) = r.pick(t.face_edges(f)) else { continue };
                    let Carrier::Curve(cc) = t.carrier(e).clone() else { continue };
                    let param = cc.range.lo + (0.2 + 0.6 * r.unit()) * (cc.range.hi - cc.range.lo);
                    let Some(pt) = t.shapes().curve(cc.curve).map(|c| c.eval(param)) else { continue };
                    if let Ok(nv) = t.make_vertex(pt) {
                        n += usize::from(t.split_edge(e, nv, param).is_ok());
                    }
                    continue;
                }
                let i = r.below(vs.len());
                let j = (i + 2 + r.below(vs.len() - 3)) % vs.len();
                if line_edge(t, vs[i], vs[j]).and_then(|e| t.split_face(f, FaceCut::Chord { edge: e }, InnerLoopAssign::None)).is_ok() {
                    n += 1;
                }
            }
            format!("grow Ok({n})")
        }
        _ => unreachable!(),
    }
}

type Stats = std::collections::BTreeMap<String, (usize, usize)>;

fn tally(stats: &mut Stats, log: &[String]) {
    for l in log {
        let label = l.split_whitespace().next().unwrap_or("?").to_string();
        let ok = l.contains("Ok(");
        let e = stats.entry(label).or_default();
        if ok { e.0 += 1 } else { e.1 += 1 }
    }
}

fn drive(seed: u64, ops: usize, check_every: usize, stats: &mut Stats) -> sm_topo::Counters {
    let mut tw = Twin::new();
    let mut rng = Rng(seed);
    for n in 1..=ops {
        let s = rng.next();
        let abort = rng.below(8) == 0;
        let ctx = format!("seed {seed} op {n} (sub-seed {s:#x}, abort {abort})");
        let hash_before = sm_topo_api::TopologyView::canonical_hash(&tw.a);
        tw.run(&ctx, |t| t.begin_op(OpId(n as u64))).expect("begin");
        let log = tw.run(&ctx, |t| {
            let mut r = Rng(s);
            let steps = 1 + r.below(3);
            (0..steps).map(|_| act(t, &mut r)).collect::<Vec<String>>()
        });
        tally(stats, &log);
        if abort {
            stats.entry("<abort>".into()).or_default().0 += 1;
            tw.run(&ctx, |t| t.abort_op());
            assert_eq!(sm_topo_api::TopologyView::canonical_hash(&tw.a), hash_before, "{ctx}: abort must restore the hash ({log:?})");
            assert_eq!(sm_topo_api::TopologyView::canonical_hash(&tw.b), hash_before, "{ctx}: abort must restore the hash (sm-topo) ({log:?})");
        } else {
            let end = tw.run(&ctx, |t| t.end_op().map(|o| (o.op, o.created, o.retired, o.lineage_start, o.lineage_len)));
            stats.entry("<end_op>".into()).or_default().0 += usize::from(end.is_ok());
            stats.entry("<end_op>".into()).or_default().1 += usize::from(end.is_err());
            if end.is_err() {
                assert_eq!(sm_topo_api::TopologyView::canonical_hash(&tw.a), hash_before, "{ctx}: failed end_op must restore the hash ({log:?})");
            }
        }
        if n % check_every == 0 || n == ops {
            tw.same(&ctx);
        } else {
            assert_eq!(sm_topo_api::TopologyView::canonical_hash(&tw.a), sm_topo_api::TopologyView::canonical_hash(&tw.b), "{ctx}: hash");
        }
    }
    tw.same(&format!("seed {seed} final"));
    tw.b.counters()
}

fn report(stats: &Stats) {
    for (k, (ok, err)) in stats {
        println!("{k:>20}: ok {ok:5}  err {err:5}");
    }
}

#[test]
fn random_sequences_agree_with_the_reference() {
    let mut stats = Stats::new();
    let (mut sorted, mut full) = (0, 0);
    for seed in 1..=8u64 {
        let c = drive(seed, 60, 4, &mut stats);
        sorted += c.sorted_parent_rewrites;
        full += c.full_parent_rewrites;
    }
    report(&stats);
    println!("sorted rewrites {sorted}, full rewrites {full}");
    assert!(sorted > 0, "the positional parent rewrite never ran");
    // The driver must really exercise the operators (not only their error paths).
    for k in ["brick", "split_edge", "split_face", "merge_vertices", "unmerge_face", "kill_cascade", "reverse", "remove_face_use", "make_inner_loop", "imprint", "sheet", "tetra"] {
        let (ok, _) = stats.get(k).copied().unwrap_or_default();
        assert!(ok > 0, "operator {k} never succeeded");
    }
    assert!(stats["<end_op>"].0 > 0);
}

#[test]
fn long_random_sequences_agree_with_the_reference() {
    let mut stats = Stats::new();
    for seed in 100..102u64 {
        drive(seed, 150, 30, &mut stats);
    }
    report(&stats);
}
