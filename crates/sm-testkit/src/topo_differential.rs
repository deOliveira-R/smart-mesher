//! Differential testing of two `TopologyEdit` implementations: a full observable-state
//! comparison ([`same_state`]) and seeded random drivers that run the same operator sequences
//! on both in lock step and demand equal results, errors, rollbacks and state.
//!
//! All randomness is a [`Rng`] (SplitMix64) seeded from the inputs, so a failure reproduces from
//! its seed. Failure messages carry the seed, the operation number and its sub-seed.

#![allow(clippy::type_complexity)]

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, ModelTolerance, PCurve, Plane, SurfaceDef, Uv};
use sm_topo_api::fixtures::{self, FixtureError};
use sm_topo_api::*;
use std::collections::BTreeMap;
use std::fmt::Debug;

macro_rules! eq {
    ($a:expr, $b:expr, $($msg:tt)+) => {{
        let (x, y) = (&$a, &$b);
        if x != y {
            return Err(format!("{}: {:?} vs {:?}", format!($($msg)+), x, y));
        }
    }};
}

/// SplitMix64: a tiny seeded generator for the randomized drivers.
#[derive(Clone, Debug)]
pub struct Rng(pub u64);

impl Rng {
    /// The next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    /// Uniform in `0..n` (0 if `n` is 0).
    pub fn below(&mut self, n: usize) -> usize {
        if n == 0 { 0 } else { (self.next_u64() % n as u64) as usize }
    }
    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// A uniformly chosen element, or `None` if `xs` is empty.
    pub fn pick<T: Copy>(&mut self, xs: &[T]) -> Option<T> {
        if xs.is_empty() { None } else { Some(xs[self.below(xs.len())]) }
    }
}

/// Compare everything observable of two views: counts and live cells per rank, states, birth
/// ops, flags, carriers, boundary and coboundary rows, tolerance overrides, radial rings,
/// derived face and volume lists, shell kinds, pcurves, coincidences, the lineage log and its
/// indexes, the validation results, a few geometric queries and the canonical hash and
/// [`canonical_words`] stream. `Err` names the first difference.
///
/// # Errors
/// A description of the first difference found.
pub fn same_state(a: &dyn TopologyView, b: &dyn TopologyView) -> Result<(), String> {
    eq!(a.canonical_hash(), b.canonical_hash(), "canonical hash");
    let (wa, wb) = (canonical_words(a), canonical_words(b));
    eq!(wa.len(), wb.len(), "canonical word count");
    if let Some(i) = (0..wa.len()).find(|&i| wa[i] != wb[i]) {
        return Err(format!("canonical words differ at {i}: {:?} vs {:?}", wa[i], wb[i]));
    }
    eq!(a.revision(), b.revision(), "revision");
    eq!(a.tolerance(), b.tolerance(), "tolerance");
    eq!(a.shapes().curve_count(), b.shapes().curve_count(), "curve count");
    eq!(a.shapes().surface_count(), b.shapes().surface_count(), "surface count");
    for d in Dim::ALL {
        eq!(a.cells(d), b.cells(d), "live {d:?}");
        let mut n = 1u64;
        loop {
            let id = EntityId::new(d.kind(), n).unwrap();
            let (sa, sb) = (a.state(id), b.state(id));
            eq!(sa, sb, "state of {id:?}");
            if sa.is_none() {
                break;
            }
            eq!(a.dim_of(id), b.dim_of(id), "dim_of {id:?}");
            eq!(a.born(id), b.born(id), "born {id:?}");
            eq!(a.flags(id), b.flags(id), "flags {id:?}");
            eq!(a.carrier(id), b.carrier(id), "carrier {id:?}");
            eq!(a.boundary(id), b.boundary(id), "boundary {id:?}");
            eq!(a.coboundary(id), b.coboundary(id), "coboundary {id:?}");
            eq!(a.tolerance_override(id), b.tolerance_override(id), "override {id:?}");
            eq!(a.radial_uses(id), b.radial_uses(id), "radial {id:?}");
            eq!(a.volume_faces(id), b.volume_faces(id), "volume_faces {id:?}");
            eq!(a.face_edges(id), b.face_edges(id), "face_edges {id:?}");
            eq!(a.coincident_faces(id), b.coincident_faces(id), "coincident_faces {id:?}");
            for ring in 0..6 {
                eq!(a.shell_kind(id, ring), b.shell_kind(id, ring), "shell kind {id:?}/{ring}");
            }
            if d == Dim::Face {
                for e in a.boundary(id) {
                    for side in [Side::Left, Side::Right] {
                        let k = CoedgeKey { face: id, edge: e.cell, side };
                        eq!(a.pcurve(k), b.pcurve(k), "pcurve {k:?}");
                    }
                }
                eq!(a.face_ordinal(id), b.face_ordinal(id), "face_ordinal {id:?}");
                eq!(a.face_sides(id), b.face_sides(id), "face_sides {id:?}");
            }
            eq!(a.live_cover(id), b.live_cover(id), "live_cover {id:?}");
            n += 1;
        }
    }
    eq!(a.coincidences(), b.coincidences(), "coincidences");
    eq!(a.lineage().records(), b.lineage().records(), "lineage records");
    eq!(a.lineage(), b.lineage(), "lineage indexes");
    eq!(validate(a), validate(b), "validate");
    eq!(validate_geometry(a), validate_geometry(b), "validate_geometry");
    // The derived queries are provided methods on the trait, so they agree if the rows do; spot-check
    // the geometric ones that read many fields.
    for &v in a.cells(Dim::Volume) {
        let (ma, mb) = (a.volume_moments(v), b.volume_moments(v));
        eq!(format!("{ma:?}"), format!("{mb:?}"), "volume_moments {v:?}");
    }
    for &f in a.cells(Dim::Face) {
        let (ma, mb) = (a.face_moments(f), b.face_moments(f));
        eq!(format!("{ma:?}"), format!("{mb:?}"), "face_moments {f:?}");
    }
    Ok(())
}

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

/// One biased action toward the states that stress the shell checks: pole /
/// baffle fixtures, lone DEGENERATE toggles on edges (checked at `end_op` with their faces and volumes)
/// and ops that merely touch faces and volumes afterwards.
fn act_biased(t: &mut dyn TopologyEdit, r: &mut Rng, kind: u8) -> String {
    match kind {
        0 => format!("sphere {:?}", fx(fixtures::sphere(t, 0.5 + r.unit())).map(|c| c.volume)),
        1 => format!("baffle {:?}", fx(fixtures::brick_with_baffle(t)).map(|c| c.1)),
        2 | 3 => {
            // Prefer degenerate (pole) edges; fall back to any edge.
            let edges = t.cells(Dim::Edge).to_vec();
            let poles: Vec<EntityId> = edges.iter().copied().filter(|&e| t.flags(e).contains(CellFlags::DEGENERATE)).collect();
            let Some(e) = (if poles.is_empty() || r.below(4) == 0 { r.pick(&edges) } else { r.pick(&poles) }) else { return "no edge".into() };
            let f = if t.flags(e).contains(CellFlags::DEGENERATE) { CellFlags::NONE } else { CellFlags::DEGENERATE };
            format!("set_flags {:?}", t.set_flags(e, f))
        }
        4 => {
            let Some(f) = pick(t, Dim::Face, r) else { return "no face".into() };
            let c = t.carrier(f).clone();
            format!("touch_face {:?}", t.set_carrier(f, c))
        }
        5 => {
            let Some(v) = pick(t, Dim::Volume, r) else { return "no volume".into() };
            let c = t.carrier(v).clone();
            format!("touch_volume {:?}", t.set_carrier(v, c))
        }
        6 => {
            let Some(v) = pick(t, Dim::Volume, r) else { return "no volume".into() };
            format!("raise_tol {:?}", t.raise_tolerance(v, 1e-6 * (1.0 + r.unit())))
        }
        _ => act(t, r),
    }
}


/// Two implementations driven in lock step.
struct Pair<'a, A, B> {
    a: A,
    b: B,
    hook: &'a dyn Fn(&A, &B),
}

impl<A: TopologyEdit, B: TopologyEdit> Pair<'_, A, B> {
    /// Run `f` on both and demand equal results (errors included).
    fn run<R: PartialEq + Debug>(&mut self, ctx: &str, f: impl Fn(&mut dyn TopologyEdit) -> R) -> R {
        let ra = f(&mut self.a);
        let rb = f(&mut self.b);
        assert_eq!(ra, rb, "{ctx}: results differ (A vs B)");
        ra
    }

    /// Full state comparison plus the caller's hook.
    fn same(&self, ctx: &str) {
        if let Err(e) = same_state(&self.a, &self.b) {
            panic!("{ctx}: {e}");
        }
        (self.hook)(&self.a, &self.b);
    }
}

fn end_summary(t: &mut dyn TopologyEdit) -> Result<(OpId, Vec<EntityId>, Vec<EntityId>, usize, usize), TopoError> {
    t.end_op().map(|o| (o.op, o.created, o.retired, o.lineage_start, o.lineage_len))
}

/// Per-action outcome counts of a random run: label -> (successes, failures).
pub type Stats = BTreeMap<String, (usize, usize)>;

fn tally(stats: &mut Stats, log: &[String]) {
    for l in log {
        let label = l.split_whitespace().next().unwrap_or("?").to_string();
        let ok = l.contains("Ok(");
        let e = stats.entry(label).or_default();
        if ok { e.0 += 1 } else { e.1 += 1 }
    }
}

/// The result of [`run_random_differential`]: the two final states and what the run exercised.
pub struct RandomReport<A, B> {
    /// The final state of the first implementation.
    pub a: A,
    /// The final state of the second implementation.
    pub b: B,
    /// Outcome counts per action label (`<abort>` and `<end_op>` included).
    pub stats: Stats,
}

/// Run `steps` random operations (each 1 to 3 random actions, valid and invalid, one in eight
/// aborted) on fresh instances from `make_a` and `make_b` in lock step. Every result, error and
/// rollback must agree after every operation and the full state ([`same_state`]) must agree at
/// regular checkpoints and at the end; an abort or a failed `end_op` must restore the canonical
/// hash. Seeded from `seed` only. Panics with the seed and operation on any difference.
pub fn run_random_differential<A: TopologyEdit, B: TopologyEdit>(seed: u64, steps: usize, make_a: &dyn Fn() -> A, make_b: &dyn Fn() -> B) -> RandomReport<A, B> {
    run_random_differential_with(seed, steps, make_a, make_b, &|_, _| {})
}

/// [`run_random_differential`] with `hook(a, b)` called after every full-state comparison, for
/// implementation-specific extra checks.
pub fn run_random_differential_with<A: TopologyEdit, B: TopologyEdit>(
    seed: u64,
    steps: usize,
    make_a: &dyn Fn() -> A,
    make_b: &dyn Fn() -> B,
    hook: &dyn Fn(&A, &B),
) -> RandomReport<A, B> {
    let check_every = if steps <= 60 { 4 } else { 30 };
    let mut tw = Pair { a: make_a(), b: make_b(), hook };
    let mut stats = Stats::new();
    let mut rng = Rng(seed);
    for n in 1..=steps {
        let s = rng.next_u64();
        let abort = rng.below(8) == 0;
        let ctx = format!("seed {seed} op {n} (sub-seed {s:#x}, abort {abort})");
        let hash_before = tw.a.canonical_hash();
        tw.run(&ctx, |t| t.begin_op(OpId(n as u64))).expect("begin");
        let log = tw.run(&ctx, |t| {
            let mut r = Rng(s);
            let k = 1 + r.below(3);
            (0..k).map(|_| act(t, &mut r)).collect::<Vec<String>>()
        });
        tally(&mut stats, &log);
        if abort {
            stats.entry("<abort>".into()).or_default().0 += 1;
            tw.run(&ctx, |t| t.abort_op());
            assert_eq!(tw.a.canonical_hash(), hash_before, "{ctx}: abort must restore the hash ({log:?})");
            assert_eq!(tw.b.canonical_hash(), hash_before, "{ctx}: abort must restore the hash (B) ({log:?})");
        } else {
            let end = tw.run(&ctx, end_summary);
            let e = stats.entry("<end_op>".into()).or_default();
            e.0 += usize::from(end.is_ok());
            e.1 += usize::from(end.is_err());
            if end.is_err() {
                assert_eq!(tw.a.canonical_hash(), hash_before, "{ctx}: failed end_op must restore the hash ({log:?})");
            }
        }
        if n % check_every == 0 || n == steps {
            tw.same(&ctx);
        } else {
            assert_eq!(tw.a.canonical_hash(), tw.b.canonical_hash(), "{ctx}: hash");
        }
    }
    tw.same(&format!("seed {seed} final"));
    RandomReport { a: tw.a, b: tw.b, stats }
}

/// Run an operation sequence `(sub-seed, kind, abort)` biased toward the states where
/// committed-but-invalid shells can arise (pole and baffle fixtures, lone DEGENERATE toggles,
/// ops that merely touch faces and volumes) in lock step, with the full state compared after
/// every operation and `hook(a, b)` called after each comparison. `kind` is below 12.
pub fn run_biased_differential<A: TopologyEdit, B: TopologyEdit>(ops: &[(u64, u8, bool)], make_a: &dyn Fn() -> A, make_b: &dyn Fn() -> B, hook: &dyn Fn(&A, &B)) -> (A, B) {
    let mut tw = Pair { a: make_a(), b: make_b(), hook };
    for (n, &(s, kind, abort)) in ops.iter().enumerate() {
        let ctx = format!("op {} (sub-seed {s:#x}, kind {kind}, abort {abort})", n + 1);
        tw.run(&ctx, |t| t.begin_op(OpId(n as u64 + 1))).expect("begin");
        let log = tw.run(&ctx, |t| {
            let mut r = Rng(s);
            let steps = 1 + r.below(2);
            (0..steps)
                .map(|i| {
                    let k = if i == 0 { kind } else { 8 + (r.below(4) as u8) };
                    act_biased(t, &mut r, k)
                })
                .collect::<Vec<String>>()
        });
        if abort {
            tw.run(&ctx, |t| t.abort_op());
        } else {
            let _ = tw.run(&ctx, end_summary);
        }
        tw.same(&format!("{ctx} {log:?}"));
    }
    (tw.a, tw.b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_topo_api::CellComplex;

    fn brick_complex(size: f64) -> CellComplex {
        let mut t = CellComplex::new();
        fixtures::with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [size, 1.0, 1.0])).unwrap();
        t
    }

    #[test]
    fn same_state_accepts_equal_and_names_the_difference() {
        assert_eq!(same_state(&brick_complex(1.0), &brick_complex(1.0)), Ok(()));
        let e = same_state(&brick_complex(1.0), &brick_complex(2.0)).unwrap_err();
        assert!(e.contains("canonical hash"), "{e}");
        let e = same_state(&brick_complex(1.0), &CellComplex::new()).unwrap_err();
        assert!(!e.is_empty());
    }

    #[test]
    fn rng_is_seeded_from_its_input_only() {
        let (mut a, mut b) = (Rng(7), Rng(7));
        assert!((0..16).all(|_| a.next_u64() == b.next_u64()));
        assert_ne!(Rng(7).next_u64(), Rng(8).next_u64());
        assert_eq!(Rng(1).below(0), 0);
        assert!(Rng(1).pick::<u8>(&[]).is_none());
        assert!((0..100).all(|_| (0.0..1.0).contains(&a.unit())));
    }

    #[test]
    fn identical_implementations_agree_and_the_run_is_reproducible() {
        let r1 = run_random_differential(3, 25, &CellComplex::new, &CellComplex::new);
        let r2 = run_random_differential(3, 25, &CellComplex::new, &CellComplex::new);
        assert_eq!(r1.a.canonical_hash(), r2.a.canonical_hash());
        assert_eq!(r1.stats, r2.stats);
        assert!(r1.stats["<end_op>"].0 > 0);
        let hash = |seed| run_random_differential(seed, 25, &CellComplex::new, &CellComplex::new).a.canonical_hash();
        assert_ne!(hash(3), hash(4), "different seeds give different histories");
    }

    #[test]
    fn biased_driver_runs_and_hooks_are_called() {
        let calls = std::cell::Cell::new(0usize);
        let ops = [(1u64, 0u8, false), (2, 2, false), (3, 4, true), (4, 1, false)];
        let (a, b) = run_biased_differential(&ops, &CellComplex::new, &CellComplex::new, &|_, _| calls.set(calls.get() + 1));
        assert_eq!(calls.get(), ops.len());
        assert_eq!(a.canonical_hash(), b.canonical_hash());
    }

    #[test]
    #[should_panic(expected = "hash")]
    fn diverging_implementations_are_caught() {
        // B has a different tolerance policy, so results or the first checkpoint differ.
        let make_b = || CellComplex::with_tolerance(ModelTolerance::new(1e-3, 1e-9, 1e-9, 1e-2));
        run_random_differential(1, 40, &CellComplex::new, &make_b);
    }
}
