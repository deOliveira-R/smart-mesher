//! Conformance scenarios: `join_edges` and `join_faces`, the inverses of `split_edge` and
//! `split_face`.
//!
//! "Restores" means the id-free structural signature of the never-split complex (the canonical
//! hash includes ids, the revision and the lineage log, which a split and a join legitimately
//! change). Refusals must leave the complex bit-identical (`canonical_hash` before and after,
//! inside the same open operation).

use super::{Mk, Scenario, Sys};
use core::f64::consts::TAU;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Circle, CurveDef, Frame, Interval, Line, Line2, PCurve, Uv};
use sm_topo_api::fixtures::{self, FixtureError, with_op};
use sm_topo_api::*;

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
    let bad = validate_geometry(v);
    assert!(bad.is_empty(), "geometry violations: {bad:#?}");
}

/// An id-free structural signature: per-rank counts, sorted vertex positions with degree, edge
/// curve ranges with end-vertex positions, face areas with loop sizes and pcurve counts, the
/// pcurve values themselves (sorted, quantized), volume sizes and moments.
fn signature<E: Sys>(t: &E) -> Vec<String> {
    let q = |x: f64| (x * 1e9).round() as i64;
    let mut out = vec![format!("{:?}", Dim::ALL.map(|d| t.cells(d).len()))];
    let mut vs: Vec<String> = t
        .cells(Dim::Vertex)
        .iter()
        .map(|v| match t.carrier(*v) {
            Carrier::Point(p) => format!("v{:?}", (q(p.x), q(p.y), q(p.z), t.coboundary(*v).len())),
            _ => "v?".into(),
        })
        .collect();
    vs.sort();
    out.extend(vs);
    let at = |v: EntityId| match t.carrier(v) {
        Carrier::Point(p) => (q(p.x), q(p.y), q(p.z)),
        _ => (0, 0, 0),
    };
    let mut es: Vec<String> = t
        .cells(Dim::Edge)
        .iter()
        .map(|e| {
            let curve = match t.carrier(*e) {
                Carrier::Curve(c) => format!("c{}:{}:{}:{:?}", c.curve.to_raw(), q(c.range.lo), q(c.range.hi), c.sense),
                _ => "none".into(),
            };
            let ends = t.edge_ends(*e).map(|(a, b)| (at(a), at(b)));
            format!("e{curve} {ends:?} up{} flags{}", t.coboundary(*e).len(), t.flags(*e).bits())
        })
        .collect();
    es.sort();
    out.extend(es);
    let mut fs: Vec<String> = t
        .cells(Dim::Face)
        .iter()
        .map(|f| {
            let m = t.face_moments(*f).unwrap();
            let loops: Vec<usize> = t.loops(*f).map(|l| l.len()).collect();
            let pcs = t.coedges(*f).filter(|c| t.pcurve(c.key).is_some()).count();
            let c = m.first;
            format!("f{loops:?} pc{pcs} {} {:?} up{}", q(m.area), (q(c.x), q(c.y), q(c.z)), t.coboundary(*f).len())
        })
        .collect();
    fs.sort();
    out.extend(fs);
    // Pcurve values, one line per coedge: wrong data or a mis-wired coedge cannot pass on counts.
    let mut pcs: Vec<String> = Vec::new();
    for f in t.cells(Dim::Face) {
        for c in t.coedges(*f) {
            let Some(pc) = t.pcurve(c.key) else { continue };
            let data = match pc {
                PCurve::Line2(l) => format!("L{:?}", [l.origin.u, l.origin.v, l.dir.u, l.dir.v, l.range.lo, l.range.hi].map(q)),
                other => format!("{other:?}"),
            };
            pcs.push(format!("pc {:?} {:?} {data}", c.key.side, t.edge_ends(c.key.edge).map(|(a, b)| (at(a), at(b)))));
        }
    }
    pcs.sort();
    out.extend(pcs);
    let mut vols: Vec<String> = t
        .cells(Dim::Volume)
        .iter()
        .map(|v| {
            let m = t.volume_moments(*v).unwrap();
            format!("V{} {} {:?}", t.volume_faces(*v).len(), q(m.volume), (q(m.first.x), q(m.first.y), q(m.first.z)))
        })
        .collect();
    vols.sort();
    out.extend(vols);
    out
}

fn line_edge<E: Sys>(t: &mut E, a: (EntityId, Point3), b: (EntityId, Point3)) -> Result<EntityId, FixtureError> {
    let d = b.1 - a.1;
    let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(a.1, d)?))?;
    Ok(t.make_edge(a.0, b.0, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?)
}

fn plain_brick<E: Sys>(make: &Mk<E>) -> E {
    let mut t = make(None);
    with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
    t
}

fn plain_cylinder<E: Sys>(make: &Mk<E>) -> E {
    let mut t = make(None);
    with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    t
}

/// Split brick edge `i` at fraction `frac` of its length.
fn split_brick_edge<E: Sys>(t: &mut E, b: &fixtures::BrickIds, i: usize, frac: f64) -> Result<(EntityId, [EntityId; 2]), FixtureError> {
    let Carrier::Curve(cc) = t.carrier(b.edges[i]).clone() else { panic!("brick edge without curve") };
    let param = cc.range.lo + frac * (cc.range.hi - cc.range.lo);
    let Some(CurveDef::Line(l)) = t.shapes().curve_def(cc.curve) else { panic!("line expected") };
    let p = l.origin + l.dir * param;
    let m = t.make_vertex(p)?;
    let halves = t.split_edge(b.edges[i], m, param)?;
    Ok((m, halves))
}

pub(super) fn split_then_join_restores_every_brick_edge<E: Sys>(make: &Mk<E>) {
    let plain = plain_brick(make);
    for i in 0..12 {
        for frac in [0.25, 0.5, 0.8] {
            let mut t = make(None);
            let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
            let (m, halves) = with_op(&mut t, OpId(2), |t| split_brick_edge(t, &b, i, frac)).unwrap();
            assert_ne!(signature(&t), signature(&plain), "the split is visible");
            // Either argument order gives the same result: the lower id survives.
            let (x, y) = if (i + (frac * 4.0) as usize).is_multiple_of(2) { (halves[0], halves[1]) } else { (halves[1], halves[0]) };
            let joined = with_op(&mut t, OpId(3), |t| Ok(t.join_edges(x, y)?)).unwrap();
            assert_eq!(joined, halves[0].min(halves[1]));
            assert_valid(&t);
            assert_eq!(signature(&t), signature(&plain), "edge {i} at {frac}: split + join = never split, up to ids");
            let other = halves[0].max(halves[1]);
            assert_eq!(t.state(other), Some(EntityState::Merged(OpId(3))));
            assert_eq!(t.state(m), Some(EntityState::Deleted(OpId(3))));
            assert!(t.is_alive(joined));
            assert!(t.lineage().of(other).any(|r| matches!(r.kind, LineageKind::Merged { into, gap } if into == joined && gap == 0.0)));
            assert!(t.lineage().of(joined).any(|r| r.kind == LineageKind::Modified(ModKind::Carrier)));
            assert!(t.lineage().of(m).any(|r| r.kind == LineageKind::Deleted));
            assert_eq!(t.lineage().successors(other), vec![joined]);
        }
    }
}

pub(super) fn join_edges_restores_seam_and_closed_circle_with_their_pcurves<E: Sys>(make: &Mk<E>) {
    let plain = plain_cylinder(make);
    // The seam is used twice by the lateral face (once per side): both coedges join.
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let h = 2.0;
    let original: Vec<Option<PCurve>> = [Side::Left, Side::Right].iter().map(|&s| t.pcurve(CoedgeKey { face: c.faces[2], edge: c.edges[2], side: s }).cloned()).collect();
    let (_, halves) = with_op(&mut t, OpId(2), |t| {
        let m = t.make_vertex(Point3::new(1.5, 0.0, 0.5 * h))?;
        Ok((m, t.split_edge(c.edges[2], m, 0.5 * h)?))
    })
    .unwrap();
    with_op(&mut t, OpId(3), |t| Ok(t.join_edges(halves[1], halves[0])?)).unwrap();
    assert_valid(&t);
    assert_eq!(signature(&t), signature(&plain));
    let joined = halves[0];
    let now: Vec<Option<PCurve>> = [Side::Left, Side::Right].iter().map(|&s| t.pcurve(CoedgeKey { face: c.faces[2], edge: joined, side: s }).cloned()).collect();
    assert_eq!(now, original, "equal pcurves of the halves join to the original");
    assert!(t.pcurve(CoedgeKey { face: c.faces[2], edge: halves[1], side: Side::Left }).is_none());

    // A closed circle split once gives two arcs sharing both vertices; the vertex with degree 2
    // dissolves and the survivor is closed again on the seam vertex.
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let (m, halves) = with_op(&mut t, OpId(2), |t| {
        let m = t.make_vertex(Point3::new(-1.5, 0.0, 0.0))?;
        Ok((m, t.split_edge(c.edges[0], m, core::f64::consts::PI)?))
    })
    .unwrap();
    assert_valid(&t);
    let joined = with_op(&mut t, OpId(3), |t| Ok(t.join_edges(halves[0], halves[1])?)).unwrap();
    assert_valid(&t);
    assert_eq!(signature(&t), signature(&plain));
    assert_eq!(t.edge_ends(joined), Some((c.vertices[0], c.vertices[0])));
    assert_eq!(t.state(m), Some(EntityState::Deleted(OpId(3))));
}

pub(super) fn join_edges_fuses_adjacent_line_pcurves_and_refuses_unfusable_ones<E: Sys>(make: &Mk<E>) {
    let line2 = |range: Interval| PCurve::Line2(Line2 { origin: Uv::new(core::f64::consts::TAU, 0.0), dir: Uv::new(0.0, 1.0), range });
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let (_, halves) = with_op(&mut t, OpId(2), |t| {
        let m = t.make_vertex(Point3::new(1.5, 0.0, 1.0))?;
        Ok((m, t.split_edge(c.edges[2], m, 1.0)?))
    })
    .unwrap();
    let key = |edge, side| CoedgeKey { face: c.faces[2], edge, side };
    t.begin_op(OpId(3)).unwrap();
    // Left: contiguous pieces of one line fuse. Right: a different line cannot.
    t.set_pcurve(key(halves[0], Side::Left), Some(line2(Interval::new(0.0, 1.0)))).unwrap();
    t.set_pcurve(key(halves[1], Side::Left), Some(line2(Interval::new(1.0, 2.0)))).unwrap();
    let other = PCurve::Line2(Line2 { origin: Uv::new(0.5, 0.0), dir: Uv::new(0.0, 1.0), range: Interval::new(0.0, 2.0) });
    t.set_pcurve(key(halves[1], Side::Right), Some(other)).unwrap();
    let before = t.canonical_hash();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::Unsupported(_))));
    assert_eq!(t.canonical_hash(), before, "refusal changes nothing");
    t.set_pcurve(key(halves[1], Side::Right), t.pcurve(key(halves[0], Side::Right)).cloned()).unwrap();
    let joined = t.join_edges(halves[0], halves[1]).unwrap();
    assert_eq!(t.pcurve(key(joined, Side::Left)), Some(&line2(Interval::new(0.0, 2.0))));
    t.end_op().unwrap();
    assert_valid(&t);
}

pub(super) fn join_edges_refuses_what_it_cannot_undo<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
    t.begin_op(OpId(2)).unwrap();
    let h0 = t.canonical_hash();
    // Wrong rank, the same edge twice.
    assert!(matches!(t.join_edges(b.edges[0], b.vertices[0]), Err(TopoError::WrongDim { .. })));
    assert!(matches!(t.join_edges(b.edges[0], b.edges[0]), Err(TopoError::InvalidParameter(_))));
    // Edges that do not share a vertex.
    assert!(matches!(t.join_edges(b.edges[0], b.edges[11]), Err(TopoError::InvalidParameter(_))));
    // A shared vertex of degree three.
    assert!(matches!(t.join_edges(b.edges[0], b.edges[4]), Err(TopoError::InvalidParameter(_))));
    assert_eq!(t.canonical_hash(), h0, "refusals change nothing");
    // Different curves: split an edge, then move one half to another curve.
    let (_, halves) = split_brick_edge(&mut t, &b, 0, 0.5).unwrap();
    let other_curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap())).unwrap();
    let Carrier::Curve(c1) = t.carrier(halves[1]).clone() else { panic!() };
    t.set_carrier(halves[1], Carrier::Curve(CurveCarrier { curve: other_curve, ..c1 })).unwrap();
    let h2 = t.canonical_hash();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::NotCoincident(_))));
    assert_eq!(t.canonical_hash(), h2);
    // Non-adjacent ranges.
    t.set_carrier(halves[1], Carrier::Curve(CurveCarrier { range: Interval::new(c1.range.lo + 0.1, c1.range.hi), ..c1 })).unwrap();
    let h3 = t.canonical_hash();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::NotCoincident(_))));
    assert_eq!(t.canonical_hash(), h3);
    // Curve on one side only.
    t.set_carrier(halves[1], Carrier::None).unwrap();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::NotCoincident(_))));
    // Flags must agree.
    t.set_carrier(halves[1], Carrier::Curve(c1)).unwrap();
    t.set_flags(halves[1], CellFlags::DEGENERATE).unwrap();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::Unsupported(_))));
    // Any other differing bit reaches the flags comparison.
    t.set_flags(halves[1], CellFlags::from_bits(2).unwrap()).unwrap();
    let h4 = t.canonical_hash();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::NotCoincident("edges differ in flags"))));
    assert_eq!(t.canonical_hash(), h4);
    t.set_flags(halves[1], CellFlags::NONE).unwrap();
    // With the carrier restored the halves join again.
    t.join_edges(halves[0], halves[1]).unwrap();
    t.end_op().unwrap();
    assert_valid(&t);
    // Outside an operation.
    assert!(matches!(t.join_edges(b.edges[1], b.edges[2]), Err(TopoError::NoOpenOp)));
}

pub(super) fn join_edges_works_without_curves<E: Sys>(make: &Mk<E>) {
    // A curve-less wire: a - b - c, joined into a - c, then refused to join a closed pair.
    let mut t = make(None);
    let [a, b, c, d] = with_op(&mut t, OpId(1), |t| {
        Ok([t.make_vertex(Point3::new(0.0, 0.0, 0.0))?, t.make_vertex(Point3::new(1.0, 0.0, 0.0))?, t.make_vertex(Point3::new(2.0, 0.0, 0.0))?, t.make_vertex(Point3::new(3.0, 0.0, 0.0))?])
    })
    .unwrap();
    let (e1, e2, e3) = with_op(&mut t, OpId(2), |t| Ok((t.make_edge(a, b, None)?, t.make_edge(c, b, None)?, t.make_edge(c, d, None)?))).unwrap();
    // e1: a->b, e2: c->b (reversed relative to the path a..c): the result keeps e1's direction.
    let j = with_op(&mut t, OpId(3), |t| Ok(t.join_edges(e2, e1)?)).unwrap();
    assert_eq!(j, e1);
    assert_eq!(t.edge_ends(j), Some((a, c)));
    assert_eq!(t.carrier(j), &Carrier::None);
    assert!(!t.is_alive(b));
    // Now a->c and c->d meet at c, which also touches e2? e2 is dead, so c has degree two.
    let j2 = with_op(&mut t, OpId(4), |t| Ok(t.join_edges(j, e3)?)).unwrap();
    assert_eq!(t.edge_ends(j2), Some((a, d)));
    assert_eq!(t.cells(Dim::Edge), &[j2]);
    assert_eq!(t.cells(Dim::Vertex), &[a, d]);
}

/// The unit-ish brick with a face cut by its diagonal chord: returns the complex, brick ids,
/// the chord and the two children of the bottom face.
fn diagonal_cut<E: Sys>(make: &Mk<E>) -> (E, fixtures::BrickIds, EntityId, [EntityId; 2]) {
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
    let (chord, kids) = with_op(&mut t, OpId(2), |t| {
        // Bottom face (-z) has corners v0, v2, v3, v1; the diagonal runs v0 -> v3.
        let chord = line_edge(t, (b.vertices[0], Point3::new(0.0, 0.0, 0.0)), (b.vertices[3], Point3::new(2.0, 3.0, 0.0)))?;
        let kids = t.split_face(b.faces[4], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?;
        Ok((chord, kids))
    })
    .unwrap();
    (t, b, chord, kids)
}

pub(super) fn split_then_join_restores_a_chord_cut_face<E: Sys>(make: &Mk<E>) {
    let plain = plain_brick(make);
    for swap in [false, true] {
        let (mut t, _b, chord, kids) = diagonal_cut(make);
        assert_valid(&t);
        assert_ne!(signature(&t), signature(&plain));
        let (keep, drop) = if swap { (kids[1], kids[0]) } else { (kids[0], kids[1]) };
        let got = with_op(&mut t, OpId(3), |t| Ok(t.join_faces(keep, drop)?)).unwrap();
        assert_eq!(got, keep);
        assert_valid(&t);
        assert_eq!(signature(&t), signature(&plain), "swap {swap}: split + join = never split, up to ids");
        assert_eq!(t.state(drop), Some(EntityState::Merged(OpId(3))));
        assert_eq!(t.state(chord), Some(EntityState::Deleted(OpId(3))));
        assert!(t.is_alive(keep));
        assert!(t.lineage().of(drop).any(|r| matches!(r.kind, LineageKind::Merged { into, gap } if into == keep && gap == 0.0)));
        assert!(t.lineage().of(chord).any(|r| r.kind == LineageKind::Deleted));
        assert_eq!(t.lineage().successors(drop), vec![keep]);
        // The chord ends stay: the cut vertices are corners of the face.
        assert_eq!(t.cells(Dim::Vertex).len(), 8);
        assert!(t.volume_faces(_b.volume).contains(&keep) && !t.volume_faces(_b.volume).contains(&drop));
    }
}

pub(super) fn join_faces_undoes_an_interior_loop_imprint_in_both_roles<E: Sys>(make: &Mk<E>) {
    let plain = plain_brick(make);
    for disk_is_keep in [false, true] {
        let mut t = make(None);
        let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
        // A square imprinted on the +z face (z = 4), counter-clockwise seen from +z.
        let (disk, annulus, loop_edges) = with_op(&mut t, OpId(2), |t| {
            let pts = [(0.5, 0.5), (1.5, 0.5), (1.5, 2.0), (0.5, 2.0)];
            let vs: Vec<EntityId> = pts.iter().map(|&(x, y)| t.make_vertex(Point3::new(x, y, 4.0))).collect::<Result<_, _>>()?;
            let mut es = Vec::new();
            for i in 0..4 {
                let (p, q) = (pts[i], pts[(i + 1) % 4]);
                es.push(line_edge(t, (vs[i], Point3::new(p.0, p.1, 4.0)), (vs[(i + 1) % 4], Point3::new(q.0, q.1, 4.0)))?);
            }
            let lp: Vec<CoedgeSpec> = es.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
            let [annulus, disk] = t.split_face(b.faces[5], FaceCut::ClosedLoop { edges: &lp }, InnerLoopAssign::None)?;
            Ok((disk, annulus, es))
        })
        .unwrap();
        assert_valid(&t);
        assert_eq!(t.loops(annulus).count(), 2);
        let (keep, drop) = if disk_is_keep { (disk, annulus) } else { (annulus, disk) };
        let got = with_op(&mut t, OpId(3), |t| Ok(t.join_faces(keep, drop)?)).unwrap();
        assert_eq!(got, keep);
        assert_valid(&t);
        assert_eq!(signature(&t), signature(&plain), "disk_is_keep {disk_is_keep}");
        assert_eq!(t.loops(keep).count(), 1, "the hole is gone");
        // The whole chain dies: four edges and four vertices.
        assert!(loop_edges.iter().all(|e| t.state(*e) == Some(EntityState::Deleted(OpId(3)))));
        assert_eq!(t.cells(Dim::Vertex).len(), 8);
        assert!(t.lineage().of(keep).any(|r| r.kind == LineageKind::Modified(ModKind::InnerLoop)));
    }
}

pub(super) fn join_faces_and_edges_undo_a_seam_cut_of_a_cylinder<E: Sys>(make: &Mk<E>) {
    let plain = plain_cylinder(make);
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let (circle, halves, kids) = with_op(&mut t, OpId(2), |t| {
        // A circle at mid height through the seam: split the seam there, then cut with the circle.
        let m = t.make_vertex(Point3::new(1.5, 0.0, 1.0))?;
        let halves = t.split_edge(c.edges[2], m, 1.0)?;
        let frame = Frame::from_zx(Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0))?;
        let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, 1.5)?))?;
        let circle = t.make_edge(m, m, Some(CurveCarrier { curve, range: Interval::new(0.0, TAU), sense: Sense::Forward }))?;
        let kids = t.split_face(c.faces[2], FaceCut::ClosedLoop { edges: &[CoedgeSpec { edge: circle, sense: Sense::Forward }] }, InnerLoopAssign::None)?;
        Ok((circle, halves, kids))
    })
    .unwrap();
    assert_valid(&t);
    assert_eq!(t.cells(Dim::Face).len(), 4);
    let keep = with_op(&mut t, OpId(3), |t| Ok(t.join_faces(kids[0], kids[1])?)).unwrap();
    assert_eq!(keep, kids[0]);
    assert_eq!(t.state(circle), Some(EntityState::Deleted(OpId(3))));
    assert_eq!(t.cells(Dim::Face).len(), 3);
    // The seam vertex stays until the seam halves are joined.
    let seam = with_op(&mut t, OpId(4), |t| Ok(t.join_edges(halves[0], halves[1])?)).unwrap();
    assert_valid(&t);
    assert_eq!(seam, halves[0]);
    assert_eq!(signature(&t), signature(&plain), "cut + join_faces + join_edges = the cylinder, up to ids");
    // The kept lateral face has its seam pcurves back.
    for side in [Side::Left, Side::Right] {
        assert!(t.pcurve(CoedgeKey { face: keep, edge: seam, side }).is_some(), "{side:?}");
    }
}

pub(super) fn join_faces_refuses_what_it_cannot_undo<E: Sys>(make: &Mk<E>) {
    // Wrong kinds, the same face twice, different carriers.
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
    t.begin_op(OpId(2)).unwrap();
    let h0 = t.canonical_hash();
    assert!(matches!(t.join_faces(b.faces[0], b.faces[2]), Err(TopoError::NotCoincident(_))), "adjacent faces on different planes");
    assert!(matches!(t.join_faces(b.faces[0], b.faces[0]), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.join_faces(b.faces[0], b.edges[0]), Err(TopoError::WrongDim { .. })));
    assert!(matches!(t.join_faces(b.faces[0], b.volume), Err(TopoError::WrongDim { .. })));
    assert_eq!(t.canonical_hash(), h0, "refusals change nothing");
    // Same carrier but no shared edge (opposite faces given each other's surface).
    let carrier = t.carrier(b.faces[0]).clone();
    t.set_carrier(b.faces[1], carrier).unwrap();
    let h1 = t.canonical_hash();
    assert!(matches!(t.join_faces(b.faces[0], b.faces[1]), Err(TopoError::InvalidParameter(_))), "no shared edge");
    assert_eq!(t.canonical_hash(), h1);
    t.abort_op();

    // Different volume uses: one child of a cut loses its volume (the planner looks before
    // end_op checks closure), and the retired parent is dead.
    let (mut t, b, _chord, kids) = diagonal_cut(make);
    t.begin_op(OpId(3)).unwrap();
    assert!(matches!(t.join_faces(kids[0], b.faces[4]), Err(TopoError::Dead(_))));
    t.remove_face_use(b.volume, kids[1], Side::Left).unwrap();
    let h = t.canonical_hash();
    assert!(matches!(t.join_faces(kids[0], kids[1]), Err(TopoError::NotCoincident(_))));
    assert_eq!(t.canonical_hash(), h);
    t.abort_op();

    // The pieces on either side of a webcut chord share a plane and an edge but belong to
    // different volumes; the cap differs in carrier.
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.5)).unwrap();
    t.begin_op(OpId(3)).unwrap();
    let h = t.canonical_hash();
    assert!(matches!(t.join_faces(w.split_faces[0][0], w.split_faces[0][1]), Err(TopoError::NotCoincident(_))));
    assert!(matches!(t.join_faces(w.split_faces[0][0], w.cap), Err(TopoError::NotCoincident(_))));
    assert_eq!(t.canonical_hash(), h);
    t.abort_op();

    // A shared edge that also borders a third face: hang a sheet on the diagonal chord.
    let (mut t, _b, chord, kids) = diagonal_cut(make);
    t.begin_op(OpId(3)).unwrap();
    let (v0, v3) = t.edge_ends(chord).unwrap();
    let (p0, p3) = (Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 3.0, 0.0));
    let (q0, q3) = (Point3::new(0.0, 0.0, -1.0), Point3::new(2.0, 3.0, -1.0));
    let (m0, m3) = (t.make_vertex(q0).unwrap(), t.make_vertex(q3).unwrap());
    let e_out = line_edge(&mut t, (v3, p3), (m3, q3)).unwrap();
    let e_low = line_edge(&mut t, (m0, q0), (m3, q3)).unwrap();
    let e_back = line_edge(&mut t, (m0, q0), (v0, p0)).unwrap();
    let sheet = [
        CoedgeSpec { edge: chord, sense: Sense::Forward },
        CoedgeSpec { edge: e_out, sense: Sense::Forward },
        CoedgeSpec { edge: e_low, sense: Sense::Reversed },
        CoedgeSpec { edge: e_back, sense: Sense::Forward },
    ];
    let surface = t.shapes_mut().unwrap().add_surface(sm_geom_api::SurfaceDef::Plane(sm_geom_api::Plane { frame: Frame::from_z(p0, Vector3::new(3.0, -2.0, 0.0)).unwrap() })).unwrap();
    t.make_face(&[&sheet], Some(SurfaceCarrier { surface, sense: Sense::Forward })).unwrap();
    let h = t.canonical_hash();
    assert!(matches!(t.join_faces(kids[0], kids[1]), Err(TopoError::NotCoincident(_))), "the chord borders a third face");
    assert_eq!(t.canonical_hash(), h);
    t.abort_op();
}

pub(super) fn join_edges_refuses_a_pcurve_on_one_coedge_only<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let (_, halves) = with_op(&mut t, OpId(2), |t| {
        let m = t.make_vertex(Point3::new(1.5, 0.0, 1.0))?;
        Ok((m, t.split_edge(c.edges[2], m, 1.0)?))
    })
    .unwrap();
    let key = |edge, side| CoedgeKey { face: c.faces[2], edge, side };
    t.begin_op(OpId(3)).unwrap();
    assert!(t.pcurve(key(halves[0], Side::Left)).is_some() && t.pcurve(key(halves[1], Side::Left)).is_some());
    t.set_pcurve(key(halves[1], Side::Left), None).unwrap();
    let h = t.canonical_hash();
    assert!(matches!(t.join_edges(halves[0], halves[1]), Err(TopoError::Unsupported(_))), "pcurve on one of the two coedges");
    assert_eq!(t.canonical_hash(), h, "refusal changes nothing");
    t.abort_op();
}

/// Cylinder with its bottom and top circles split at `u = pi` (vertices `m_b`, `m_t`) and the
/// lateral face cut by the generator between them. Returns `(complex, ids, chord, kids,
/// [bottom halves, top halves])`.
#[allow(clippy::type_complexity)]
fn periodic_chord_cut<E: Sys>(make: &Mk<E>, chord_pcurves: bool) -> (E, fixtures::CylinderIds, EntityId, [EntityId; 2], [[EntityId; 2]; 2]) {
    let mut t = make(None);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.5, 2.0)).unwrap();
    let h = 2.0;
    let (chord, kids, circles) = with_op(&mut t, OpId(2), |t| {
        let mb = t.make_vertex(Point3::new(-1.5, 0.0, 0.0))?;
        let mt = t.make_vertex(Point3::new(-1.5, 0.0, h))?;
        let bottom = t.split_edge(c.edges[0], mb, core::f64::consts::PI)?;
        let top = t.split_edge(c.edges[1], mt, core::f64::consts::PI)?;
        let chord = line_edge(t, (mb, Point3::new(-1.5, 0.0, 0.0)), (mt, Point3::new(-1.5, 0.0, h)))?;
        let kids = t.split_face(c.faces[2], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?;
        if chord_pcurves {
            let line = PCurve::Line2(Line2 { origin: Uv::new(core::f64::consts::PI, 0.0), dir: Uv::new(0.0, 1.0), range: Interval::new(0.0, h) });
            for f in kids {
                let side = t.boundary(f).iter().find(|i| i.cell == chord).map(|i| i.side).unwrap();
                t.set_pcurve(CoedgeKey { face: f, edge: chord, side }, Some(line.clone()))?;
            }
        }
        Ok((chord, kids, [bottom, top]))
    })
    .unwrap();
    (t, c, chord, kids, circles)
}

pub(super) fn join_faces_undoes_a_single_chord_cut_of_a_periodic_face<E: Sys>(make: &Mk<E>) {
    let plain = plain_cylinder(make);
    for chord_pcurves in [false, true] {
        for swap in [false, true] {
            let (mut t, c, chord, kids, circles) = periodic_chord_cut(make, chord_pcurves);
            assert_valid(&t);
            assert_ne!(signature(&t), signature(&plain));
            let (keep, drop) = if swap { (kids[1], kids[0]) } else { (kids[0], kids[1]) };
            let got = with_op(&mut t, OpId(3), |t| Ok(t.join_faces(keep, drop)?)).unwrap();
            assert_eq!(got, keep);
            assert_valid(&t);
            assert_eq!(t.state(chord), Some(EntityState::Deleted(OpId(3))), "the chord dissolves");
            assert!(t.is_alive(c.edges[2]), "the seam stays");
            assert_eq!(t.coedges(keep).filter(|k| k.key.edge == c.edges[2]).count(), 2, "the merged face uses the seam on both sides");
            for side in [Side::Left, Side::Right] {
                assert!(t.pcurve(CoedgeKey { face: keep, edge: c.edges[2], side }).is_some(), "{side:?}");
            }
            assert!(t.lineage().of(drop).any(|r| matches!(r.kind, LineageKind::Merged { into, gap } if into == keep && gap == 0.0)));
            // The circle halves and their vertices are joined back.
            for (i, halves) in circles.into_iter().enumerate() {
                with_op(&mut t, OpId(4 + i as u64), |t| Ok(t.join_edges(halves[0], halves[1])?)).unwrap();
            }
            assert_valid(&t);
            assert_eq!(signature(&t), signature(&plain), "chord pcurves {chord_pcurves}, swap {swap}: cut + join = never cut, up to ids");
        }
    }
}

pub(super) fn join_faces_undoes_a_meridian_cut_of_a_sphere<E: Sys>(make: &Mk<E>) {
    use core::f64::consts::{FRAC_PI_2, PI};
    let r = 2.0;
    let mut t = make(None);
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, r)).unwrap();
    let [pole_s, seam, pole_n] = s.edges;
    let [vs, vn] = s.vertices;
    let (chord, f1, f2) = with_op(&mut t, OpId(2), |t| {
        let [h1, _h2] = t.split_degenerate_edge(pole_s, PI)?;
        let [_n1, n2] = t.split_degenerate_edge(pole_n, PI)?;
        let frame = Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(-1.0, 0.0, 0.0))?;
        let curve = t.shapes_mut()?.add_curve(CurveDef::Circle(Circle::new(frame, r)?))?;
        let chord = t.make_edge(vs, vn, Some(CurveCarrier { curve, range: Interval::new(-FRAC_PI_2, FRAC_PI_2), sense: Sense::Forward }))?;
        let corner = |edge, side| CornerKey { face: s.face, incoming: CoedgeKey { face: s.face, edge, side } };
        let [f1, f2] = t.split_face(s.face, FaceCut::ChordAt { edge: chord, start: corner(h1, Side::Left), end: corner(n2, Side::Right) }, InnerLoopAssign::None)?;
        let line = PCurve::Line2(Line2 { origin: Uv::new(PI, 0.0), dir: Uv::new(0.0, 1.0), range: Interval::new(-FRAC_PI_2, FRAC_PI_2) });
        for f in [f1, f2] {
            let side = t.boundary(f).iter().find(|i| i.cell == chord).map(|i| i.side).unwrap();
            t.set_pcurve(CoedgeKey { face: f, edge: chord, side }, Some(line.clone()))?;
        }
        Ok((chord, f1, f2))
    })
    .unwrap();
    assert_valid(&t);
    let keep = with_op(&mut t, OpId(3), |t| Ok(t.join_faces(f1, f2)?)).unwrap();
    assert_eq!(keep, f1);
    assert_valid(&t);
    assert_eq!(t.state(chord), Some(EntityState::Deleted(OpId(3))));
    assert!(t.is_alive(seam), "the meridian seam stays");
    assert_eq!(t.coedges(keep).filter(|c| c.key.edge == seam).count(), 2);
    let m = t.face_moments(keep).unwrap();
    assert!((m.area - 4.0 * PI * r * r).abs() < 1e-6, "{}", m.area);
    assert_eq!(t.cells(Dim::Face).len(), 1);
}

pub(super) fn join_faces_keeps_the_survivors_coincidence_record<E: Sys>(make: &Mk<E>) {
    let partner = |t: &E, f: EntityId, x: EntityId| t.coincident_faces(f).iter().find(|p| p.other(f) == Some(x)).map(|p| p.status);
    let setup = |both: bool| {
        let (mut t, b, _chord, kids) = diagonal_cut(make);
        let x = b.faces[5];
        with_op(&mut t, OpId(3), |t| {
            if both {
                t.record_coincidence(kids[0], x, Coincidence::ImprintedNotMerged)?;
            }
            t.record_coincidence(kids[1], x, Coincidence::Overlapping)?;
            Ok(())
        })
        .unwrap();
        (t, x, kids)
    };
    // Both halves coincide with the same face: keep's record wins, drop's is discarded.
    let (mut t, x, kids) = setup(true);
    with_op(&mut t, OpId(4), |t| Ok(t.join_faces(kids[0], kids[1])?)).unwrap();
    assert_eq!(partner(&t, kids[0], x), Some(Coincidence::ImprintedNotMerged));
    assert_eq!(partner(&t, kids[1], x), None);
    // Swapped roles: the other record wins.
    let (mut t, x, kids) = setup(true);
    with_op(&mut t, OpId(4), |t| Ok(t.join_faces(kids[1], kids[0])?)).unwrap();
    assert_eq!(partner(&t, kids[1], x), Some(Coincidence::Overlapping));
    assert_eq!(partner(&t, kids[0], x), None);
    // Only drop has a record: it transfers.
    let (mut t, x, kids) = setup(false);
    with_op(&mut t, OpId(4), |t| Ok(t.join_faces(kids[0], kids[1])?)).unwrap();
    assert_eq!(partner(&t, kids[0], x), Some(Coincidence::Overlapping));
}

pub(super) fn join_faces_refuses_a_t_junction_in_the_middle_of_the_chain<E: Sys>(make: &Mk<E>) {
    let (mut t, b, chord, kids) = diagonal_cut(make);
    t.begin_op(OpId(3)).unwrap();
    // Split the chord in two, then hang a loose edge on the middle vertex.
    let Carrier::Curve(cc) = t.carrier(chord).clone() else { panic!("chord curve") };
    let mid = 0.5 * (cc.range.lo + cc.range.hi);
    let Some(CurveDef::Line(l)) = t.shapes().curve_def(cc.curve) else { panic!("line") };
    let pm = l.origin + l.dir * mid;
    let m = t.make_vertex(pm).unwrap();
    t.split_edge(chord, m, mid).unwrap();
    line_edge(&mut t, (m, pm), (b.vertices[2], Point3::new(0.0, 3.0, 0.0))).unwrap();
    let h = t.canonical_hash();
    assert!(matches!(t.join_faces(kids[0], kids[1]), Err(TopoError::Unsupported("join_faces: a chain vertex has other incident cells"))));
    assert_eq!(t.canonical_hash(), h);
    t.abort_op();
}

pub(super) fn join_operators_need_an_open_operation<E: Sys>(make: &Mk<E>) {
    let (mut t, b, _chord, kids) = diagonal_cut(make);
    assert!(matches!(t.join_faces(kids[0], kids[1]), Err(TopoError::NoOpenOp)));
    assert!(matches!(t.join_edges(b.edges[0], b.edges[1]), Err(TopoError::NoOpenOp)));
    // An aborted join leaves the complex as it was.
    let h = t.canonical_hash();
    t.begin_op(OpId(3)).unwrap();
    t.join_faces(kids[0], kids[1]).unwrap();
    t.abort_op();
    assert_eq!(t.canonical_hash(), h);
}

pub(super) fn scenarios<E: Sys>() -> Vec<Scenario<E>> {
    vec![
        ("split_then_join_restores_every_brick_edge", split_then_join_restores_every_brick_edge::<E>),
        ("join_edges_restores_seam_and_closed_circle_with_their_pcurves", join_edges_restores_seam_and_closed_circle_with_their_pcurves::<E>),
        ("join_edges_fuses_adjacent_line_pcurves_and_refuses_unfusable_ones", join_edges_fuses_adjacent_line_pcurves_and_refuses_unfusable_ones::<E>),
        ("join_edges_refuses_what_it_cannot_undo", join_edges_refuses_what_it_cannot_undo::<E>),
        ("join_edges_works_without_curves", join_edges_works_without_curves::<E>),
        ("split_then_join_restores_a_chord_cut_face", split_then_join_restores_a_chord_cut_face::<E>),
        ("join_faces_undoes_an_interior_loop_imprint_in_both_roles", join_faces_undoes_an_interior_loop_imprint_in_both_roles::<E>),
        ("join_faces_and_edges_undo_a_seam_cut_of_a_cylinder", join_faces_and_edges_undo_a_seam_cut_of_a_cylinder::<E>),
        ("join_faces_refuses_what_it_cannot_undo", join_faces_refuses_what_it_cannot_undo::<E>),
        ("join_edges_refuses_a_pcurve_on_one_coedge_only", join_edges_refuses_a_pcurve_on_one_coedge_only::<E>),
        ("join_faces_undoes_a_single_chord_cut_of_a_periodic_face", join_faces_undoes_a_single_chord_cut_of_a_periodic_face::<E>),
        ("join_faces_undoes_a_meridian_cut_of_a_sphere", join_faces_undoes_a_meridian_cut_of_a_sphere::<E>),
        ("join_faces_keeps_the_survivors_coincidence_record", join_faces_keeps_the_survivors_coincidence_record::<E>),
        ("join_faces_refuses_a_t_junction_in_the_middle_of_the_chain", join_faces_refuses_a_t_junction_in_the_middle_of_the_chain::<E>),
        ("join_operators_need_an_open_operation", join_operators_need_an_open_operation::<E>),
    ]
}
