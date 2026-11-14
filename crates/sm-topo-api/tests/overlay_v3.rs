//! ACR-V3: virtual edge composites and collapses, the overlay's canonical digest.

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::*;
use std::sync::Arc;

const SHORT: f64 = 2e-4;

struct Strip {
    host: Arc<CellComplex>,
    v: [EntityId; 6],
    e01: EntityId,
    e12: EntityId,
    e23: EntityId,
    e34: EntityId,
    e45: EntityId,
    e1w: EntityId,
    faces: [EntityId; 3],
}

fn pt(x: f64, y: f64) -> Point3 {
    Point3::new(x, y, 0.0)
}

fn line_edge(t: &mut CellComplex, a: EntityId, b: EntityId, pa: Point3, pb: Point3) -> EntityId {
    let d = pb - pa;
    let len = d.norm();
    let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(pa, d).unwrap())).unwrap();
    t.make_edge(a, b, Some(CurveCarrier { curve, range: Interval::new(0.0, len), sense: Sense::Forward })).unwrap()
}

/// Three planar faces: F1 = unit square with a short edge `e23` on its right side, F3 = the
/// square to its right (shares `e12`, `e23`, `e34`), F2 = the square to its left (untouched).
fn strip() -> Strip {
    let mut t = CellComplex::new();
    let ids = with_op(&mut t, OpId(1), |t| {
        let p = [pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 0.5), pt(1.0, 0.5 + SHORT), pt(1.0, 1.0), pt(0.0, 1.0), pt(2.0, 0.0), pt(2.0, 1.0), pt(-1.0, 0.0), pt(-1.0, 1.0)];
        let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(*q).unwrap()).collect();
        let e = |t: &mut CellComplex, a: usize, b: usize| line_edge(t, v[a], v[b], p[a], p[b]);
        let (e01, e12, e23, e34, e45, e50) = (e(t, 0, 1), e(t, 1, 2), e(t, 2, 3), e(t, 3, 4), e(t, 4, 5), e(t, 5, 0));
        let (e1w, ew2, e24) = (e(t, 1, 6), e(t, 6, 7), e(t, 7, 4));
        let (el0, e5l, el1) = (e(t, 8, 0), e(t, 5, 9), e(t, 9, 8));
        let f = |e: EntityId, fwd: bool| CoedgeSpec { edge: e, sense: if fwd { Sense::Forward } else { Sense::Reversed } };
        let plane = SurfaceDef::Plane(Plane { frame: Frame::from_z(pt(0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap() });
        let surf = t.shapes_mut().unwrap().add_surface(plane).unwrap();
        let sc = Some(SurfaceCarrier { surface: surf, sense: Sense::Forward });
        let f1 = t.make_face(&[&[f(e01, true), f(e12, true), f(e23, true), f(e34, true), f(e45, true), f(e50, true)]], sc).unwrap();
        let f3 = t.make_face(&[&[f(e1w, true), f(ew2, true), f(e24, true), f(e34, false), f(e23, false), f(e12, false)]], sc).unwrap();
        let f2 = t.make_face(&[&[f(el0, true), f(e50, false), f(e5l, true), f(el1, true)]], sc).unwrap();
        Ok((v, [e01, e12, e23, e34, e45, e1w], [f1, f2, f3]))
    })
    .unwrap();
    let (v, e, faces) = ids;
    Strip { host: Arc::new(t), v: [v[0], v[1], v[2], v[3], v[4], v[5]], e01: e[0], e12: e[1], e23: e[2], e34: e[3], e45: e[4], e1w: e[5], faces }
}

fn host_dyn(s: &Strip) -> Arc<dyn TopologyView> {
    s.host.clone()
}

fn near(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}

fn vertex_point(v: &dyn TopologyView, id: EntityId) -> Point3 {
    match v.carrier(id) {
        Carrier::Point(p) => *p,
        other => panic!("not a vertex: {other:?}"),
    }
}

#[test]
fn composite_of_two_collinear_edges_hides_the_inner_vertex_and_replays_equal() {
    let s = strip();
    let h0 = s.host.canonical_hash();
    let mut ov = Overlay::new(host_dyn(&s));
    let id = ov.composite_edges(&[s.e23, s.e12]).unwrap();
    assert_eq!(ov.dim_of(id), Some(Dim::Edge));
    assert_eq!(ov.state(id), Some(EntityState::Live));
    assert_eq!(ov.resolve(id), &[s.e12, s.e23], "members in chain order");
    assert!(validate_geometry(&ov).is_empty(), "{:?}", validate_geometry(&ov));
    assert_eq!(ov.virtual_of(s.e12), Some(id));
    assert_eq!(ov.virtual_of(s.e23), Some(id));
    assert_eq!(VirtualOp::CompositeEdges { edges: vec![s.e12, s.e23], id }.created().as_slice(), &[id]);
    // Hidden: members and the inner vertex. Counts: 12 edges -> 11, 10 vertices -> 9.
    assert!(!ov.is_alive(s.e12) && !ov.is_alive(s.e23) && !ov.is_alive(s.v[2]));
    assert_eq!(ov.cells(Dim::Edge).len(), 11);
    assert_eq!(ov.cells(Dim::Vertex).len(), 9);
    assert_eq!(s.host.cells(Dim::Edge).len(), 12, "the host is untouched");
    assert_eq!(s.host.canonical_hash(), h0);
    // The virtual edge joins the two tips and both faces use it once.
    let ends: Vec<EntityId> = ov.boundary(id).iter().map(|e| e.cell).collect();
    assert_eq!(ends, vec![s.v[1], s.v[3]]);
    for f in [s.faces[0], s.faces[2]] {
        assert!(ov.face_edges(f).contains(&id) && !ov.face_edges(f).contains(&s.e12));
        assert_eq!(ov.coedges(f).count(), 5);
        assert_eq!(ov.loops(f).count(), 1);
    }
    assert!(ov.face_edges(s.faces[1]).iter().all(|e| ov.is_alive(*e)));
    // The two faces see the chain with opposite senses.
    let sense = |f: EntityId| ov.boundary(f).iter().find(|e| e.cell == id).map(|e| e.side).unwrap();
    assert_eq!(sense(s.faces[0]), Side::Left);
    assert_eq!(sense(s.faces[2]), Side::Right);
    // The sampler walks both members: ends on the tips, the middle on the hidden vertex.
    let sm = ov.edge_sampler(id).unwrap();
    assert!(near(sm.eval(0.0), pt(1.0, 0.0), 1e-12));
    // Shares of [0, 1] follow each member's arc length (here equal to its parameter span), not an even split.
    let (l12, l23) = ((vertex_point(&*s.host, s.v[2]) - vertex_point(&ov, s.v[1])).norm(), (vertex_point(&ov, s.v[3]) - vertex_point(&*s.host, s.v[2])).norm());
    let s_inner = l12 / (l12 + l23);
    assert!(s_inner > 0.99, "inner vertex sits near the far end, s = {s_inner}");
    assert!(near(sm.eval(s_inner), vertex_point(&*s.host, s.v[2]), 1e-9));
    assert!(near(sm.eval(1.0), vertex_point(&ov, s.v[3]), 1e-12));
    assert!(near(sm.eval(0.25), pt(1.0, 0.25 * (l12 + l23)), 1e-9));
    assert!(sm.eval(0.5).y < 0.5, "midpoint of s is inside e12, not on the sliver");
    // Replay through from_data reproduces ids, ops and the view.
    let data = ov.data();
    let back = Overlay::from_data(host_dyn(&s), data.clone()).unwrap();
    assert_eq!(back.ops(), ov.ops());
    assert_eq!(back.data(), data);
    assert_eq!(back.canonical_hash(), ov.canonical_hash());
    assert_eq!(back.resolve(id), ov.resolve(id));
    // The next minted edge id is fresh, also after replay.
    let mut a = ov.clone();
    let mut b = back;
    let (_, ha) = a.partition_edge(s.e45, 0.25).unwrap();
    let (_, hb) = b.partition_edge(s.e45, 0.25).unwrap();
    assert_eq!(ha, hb);
    assert!(!ha.contains(&id));
    // Remove restores the view.
    ov.remove(id).unwrap();
    assert_eq!(ov.canonical_hash(), h0);
    assert!(ov.is_alive(s.e12) && ov.is_alive(s.v[2]));
}

#[test]
fn composite_edges_validates_and_is_atomic() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let h0 = ov.canonical_hash();
    let chain = ov.composite_edges(&[s.e12, s.e23, s.e34]).unwrap();
    assert!(validate(&ov).is_empty(), "{:?}", validate(&ov));
    assert!(validate_geometry(&ov).is_empty(), "{:?}", validate_geometry(&ov));
    ov.remove(chain).unwrap();
    let mut ov = Overlay::new(host_dyn(&s));
    let serial = ov.data().next_serial;
    // Too few, not connected, a branch, an inner vertex of degree 3, a dead and a non-edge member.
    let cases: Vec<(&str, Vec<EntityId>)> = vec![
        ("one edge", vec![s.e12]),
        ("disconnected", vec![s.e12, s.e45]),
        ("branch at v1", vec![s.e01, s.e12, s.e1w]),
        ("inner vertex of degree 3", vec![s.e01, s.e12]),
        ("a face", vec![s.e12, s.faces[0]]),
    ];
    for (what, edges) in cases {
        let err = ov.composite_edges(&edges).unwrap_err();
        assert!(matches!(err, TopoError::InvalidParameter(_) | TopoError::Unsupported(_) | TopoError::WrongDim { .. }), "{what}: {err:?}");
        assert!(ov.ops().is_empty(), "{what}: failed op left a record");
        assert_eq!(ov.data().next_serial, serial, "{what}: counters moved");
        assert_eq!(ov.canonical_hash(), h0, "{what}: view changed");
    }
    // The specific refusals carry their reason.
    assert!(matches!(ov.composite_edges(&[s.e12, s.e45]), Err(TopoError::InvalidParameter(m)) if m.contains("connected")));
    assert!(matches!(ov.composite_edges(&[s.e01, s.e12, s.e1w]), Err(TopoError::Unsupported(m)) if m.contains("branches")));
    assert!(matches!(ov.composite_edges(&[s.e01, s.e12]), Err(TopoError::Unsupported(m)) if m.contains("degree")));
    // A host edge already absorbed cannot join a second composite.
    let c = ov.composite_edges(&[s.e12, s.e23]).unwrap();
    assert!(matches!(ov.composite_edges(&[s.e12, s.e23]), Err(TopoError::Dead(_))));
    assert!(matches!(ov.composite_edges(&[c, s.e34]), Err(TopoError::WrongDim { .. } | TopoError::Dead(_))), "a virtual member is refused");
}

#[test]
fn collapse_merges_the_vertices_with_a_recorded_gap_and_blended_samplers() {
    let s = strip();
    let h0 = s.host.canonical_hash();
    let mut ov = Overlay::new(host_dyn(&s));
    let linear = ov.tolerance().linear;
    let c = ov.collapse_edge(s.e23, s.v[2]).unwrap();
    assert_eq!(c.dropped_vertex, s.v[3]);
    assert!((c.gap - SHORT).abs() < 1e-12, "gap {} vs edge length {SHORT}", c.gap);
    assert_eq!(c.repointed.len(), 1);
    let (orig, copy) = c.repointed[0];
    assert_eq!(orig, s.e34);
    // The op records the allocated ids and the gap; created() lists the copies.
    let VirtualOp::CollapseEdge { edge, into_vertex, gap, repointed } = &ov.ops()[0] else { panic!("op") };
    assert_eq!((*edge, *into_vertex, repointed.as_slice()), (s.e23, s.v[2], &[(s.e34, copy)][..]));
    assert!((gap - SHORT).abs() < 1e-12);
    assert_eq!(ov.ops()[0].created().as_slice(), &[copy]);
    // View: the short edge and the dropped vertex are gone, the copy runs v2 -> v4.
    assert!(!ov.is_alive(s.e23) && !ov.is_alive(s.v[3]) && !ov.is_alive(s.e34));
    assert_eq!(ov.cells(Dim::Vertex).len(), 9);
    assert_eq!(ov.cells(Dim::Edge).len(), 11);
    assert_eq!(ov.live_cover(s.v[3]), vec![s.v[2]], "the dropped vertex maps to the survivor");
    assert_eq!(ov.virtual_of(s.v[3]), Some(s.v[2]));
    let ends: Vec<EntityId> = ov.boundary(copy).iter().map(|e| e.cell).collect();
    assert_eq!(ends, vec![s.v[2], s.v[4]]);
    for f in [s.faces[0], s.faces[2]] {
        assert!(ov.face_edges(f).contains(&copy) && !ov.face_edges(f).contains(&s.e23) && !ov.face_edges(f).contains(&s.e34));
        assert_eq!(ov.coedges(f).count(), 5);
        assert_eq!(ov.loops(f).count(), 1);
    }
    assert_eq!(ov.coboundary(s.v[2]).iter().map(|e| e.cell).collect::<Vec<_>>(), vec![s.e12, copy], "the survivor now ends the copy");
    // Tolerance: touched cells carry max(own, gap); untouched cells are unchanged.
    for t in [s.faces[0], s.faces[2], s.v[2], copy] {
        assert!(ov.tolerance_of(t) >= c.gap, "{t:?}");
        assert!(ov.tolerance_of(t) <= ov.tolerance().linear_max);
    }
    for u in [s.faces[1], s.v[0], s.v[1], s.v[4], s.e01, s.e45, s.e12, s.e1w] {
        assert_eq!(ov.tolerance_override(u), s.host.tolerance_override(u), "{u:?}");
        assert_eq!(ov.tolerance_of(u), linear);
    }
    // Samplers: the copy's ends land on the survivor and on v4; the middle is blended.
    let sm = ov.edge_sampler(copy).unwrap();
    assert!(near(sm.eval(0.0), pt(1.0, 0.5), 1e-12));
    assert!(near(sm.eval(1.0), pt(1.0, 1.0), 1e-12));
    let raw_mid = 0.5 * (0.5 + SHORT + 1.0);
    assert!(near(sm.eval(0.5), pt(1.0, raw_mid - 0.5 * SHORT), 1e-12));
    assert!(ov.edge_sampler(s.e34).is_none(), "a hidden edge has no sampler");
    let plain = ov.edge_sampler(s.e45).unwrap();
    assert!(near(plain.eval(0.0), pt(1.0, 1.0), 1e-12) && near(plain.eval(1.0), pt(0.0, 1.0), 1e-12));
    // The tolerance makes the view valid; the host is untouched.
    assert!(validate(&ov).is_empty(), "{:?}", validate(&ov));
    assert!(validate_geometry(&ov).is_empty(), "{:?}", validate_geometry(&ov));
    assert_eq!(s.host.canonical_hash(), h0);
    // An edge whose end a collapse moved cannot be partitioned (its sampler would lose the blend).
    assert!(matches!(ov.clone().partition_edge(copy, 0.1), Err(TopoError::Unsupported(_))));
    // Replay reproduces ids, gap, tolerances and the hash.
    let back = Overlay::from_data(host_dyn(&s), ov.data()).unwrap();
    assert_eq!(back.ops(), ov.ops());
    assert_eq!(back.canonical_hash(), ov.canonical_hash());
    assert_eq!(back.tolerance_of(s.faces[0]), ov.tolerance_of(s.faces[0]));
    let sb = back.edge_sampler(copy).unwrap();
    assert!(near(sb.eval(0.5), sm.eval(0.5), 0.0));
    // Remove by the collapsed edge or by the copy restores the host view.
    for victim in [s.e23, copy] {
        let mut o = ov.clone();
        o.remove(victim).unwrap();
        assert_eq!(o.canonical_hash(), h0, "{victim:?}");
        assert_eq!(o.tolerance_override(s.faces[0]), None);
    }
    // clear() restores it too.
    ov.clear();
    assert_eq!(ov.canonical_hash(), h0);
}

#[test]
fn collapse_other_direction_and_refusals() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let h0 = ov.canonical_hash();
    // Into the other end: v3 survives, e12 is re-pointed to end on it.
    let c = ov.collapse_edge(s.e23, s.v[3]).unwrap();
    assert_eq!(c.dropped_vertex, s.v[2]);
    let (orig, copy) = c.repointed[0];
    assert_eq!(orig, s.e12);
    assert_eq!(ov.boundary(copy).iter().map(|e| e.cell).collect::<Vec<_>>(), vec![s.v[1], s.v[3]]);
    let sm = ov.edge_sampler(copy).unwrap();
    assert!(near(sm.eval(0.0), pt(1.0, 0.0), 1e-12));
    assert!(near(sm.eval(1.0), pt(1.0, 0.5 + SHORT), 1e-12));
    ov.clear();

    let serial = ov.data().next_serial;
    let long = ov.collapse_edge(s.e01, s.v[0]).unwrap_err();
    assert!(matches!(long, TopoError::ToleranceTooLarge { wanted, max } if wanted > max && (wanted - 1.0).abs() < 1e-9), "{long:?}");
    assert!(matches!(ov.collapse_edge(s.e23, s.v[0]), Err(TopoError::InvalidParameter(_))), "not an end");
    assert!(matches!(ov.collapse_edge(s.faces[0], s.v[0]), Err(TopoError::WrongDim { .. })));
    assert!(ov.ops().is_empty());
    assert_eq!(ov.data().next_serial, serial);
    assert_eq!(ov.canonical_hash(), h0);
}

#[test]
fn collapse_next_to_a_composite_edge_is_refused_atomically() {
    // Wire chain far -eA- mid -eB- drop -eS(short)- into; composite [eA, eB] ends on `drop`.
    let mut t = CellComplex::new();
    let (v, e) = with_op(&mut t, OpId(1), |t| {
        let p = [pt(-1.0, 0.0), pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0 + SHORT, 0.0)];
        let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(*q).unwrap()).collect();
        let e: Vec<EntityId> = (0..3).map(|i| line_edge(t, v[i], v[i + 1], p[i], p[i + 1])).collect();
        Ok((v, e))
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t) as Arc<dyn TopologyView>);
    ov.composite_edges(&[e[0], e[1]]).unwrap();
    let h = ov.canonical_hash();
    let (n, serial) = (ops_len(&ov), ov.data().next_serial);
    let err = ov.collapse_edge(e[2], v[3]).unwrap_err();
    assert!(matches!(err, TopoError::Unsupported(m) if m.contains("composite")), "{err:?}");
    assert_eq!((ops_len(&ov), ov.data().next_serial, ov.canonical_hash()), (n, serial, h));
}

fn ops_len(ov: &Overlay) -> usize {
    ov.ops().len()
}

#[test]
fn a_second_collapse_across_a_copy_is_judged_on_the_absolute_gap() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let copy = ov.collapse_edge(s.e23, s.v[2]).unwrap().repointed[0].1;
    // The copy is 0.5 long: far above linear_max, refused, and the first collapse stands.
    let err = ov.collapse_edge(copy, s.v[2]).unwrap_err();
    assert!(matches!(err, TopoError::ToleranceTooLarge { wanted, .. } if wanted > 0.49), "{err:?}");
    assert_eq!(ov.ops().len(), 1);
    assert!(ov.is_alive(copy));
}

#[test]
fn write_canonical_separates_every_op_field_and_survives_replay() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    ov.collapse_edge(s.e23, s.v[2]).unwrap();
    ov.partition_edge(s.e45, 0.25).unwrap();
    let q = ov.tolerance().quantum();
    let tol = *ov.tolerance();
    let bytes = |d: &OverlayData| {
        let mut out = Vec::new();
        d.write_canonical(&mut out, &tol);
        out
    };
    let data = ov.data();
    let base = bytes(&data);
    // Deterministic, and equal after the from_data round trip.
    assert_eq!(bytes(&data), base);
    let back = Overlay::from_data(host_dyn(&s), data.clone()).unwrap();
    assert_eq!(bytes(&back.data()), base);

    // One of every variant with distinct, recognisable field values.
    let id = |k: u64, n: u64| EntityId::new(k_of(k), n).unwrap();
    let line = |x: f64| CurveDef::Line(Line::new(pt(x, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    let plane = |z: f64| SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, z), Vector3::new(0.0, 0.0, 1.0)).unwrap() });
    let all = |p: f64| -> Vec<VirtualOp> {
        vec![
            VirtualOp::Composite { faces: vec![id(2, 1), id(2, 2)], id: id(5, 1) },
            VirtualOp::PartitionEdge { edge: id(1, 1), param: p, vertex: id(3, 1), halves: [id(4, 1), id(4, 2)] },
            VirtualOp::PartitionFace { face: id(2, 3), cut: VirtualCut { from: id(0, 1), to: id(0, 2), curve: line(p), range: Interval::new(0.0, p) }, chord: id(4, 3), halves: [id(5, 2), id(5, 3)] },
            VirtualOp::PartitionVolume { volume: id(6, 1), cut: VolumeCut { edges: vec![id(1, 4), id(1, 5)], surface: plane(p), seed: id(2, 9) }, face: id(5, 4), children: [id(7, 1), id(7, 2)] },
            VirtualOp::CompositeEdges { edges: vec![id(1, 7), id(1, 8)], id: id(4, 9) },
            VirtualOp::CollapseEdge { edge: id(1, 10), into_vertex: id(0, 3), gap: p, repointed: vec![(id(1, 11), id(4, 10))] },
        ]
    };
    let mk = |ops: Vec<VirtualOp>| OverlayData::new(OpId(3), ops, [4, 5, 6]);
    let reference = bytes(&mk(all(0.5)));
    assert_eq!(bytes(&mk(all(0.5))), reference);
    let n = all(0.5).len();
    // Lengths (the gap) and the shape definitions go by the length quantum; edge parameters and
    // cut ranges by the (finer) parameter quantum, like the geometry digests.
    let pq = tol.param_quantum();
    assert!(pq < q);
    assert_eq!(bytes(&mk(all(0.5 + 0.1 * pq))), reference);
    assert_ne!(bytes(&mk(all(0.5 + 8.0 * pq))), reference);
    assert_ne!(bytes(&mk(all(0.5 + 3e-10))), reference, "3e-10 is well above the parameter quantum");
    // Mutating any single field of any op changes the bytes.
    let mut seen = vec![reference.clone()];
    let mut check = |what: String, d: OverlayData| {
        let b = bytes(&d);
        assert!(!seen.contains(&b), "{what}: digest did not change");
        seen.push(b);
    };
    check("host revision".into(), OverlayData::new(OpId(4), all(0.5), [4, 5, 6]));
    for k in 0..3 {
        let mut ns = [4, 5, 6];
        ns[k] += 1;
        check(format!("next_serial[{k}]"), OverlayData::new(OpId(3), all(0.5), ns));
    }
    check("fewer ops".into(), mk(all(0.5)[..n - 1].to_vec()));
    let mut swapped = all(0.5);
    swapped.swap(0, 1);
    check("op order".into(), mk(swapped));
    let bump = |e: EntityId| EntityId::new(e.kind(), e.serial() + 100).unwrap();
    for i in 0..n {
        let tweak = |f: &dyn Fn(&mut VirtualOp)| {
            let mut ops = all(0.5);
            f(&mut ops[i]);
            mk(ops)
        };
        match i {
            0 => {
                check("composite.faces".into(), tweak(&|o| if let VirtualOp::Composite { faces, .. } = o { faces[1] = bump(faces[1]) }));
                check("composite.id".into(), tweak(&|o| if let VirtualOp::Composite { id, .. } = o { *id = bump(*id) }));
            }
            1 => {
                check("pe.edge".into(), tweak(&|o| if let VirtualOp::PartitionEdge { edge, .. } = o { *edge = bump(*edge) }));
                check("pe.param".into(), tweak(&|o| if let VirtualOp::PartitionEdge { param, .. } = o { *param = 0.75 }));
                check("pe.vertex".into(), tweak(&|o| if let VirtualOp::PartitionEdge { vertex, .. } = o { *vertex = bump(*vertex) }));
                check("pe.halves".into(), tweak(&|o| if let VirtualOp::PartitionEdge { halves, .. } = o { halves[1] = bump(halves[1]) }));
            }
            2 => {
                check("pf.face".into(), tweak(&|o| if let VirtualOp::PartitionFace { face, .. } = o { *face = bump(*face) }));
                check("pf.from".into(), tweak(&|o| if let VirtualOp::PartitionFace { cut, .. } = o { cut.from = bump(cut.from) }));
                check("pf.to".into(), tweak(&|o| if let VirtualOp::PartitionFace { cut, .. } = o { cut.to = bump(cut.to) }));
                check("pf.curve".into(), tweak(&|o| if let VirtualOp::PartitionFace { cut, .. } = o { cut.curve = line(3.0) }));
                check("pf.range.lo".into(), tweak(&|o| if let VirtualOp::PartitionFace { cut, .. } = o { cut.range.lo = 0.25 }));
                check("pf.range.hi".into(), tweak(&|o| if let VirtualOp::PartitionFace { cut, .. } = o { cut.range.hi = 0.75 }));
                check("pf.chord".into(), tweak(&|o| if let VirtualOp::PartitionFace { chord, .. } = o { *chord = bump(*chord) }));
                check("pf.halves".into(), tweak(&|o| if let VirtualOp::PartitionFace { halves, .. } = o { halves[0] = bump(halves[0]) }));
            }
            3 => {
                check("pv.volume".into(), tweak(&|o| if let VirtualOp::PartitionVolume { volume, .. } = o { *volume = bump(*volume) }));
                check("pv.edges".into(), tweak(&|o| if let VirtualOp::PartitionVolume { cut, .. } = o { cut.edges.reverse() }));
                check("pv.surface".into(), tweak(&|o| if let VirtualOp::PartitionVolume { cut, .. } = o { cut.surface = plane(2.0) }));
                check("pv.seed".into(), tweak(&|o| if let VirtualOp::PartitionVolume { cut, .. } = o { cut.seed = bump(cut.seed) }));
                check("pv.face".into(), tweak(&|o| if let VirtualOp::PartitionVolume { face, .. } = o { *face = bump(*face) }));
                check("pv.children".into(), tweak(&|o| if let VirtualOp::PartitionVolume { children, .. } = o { children[1] = bump(children[1]) }));
            }
            4 => {
                check("ce.edges".into(), tweak(&|o| if let VirtualOp::CompositeEdges { edges, .. } = o { edges[0] = bump(edges[0]) }));
                check("ce.id".into(), tweak(&|o| if let VirtualOp::CompositeEdges { id, .. } = o { *id = bump(*id) }));
            }
            _ => {
                check("collapse.repointed.original".into(), tweak(&|o| if let VirtualOp::CollapseEdge { repointed, .. } = o { repointed[0].0 = bump(repointed[0].0) }));
                check("collapse.repointed.copy".into(), tweak(&|o| if let VirtualOp::CollapseEdge { repointed, .. } = o { repointed[0].1 = bump(repointed[0].1) }));
                check("collapse.edge".into(), tweak(&|o| if let VirtualOp::CollapseEdge { edge, .. } = o { *edge = bump(*edge) }));
                check("collapse.into".into(), tweak(&|o| if let VirtualOp::CollapseEdge { into_vertex, .. } = o { *into_vertex = bump(*into_vertex) }));
                check("collapse.gap".into(), tweak(&|o| if let VirtualOp::CollapseEdge { gap, .. } = o { *gap = 0.9 }));
                check("collapse.repointed".into(), tweak(&|o| if let VirtualOp::CollapseEdge { repointed, .. } = o { repointed.push((id(1, 12), id(4, 11))) }));
            }
        }
    }
    // The empty record is distinct from any non-empty one.
    assert_ne!(bytes(&OverlayData::new(OpId(3), Vec::new(), [4, 5, 6])), reference);
}

fn k_of(k: u64) -> sm_core::EntityKind {
    use sm_core::EntityKind as K;
    [K::TopoVertex, K::TopoEdge, K::TopoFace, K::PartitionVertex, K::PartitionEdge, K::PartitionFace, K::TopoVolume, K::PartitionCell][k as usize]
}

#[test]
fn building_twice_gives_the_same_bytes() {
    let run = || {
        let s = strip();
        let mut ov = Overlay::new(host_dyn(&s));
        ov.composite_edges(&[s.e12, s.e23]).unwrap();
        ov.collapse_edge(s.e34, s.v[4]).ok();
        let mut out = Vec::new();
        ov.data().write_canonical(&mut out, ov.tolerance());
        (out, ov.canonical_hash())
    };
    assert_eq!(run(), run());
}

#[cfg(feature = "serde")]
#[test]
fn existing_overlay_data_serialises_as_before() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    ov.partition_edge(s.e45, 0.25).unwrap();
    let json = serde_json::to_string(&ov.data()).unwrap();
    // Variants are externally tagged and the new ones do not disturb the old shapes.
    assert_eq!(json, r#"{"host_revision":1,"ops":[{"PartitionEdge":{"edge":"TopoEdge#5","param":0.25,"vertex":"PartitionVertex#1","halves":["PartitionEdge#1","PartitionEdge#2"]}}],"next_serial":[2,3,1]}"#);
    let back: OverlayData = serde_json::from_str(&json).unwrap();
    assert_eq!(back, ov.data());
    // The new variants round-trip and use the same tagging.
    let mut ov2 = Overlay::new(host_dyn(&s));
    ov2.composite_edges(&[s.e12, s.e23]).unwrap();
    let j2 = serde_json::to_string(&ov2.data()).unwrap();
    assert!(j2.contains(r#""CompositeEdges":{"edges":"#), "{j2}");
    assert_eq!(serde_json::from_str::<OverlayData>(&j2).unwrap(), ov2.data());
}

#[test]
fn the_hook_reaches_the_blend_through_a_dyn_view() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let copy = ov.collapse_edge(s.e23, s.v[2]).unwrap().repointed[0].1;
    let view: &dyn TopologyView = &ov;
    let [o0, o1] = view.edge_end_offsets(copy).expect("a collapse copy has end offsets");
    // Raw curve ends are (1, 0.5 + SHORT) and (1, 1); the vertices are (1, 0.5) and (1, 1).
    assert!(near(pt(0.0, 0.0) + o0, pt(0.0, -SHORT), 1e-12), "{o0:?}");
    assert!(near(pt(0.0, 0.0) + o1, pt(0.0, 0.0), 1e-12), "{o1:?}");
    // Applying the hook to the carrier's raw ends gives the reference sampler's ends.
    let smp = ov.edge_sampler(copy).unwrap();
    assert!(near(smp.eval(0.0), pt(1.0, 0.5 + SHORT) + o0, 1e-12));
    assert!(near(smp.eval(1.0), pt(1.0, 1.0) + o1, 1e-12));
    // Plain host edges and the host view itself report none.
    assert!(view.edge_end_offsets(s.e45).is_none());
    assert!(s.host.edge_end_offsets(s.e45).is_none());
}

#[test]
fn a_chained_collapse_keeps_the_dependency_on_the_copy_it_re_points() {
    let s = strip();
    let h0 = s.host.canonical_hash();
    let mut ov = Overlay::new(host_dyn(&s));
    let (p, halves) = ov.partition_edge(s.e12, 0.4999).unwrap();
    let c1 = ov.collapse_edge(s.e23, s.v[2]).unwrap();
    let copy3 = c1.repointed[0].1;
    assert_eq!(c1.repointed[0].0, s.e34);
    let c2 = ov.collapse_edge(halves[1], p).unwrap();
    assert_eq!(c2.repointed.iter().filter(|(o, _)| *o == copy3).count(), 1, "{:?}", c2.repointed);
    let VirtualOp::CollapseEdge { repointed, .. } = &ov.ops()[2] else { panic!("op") };
    assert!(repointed.iter().any(|(o, _)| *o == copy3));
    // The first collapse's copy is an input of the second: it cannot be removed under it.
    assert!(matches!(ov.clone().remove(copy3), Err(TopoError::StillReferenced(_))));
    assert!(matches!(ov.clone().remove(s.e23), Err(TopoError::StillReferenced(_))));
    // Replay is faithful and validates.
    let back = Overlay::from_data(host_dyn(&s), ov.data()).unwrap();
    assert_eq!(back.ops(), ov.ops());
    assert!(validate(&back).is_empty());
    // Replay refuses a record whose originals differ, even with the same count.
    let mut bad = ov.data();
    if let VirtualOp::CollapseEdge { repointed, .. } = &mut bad.ops[2] {
        repointed[0].0 = s.e45;
    }
    assert!(Overlay::from_data(host_dyn(&s), bad).is_err());
    // Removing in reverse order works and restores the host view.
    ov.remove(c2.repointed[0].1).unwrap();
    ov.remove(s.e23).unwrap();
    ov.remove(p).unwrap();
    assert_eq!(ov.canonical_hash(), h0);
}

/// Quarter arc (R = 10, angle parameter) then a unit line traversed against its own direction.
fn arc_line() -> (Arc<CellComplex>, [EntityId; 3], [EntityId; 2]) {
    let mut t = CellComplex::new();
    let (v, e) = with_op(&mut t, OpId(1), |t| {
        let p = [pt(10.0, 0.0), pt(0.0, 10.0), pt(0.0, 11.0)];
        let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(*q).unwrap()).collect();
        let frame = Frame::from_z(pt(0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
        let arc = t.shapes_mut().unwrap().add_curve(CurveDef::Circle(sm_geom_api::Circle::new(frame, 10.0).unwrap())).unwrap();
        let ea = t.make_edge(v[0], v[1], Some(CurveCarrier { curve: arc, range: Interval::new(0.0, std::f64::consts::FRAC_PI_2), sense: Sense::Forward })).unwrap();
        // Edge runs v2 -> v1 along the line from (0, 11) down to (0, 10); the chain walks it v1 -> v2.
        let el = line_edge(t, v[2], v[1], p[2], p[1]);
        Ok((v, [ea, el]))
    })
    .unwrap();
    (Arc::new(t), [v[0], v[1], v[2]], e)
}

#[test]
fn composite_sampler_weights_members_by_arc_length_not_parameter_span() {
    let (host, v, [ea, el]) = arc_line();
    let mut ov = Overlay::new(host as Arc<dyn TopologyView>);
    let id = ov.composite_edges(&[el, ea]).unwrap();
    let ends: Vec<EntityId> = ov.boundary(id).iter().map(|e| e.cell).collect();
    assert_eq!(ends, vec![v[0], v[2]]);
    let l_arc = 10.0 * std::f64::consts::FRAC_PI_2;
    let l_line = 1.0;
    let s_in = l_arc / (l_arc + l_line);
    let sm = ov.edge_sampler(id).unwrap();
    assert!(near(sm.eval(0.0), pt(10.0, 0.0), 1e-12));
    assert!(near(sm.eval(1.0), pt(0.0, 11.0), 1e-12));
    assert!(near(sm.eval(s_in), pt(0.0, 10.0), 1e-9), "{:?}", sm.eval(s_in));
    // Halfway through the line member (the backward member): the point (0, 10.5).
    assert!(near(sm.eval(s_in + 0.5 * (1.0 - s_in)), pt(0.0, 10.5), 1e-9));
    // Halfway through the arc: angle pi/4.
    let q = 10.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!(near(sm.eval(0.5 * s_in), pt(q, q), 1e-9));
}

#[test]
fn edge_sampler_is_reachable_through_a_dyn_view() {
    // Composite through the trait object, equal to the inherent sampler.
    let (host, _, [ea, el]) = arc_line();
    let mut ov = Overlay::new(host as Arc<dyn TopologyView>);
    let id = ov.composite_edges(&[ea, el]).unwrap();
    let view: &dyn TopologyView = &ov;
    let (d, i) = (view.edge_sampler(id).expect("dyn composite sampler"), ov.edge_sampler(id).unwrap());
    for k in 0..=20 {
        let s = k as f64 / 20.0;
        assert_eq!(d.eval(s), i.eval(s));
    }
    assert!(view.edge_sampler(ea).is_none(), "hidden member has no sampler");
    // Collapse copy through the trait object.
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let copy = ov.collapse_edge(s.e23, s.v[2]).unwrap().repointed[0].1;
    let view: &dyn TopologyView = &ov;
    let (d, i) = (view.edge_sampler(copy).unwrap(), ov.edge_sampler(copy).unwrap());
    for k in 0..=10 {
        assert_eq!(d.eval(k as f64 / 10.0), i.eval(k as f64 / 10.0));
    }
    assert!(near(d.eval(0.0), pt(1.0, 0.5), 1e-12));
    // The default impl on the host view: plain edges only, faces and vertices none.
    let hv: &dyn TopologyView = &*s.host;
    let plain = hv.edge_sampler(s.e45).unwrap();
    assert!(near(plain.eval(0.5), pt(0.5, 1.0), 1e-12));
    assert!(hv.edge_sampler(s.faces[0]).is_none());
    assert!(hv.edge_sampler(s.v[0]).is_none());
}

#[test]
fn sampler_length_and_tangent_match_analytic_values() {
    let (host, _, [ea, el]) = arc_line();
    let mut ov = Overlay::new(host as Arc<dyn TopologyView>);
    let id = ov.composite_edges(&[ea, el]).unwrap();
    let sm = ov.edge_sampler(id).unwrap();
    let l = 10.0 * std::f64::consts::FRAC_PI_2 + 1.0;
    assert!((sm.length() - l).abs() < 1e-12, "{}", sm.length());
    // Tangent against a central finite difference inside each member.
    let s_in = 10.0 * std::f64::consts::FRAC_PI_2 / l;
    for s in [0.2 * s_in, 0.7 * s_in, s_in + 0.3 * (1.0 - s_in)] {
        let h = 1e-6;
        let fd = (sm.eval(s + h) - sm.eval(s - h)) * (1.0 / (2.0 * h));
        assert!((sm.tangent(s) - fd).norm() < 1e-5 * l, "{s}: {:?} vs {fd:?}", sm.tangent(s));
        // Length-fraction parametrization: |d eval / d s| is the total length.
        assert!((sm.tangent(s).norm() - l).abs() < 1e-9);
    }
    // Collapse copy: tangent includes the end-offset blend term.
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let copy = ov.collapse_edge(s.e23, s.v[2]).unwrap().repointed[0].1;
    let sm = ov.edge_sampler(copy).unwrap();
    assert!((sm.length() - (0.5 - SHORT)).abs() < 1e-12);
    for k in 1..10 {
        let u = k as f64 / 10.0;
        let h = 1e-6;
        let fd = (sm.eval(u + h) - sm.eval(u - h)) * (1.0 / (2.0 * h));
        assert!((sm.tangent(u) - fd).norm() < 1e-8, "{u}");
    }
    let plain = ov.edge_sampler(s.e45).unwrap();
    assert!((plain.length() - 1.0).abs() < 1e-12);
}

#[test]
fn composite_faces_inherit_the_collapse_raised_tolerance() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let c = ov.collapse_edge(s.e23, s.v[2]).unwrap();
    let linear = ov.tolerance().linear;
    let vf = ov.composite_faces(&[s.faces[0], s.faces[2]]).unwrap();
    assert!(ov.tolerance_override(vf).is_some_and(|t| t >= c.gap), "{:?}", ov.tolerance_override(vf));
    assert!(ov.tolerance_of(vf) >= c.gap && ov.tolerance_of(vf) > linear);
    let back = Overlay::from_data(host_dyn(&s), ov.data()).unwrap();
    assert_eq!(back.tolerance_of(vf), ov.tolerance_of(vf));
    // Untouched members give a plain composite (no override).
    let s2 = strip();
    let mut ov2 = Overlay::new(host_dyn(&s2));
    let v2 = ov2.composite_faces(&[s2.faces[0], s2.faces[2]]).unwrap();
    assert_eq!(ov2.tolerance_override(v2), None);
}

#[test]
fn composite_edges_refuses_a_collapse_copy() {
    let s = strip();
    let mut ov = Overlay::new(host_dyn(&s));
    let copy = ov.collapse_edge(s.e23, s.v[2]).unwrap().repointed[0].1;
    let n = ov.ops().len();
    let r = ov.composite_edges(&[s.e12, copy]);
    assert!(matches!(r, Err(TopoError::WrongDim { .. } | TopoError::Dead(_))), "{r:?}");
    assert_eq!(ov.ops().len(), n);
}

#[test]
fn composite_edge_inherits_a_member_tolerance_override_and_replays() {
    let mut s = strip();
    let t = 5e-6;
    with_op(Arc::get_mut(&mut s.host).unwrap(), OpId(2), |h| Ok(h.raise_tolerance(s.e12, t)?)).unwrap();
    let mut ov = Overlay::new(host_dyn(&s));
    assert!(t > ov.tolerance().linear);
    let id = ov.composite_edges(&[s.e12, s.e23]).unwrap();
    assert_eq!(ov.tolerance_override(id), Some(t));
    let back = Overlay::from_data(host_dyn(&s), ov.data()).unwrap();
    assert_eq!(back.tolerance_override(id), Some(t));
}
