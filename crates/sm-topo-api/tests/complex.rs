//! Reference-implementor tests: brick, merged bricks, face split, cylinder seam, baffle,
//! determinism, tolerance, instancing, overlay.

use core::f64::consts::TAU;
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_geom_api::{CurveDef, GeomStore, Interval, Line, ParametricCurve, SimpleGeomStore};
use sm_topo_api::builders::{BrickIds, CylinderIds, make_brick, make_cylinder};
use sm_topo_api::*;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn count(c: &dyn TopologyView, k: EntityKind) -> i64 {
    c.count(k) as i64
}
fn euler(c: &dyn TopologyView) -> i64 {
    count(c, EntityKind::TopoVertex) - count(c, EntityKind::TopoEdge) + count(c, EntityKind::TopoFace)
}

fn one_brick() -> (Complex, SimpleGeomStore, BrickIds) {
    let mut c = Complex::default();
    let mut g = SimpleGeomStore::new();
    c.begin_op(OpId(1)).unwrap();
    let b = make_brick(&mut c, &mut g, p(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0), None).unwrap();
    c.end_op().unwrap();
    (c, g, b)
}

fn two_bricks() -> (Complex, SimpleGeomStore, BrickIds, BrickIds) {
    let (mut c, mut g, a) = one_brick();
    c.begin_op(OpId(2)).unwrap();
    let b = make_brick(&mut c, &mut g, p(1.0, 0.0, 0.0), v(1.0, 1.0, 1.0), None).unwrap();
    c.end_op().unwrap();
    (c, g, a, b)
}

/// Imprint is trivial (faces already coincide exactly); merge vertices, edges, then the face.
fn merge_shared_face(c: &mut Complex, a: &BrickIds, b: &BrickIds) -> FaceId {
    c.begin_op(OpId(3)).unwrap();
    for (ia, ib) in [(1, 0), (3, 2), (5, 4), (7, 6)] {
        c.merge_vertices(a.vertices[ia], b.vertices[ib]).unwrap();
    }
    // +x face of A: edges (1,3)=y@z0, (3,7)=z@y1, (7,5)=y@z1, (5,1)=z@y0; -x face of B likewise.
    for (ea, eb) in [(5, 4), (11, 10), (7, 6), (9, 8)] {
        c.merge_edges(a.edges[ea], b.edges[eb]).unwrap();
    }
    c.set_coincidence(a.faces[1], b.faces[0], ImprintStatus::ImprintedNotMerged, Some(1.0)).unwrap();
    let survivor = c.merge_faces(a.faces[1], b.faces[0]).unwrap();
    c.end_op().unwrap();
    survivor
}

#[test]
fn brick_is_a_closed_cell_with_two_sided_faces() {
    let (c, g, b) = one_brick();
    assert_eq!((count(&c, EntityKind::TopoVertex), count(&c, EntityKind::TopoEdge), count(&c, EntityKind::TopoFace)), (8, 12, 6));
    assert_eq!(euler(&c), 2);
    assert_eq!(validate(&c), vec![]);
    assert!(check_edge_geometry(&c, &g).is_empty());
    // ids are canonical: serials follow creation order
    assert_eq!(b.vertices[7].serial(), 7);
    assert_eq!(b.faces[5].serial(), 5);
    for f in b.faces {
        let sides = c.face_sides(f).unwrap();
        assert_eq!(sides, FaceSides { front: Side::Exterior, back: Side::Volume(b.volume) });
        assert_eq!(c.face_role(f), Some(FaceRole::Boundary));
    }
    // every edge: two coedges in two faces, opposite senses (manifold)
    for e in b.edges {
        let cos = c.edge_coedges(e);
        assert_eq!(cos.len(), 2);
        let senses: Vec<_> = cos.iter().map(|r| c.coedge(*r).unwrap().sense).collect();
        assert_ne!(senses[0], senses[1]);
        assert_eq!(c.edge_faces(e).len(), 2);
    }
    for vx in b.vertices {
        assert_eq!(c.vertex_edges(vx).len(), 3);
    }
    assert_eq!(c.volume_faces(b.volume).len(), 6);
    assert_eq!(c.lineage().records().len(), 8 + 12 + 6 + 1 + 1);
    // planar face normal agrees with the topological outward sense (+x face)
    let f = c.face(b.faces[1]).unwrap();
    let plane = g.surface(f.surface.unwrap()).unwrap().as_parametric().unwrap();
    assert!(plane.normal(0.0, 0.0).unwrap().dot(v(1.0, 0.0, 0.0)) > 0.99);
}

#[test]
fn two_bricks_imprinted_and_merged_share_one_face() {
    let (mut c, _g, a, b) = two_bricks();
    assert_eq!(euler(&c), 4); // two disjoint shells
    let before_hash = topology_hash(&c);
    // non-coincident faces cannot merge, and a failed edit changes nothing
    c.begin_op(OpId(9)).unwrap();
    assert!(matches!(c.merge_faces(a.faces[1], b.faces[1]), Err(TopoError::NotCoincident(..))));
    assert!(matches!(c.merge_edges(a.edges[5], b.edges[4]), Err(TopoError::NotCoincident(..))), "vertices must merge first");
    c.end_op().unwrap();
    assert_eq!(topology_hash(&c), before_hash, "failed edits are atomic");
    let survivor = merge_shared_face(&mut c, &a, &b);

    assert_eq!(survivor, a.faces[1], "lower id survives");
    assert!(!c.is_alive(b.faces[0].id()));
    assert_eq!((count(&c, EntityKind::TopoVertex), count(&c, EntityKind::TopoEdge), count(&c, EntityKind::TopoFace)), (12, 20, 11));
    assert_eq!(validate(&c), vec![]);

    // two sides, opposite FaceUse senses
    let sides = c.face_sides(survivor).unwrap();
    assert_eq!(sides, FaceSides { front: Side::Volume(b.volume), back: Side::Volume(a.volume) });
    assert_eq!(c.face_role(survivor), Some(FaceRole::Interface));
    let uses = c.face_uses(survivor);
    assert_eq!(uses.len(), 2);
    let sense_of = |vol| uses.iter().find(|u| u.volume == vol).map(|u| c.face_use(*u).unwrap().sense).unwrap();
    assert_eq!(sense_of(a.volume), Sense::Forward);
    assert_eq!(sense_of(b.volume), Sense::Reversed);

    // imprint/merge status
    let rec = c.coincidence(b.faces[0], a.faces[1]).unwrap();
    assert_eq!((rec.a, rec.b, rec.status, rec.overlap_area), (a.faces[1], b.faces[0], ImprintStatus::Merged, Some(1.0)));

    // lineage: merged records, and successors resolve the absorbed ids
    assert_eq!(c.lineage().successors(b.faces[0].id()), vec![a.faces[1].id()]);
    assert_eq!(c.lineage().successors(b.vertices[0].id()), vec![a.vertices[1].id()]);
    assert!(c.lineage().is_retired(b.edges[4].id()));
    let merged_ops: Vec<_> = c.lineage().records().iter().filter(|r| matches!(r.kind, LineageKind::Merged { .. })).map(|r| r.op).collect();
    assert!(merged_ops.iter().all(|o| *o == OpId(3)) && merged_ops.len() == 4 + 4 + 1);

    // the interface edges are non-manifold: shared face + A's neighbour + B's neighbour
    assert_eq!(c.edge_coedges(a.edges[5]).len(), 3);
    assert_eq!(c.edge_faces(a.edges[5]).len(), 3);
    // each brick's other faces still face the exterior
    assert_eq!(c.face_role(a.faces[0]), Some(FaceRole::Boundary));
    assert_eq!(c.face_role(b.faces[1]), Some(FaceRole::Boundary));
    // merging again is impossible (face gone)
    c.begin_op(OpId(4)).unwrap();
    assert!(matches!(c.merge_faces(a.faces[1], b.faces[0]), Err(TopoError::NotFound(_))));
    c.end_op().unwrap();
}

#[test]
fn face_split_records_lineage_and_keeps_shells_closed() {
    let (mut c, mut g, b) = one_brick();
    let top = b.faces[5];
    c.begin_op(OpId(2)).unwrap();
    let (xa, xb) = (b.edges[2], b.edges[3]); // x-edges of the top face at y=0 and y=1
    let m1 = c.make_vertex(p(0.5, 0.0, 1.0), 1e-6).unwrap();
    let m2 = c.make_vertex(p(0.5, 1.0, 1.0), 1e-6).unwrap();
    let [a1, a2] = c.split_edge(xa, m1, 0.5).unwrap();
    let [b1, b2] = c.split_edge(xb, m2, 0.5).unwrap();
    let curve = g.add_curve(CurveDef::Line(Line::new(p(0.5, 0.0, 1.0), v(0.0, 1.0, 0.0)).unwrap())).unwrap();
    let chord = c.make_edge(EdgeSpec { curve: Some(curve), range: Interval::new(0.0, 1.0), start: m1, end: m2, tol: 1e-6, degenerate: false }).unwrap();
    let ring: Vec<Coedge> = c.face(top).unwrap().loops[0].coedges.clone();
    let pos = |vx: VertexId| ring.iter().position(|co| c.coedge_vertices(co).unwrap().0 == vx).unwrap();
    let [f0, f1] = c.split_face(SplitFaceSpec { face: top, chord, a: pos(m1), b: pos(m2), pcurves: [None, None] }).unwrap();
    c.end_op().unwrap();

    assert!(!c.is_alive(top.id()) && !c.is_alive(xa.id()));
    assert_eq!(c.lineage().successors(top.id()), vec![f0.id(), f1.id()]);
    assert_eq!(c.lineage().successors(xa.id()), vec![a1.id(), a2.id()]);
    assert_eq!(c.lineage().successors(xb.id()), vec![b1.id(), b2.id()]);
    assert_eq!(c.lineage().parents(f0.id()), vec![top.id()]);
    // new ids continue the serial sequence (never reused)
    assert_eq!((f0.serial(), f1.serial()), (6, 7));
    // 6 faces - 1 + 2; V 8+2; E 12 - 2 + 4 + 1
    assert_eq!((count(&c, EntityKind::TopoVertex), count(&c, EntityKind::TopoEdge), count(&c, EntityKind::TopoFace)), (10, 15, 7));
    assert_eq!(euler(&c), 2);
    assert_eq!(validate(&c), vec![]);
    for f in [f0, f1] {
        assert_eq!(c.face(f).unwrap().loops[0].coedges.len(), 4);
        assert_eq!(c.face_role(f), Some(FaceRole::Boundary));
    }
    // the neighbouring -y face now carries both halves of the split edge
    assert_eq!(c.face(b.faces[2]).unwrap().loops[0].coedges.len(), 5);
    // the chord is used once per child, with opposite senses
    let senses: Vec<_> = c.edge_coedges(chord).iter().map(|r| c.coedge(*r).unwrap().sense).collect();
    assert_eq!(senses.len(), 2);
    assert_ne!(senses[0], senses[1]);
    assert_eq!(c.volume(b.volume).unwrap().shells[0].uses.len(), 7);
    // bad split arguments leave the complex untouched
    let h = topology_hash(&c);
    c.begin_op(OpId(3)).unwrap();
    assert!(c.split_edge(a1, m1, 0.25).is_err());
    assert!(c.split_face(SplitFaceSpec { face: f0, chord, a: 0, b: 0, pcurves: [None, None] }).is_err());
    c.end_op().unwrap();
    assert_eq!(topology_hash(&c), h);
}

#[test]
fn cylinder_has_a_periodic_seam_used_twice() {
    let mut c = Complex::default();
    let mut g = SimpleGeomStore::new();
    c.begin_op(OpId(1)).unwrap();
    let CylinderIds { vertices: [vb, vt], edges: [eb, et, es], faces: [lat, bot, top], volume, .. } = make_cylinder(&mut c, &mut g, 2.0, 3.0).unwrap();
    c.end_op().unwrap();
    assert_eq!((count(&c, EntityKind::TopoVertex), count(&c, EntityKind::TopoEdge), count(&c, EntityKind::TopoFace)), (2, 3, 3));
    assert_eq!(euler(&c), 2);
    assert_eq!(validate(&c), vec![]);
    assert!(check_edge_geometry(&c, &g).is_empty());

    // seam: two coedges, both in the lateral face, opposite senses
    let seam = c.edge_coedges(es);
    assert_eq!(seam.len(), 2);
    assert!(seam.iter().all(|r| r.owner == CoedgeOwner::Face(lat)));
    let (s0, s1) = (c.coedge(seam[0]).unwrap(), c.coedge(seam[1]).unwrap());
    assert_ne!(s0.sense, s1.sense);
    // closed circular edges: one vertex, used once by the lateral face and once by a cap
    for (e, vx) in [(eb, vb), (et, vt)] {
        let ed = c.edge(e).unwrap();
        assert_eq!((ed.start, ed.end), (vx, vx));
        assert_eq!(c.edge_faces(e).len(), 2);
    }
    assert_eq!(c.vertex_edges(vb), vec![eb, es]);
    // pcurves sit on opposite sides of the seam and are same-parameter with the 3D edges
    let surf = g.surface(c.face(lat).unwrap().surface.unwrap()).unwrap().as_parametric().unwrap();
    assert_eq!(surf.period_u(), Some(TAU));
    let us: Vec<f64> = seam.iter().map(|r| c.coedge(*r).unwrap().pcurve.unwrap().eval(0.0)[0]).collect();
    assert!(us.contains(&0.0) && us.iter().any(|u| (u - TAU).abs() < 1e-15));
    for co in &c.face(lat).unwrap().loops[0].coedges {
        let e = c.edge(co.edge).unwrap();
        let curve = g.curve(e.curve.unwrap()).unwrap();
        for k in 1..4 {
            let t = e.range.lo + (e.range.hi - e.range.lo) * f64::from(k) / 4.0;
            let [u, vv] = co.pcurve.unwrap().eval(t);
            assert!((surf.eval(u, vv) - curve.eval(t)).norm() < 1e-12, "pcurve disagrees with edge curve");
        }
    }
    // caps and sides
    assert_eq!(c.face_role(bot), Some(FaceRole::Boundary));
    assert_eq!(c.face_role(top), Some(FaceRole::Boundary));
    assert_eq!(c.face_sides(lat).unwrap().back, Side::Volume(volume));
}

#[test]
fn baffle_is_a_face_with_the_same_volume_on_both_sides() {
    let (mut c, mut g, b) = one_brick();
    c.begin_op(OpId(2)).unwrap();
    // an internal square at x = 0.5
    let pts = [p(0.5, 0.25, 0.25), p(0.5, 0.75, 0.25), p(0.5, 0.75, 0.75), p(0.5, 0.25, 0.75)];
    let vs: Vec<VertexId> = pts.iter().map(|q| c.make_vertex(*q, 1e-6).unwrap()).collect();
    let mut coedges = Vec::new();
    for i in 0..4 {
        let (a, bb) = (pts[i], pts[(i + 1) % 4]);
        let curve = g.add_curve(CurveDef::Line(Line::new(a, bb - a).unwrap())).unwrap();
        let e = c.make_edge(EdgeSpec { curve: Some(curve), range: Interval::new(0.0, (bb - a).norm()), start: vs[i], end: vs[(i + 1) % 4], tol: 1e-6, degenerate: false }).unwrap();
        coedges.push(Coedge { edge: e, sense: Sense::Forward, pcurve: None });
    }
    let surface = g.add_surface(sm_geom_api::SurfaceDef::Plane(sm_geom_api::Plane { frame: sm_geom_api::Frame::from_zx(pts[0], v(1.0, 0.0, 0.0), pts[1] - pts[0]).unwrap() })).unwrap();
    let baffle = c.make_face(FaceSpec { surface: Some(surface), surface_sense: Sense::Forward, tol: 1e-6, loops: vec![LoopData { kind: LoopKind::Outer, coedges }], embedded_edges: vec![] }).unwrap();
    assert_eq!(c.face_role(baffle), Some(FaceRole::Free));
    c.add_baffle(b.volume, baffle).unwrap();
    assert!(c.add_baffle(b.volume, baffle).is_err(), "a baffle has exactly its two uses");
    c.end_op().unwrap();

    let sides = c.face_sides(baffle).unwrap();
    assert_eq!(sides, FaceSides { front: Side::Volume(b.volume), back: Side::Volume(b.volume) });
    assert_eq!(c.face_role(baffle), Some(FaceRole::Baffle));
    let senses: Vec<Sense> = c.face_uses(baffle).iter().map(|u| c.face_use(*u).unwrap().sense).collect();
    assert_eq!(senses, vec![Sense::Forward, Sense::Reversed]);
    assert_eq!(validate(&c), vec![], "baffle uses cancel in the shell balance");
    assert_eq!(c.volume_faces(b.volume).len(), 7);
    // outer boundary unchanged
    assert_eq!(c.face_role(b.faces[0]), Some(FaceRole::Boundary));
}

fn scenario() -> Complex {
    let (mut c, _g, a, b) = {
        let (c, g, a, b) = two_bricks();
        (c, g, a, b)
    };
    merge_shared_face(&mut c, &a, &b);
    c.begin_op(OpId(5)).unwrap();
    c.raise_tolerance(a.vertices[0].id(), 5e-6).unwrap();
    c.end_op().unwrap();
    c
}

#[test]
fn determinism_same_build_same_ids_and_hash() {
    let (c1, c2) = (scenario(), scenario());
    for k in [EntityKind::TopoVertex, EntityKind::TopoEdge, EntityKind::TopoFace, EntityKind::TopoVolume, EntityKind::TopoBody] {
        assert_eq!(c1.entity_ids(k), c2.entity_ids(k));
    }
    assert_eq!(c1.lineage().records(), c2.lineage().records());
    assert_eq!(topology_hash(&c1), topology_hash(&c2));
    assert_eq!(c1.coincidences(), c2.coincidences());
    // sensitivity: a different build hashes differently
    let (c3, _, _) = one_brick();
    assert_ne!(topology_hash(&c1), topology_hash(&c3));
    // single-thread vs default threads: nothing here is threaded, but the contract is checked
    // by building under a pool of 1 and N scoped threads and comparing.
    let hashes: Vec<u64> = std::thread::scope(|s| (0..4).map(|_| s.spawn(|| topology_hash(&scenario()))).collect::<Vec<_>>().into_iter().map(|h| h.join().unwrap()).collect());
    assert!(hashes.iter().all(|h| *h == topology_hash(&c1)));
}

#[test]
fn tolerance_is_per_entity_monotone_and_capped() {
    let mut c = Complex::default();
    assert_eq!(c.begin_op(OpId(1)), Ok(()));
    assert_eq!(c.begin_op(OpId(2)), Err(TopoError::OpAlreadyActive(OpId(1))));
    let a = c.make_vertex(p(0.0, 0.0, 0.0), 1e-12).unwrap();
    assert_eq!(c.vertex(a).unwrap().tol, c.policy().linear_min, "raised to the policy floor");
    assert!(matches!(c.make_vertex(p(0.0, 0.0, 0.0), 1.0), Err(TopoError::Tolerance(_))));
    let b = c.make_vertex(p(1e-5, 0.0, 0.0), 1e-6).unwrap();
    let far = c.make_vertex(p(1e-2, 0.0, 0.0), 1e-6).unwrap();
    assert!(matches!(c.merge_vertices(a, far), Err(TopoError::Tolerance(_))), "gap beyond the cap");
    let s = c.merge_vertices(b, a).unwrap();
    assert_eq!(s, a);
    // survivor tolerance covers the other vertex's ball: max(tol_a, dist + tol_b)
    assert!((c.vertex(a).unwrap().tol - (1e-5 + 1e-6)).abs() < 1e-15);
    assert!(c.raise_tolerance(a.id(), 1e-9).is_err(), "never shrinks");
    assert!(c.raise_tolerance(a.id(), 1.0).is_err(), "capped");
    c.raise_tolerance(a.id(), 2e-5).unwrap();
    c.end_op().unwrap();
    assert!(matches!(c.raise_tolerance(a.id(), 3e-5), Err(TopoError::NoActiveOp)));
    assert!(c.policy().coincident(1.5e-5, c.vertex(a).unwrap().tol, 0.0 + 1e-7));
    assert!(c.lineage().records().iter().any(|r| matches!(r.kind, LineageKind::Modified { .. })));
}

#[test]
fn validate_reports_open_shells_and_open_loops() {
    let (mut c, _g, b) = one_brick();
    c.begin_op(OpId(2)).unwrap();
    c.delete_face(b.faces[5]).unwrap();
    assert!(matches!(c.delete_edge(b.edges[0]), Err(TopoError::InUse(_))));
    c.end_op().unwrap();
    let issues = validate(&c);
    assert!(issues.iter().any(|i| matches!(i, TopoIssue::ShellOpen { .. })));
    // a non-closed loop is rejected at construction
    let (mut c, _g, b) = one_brick();
    c.begin_op(OpId(2)).unwrap();
    let bad = vec![Coedge { edge: b.edges[0], sense: Sense::Forward, pcurve: None }, Coedge { edge: b.edges[3], sense: Sense::Forward, pcurve: None }];
    let r = c.make_face(FaceSpec { surface: None, surface_sense: Sense::Forward, tol: 1e-6, loops: vec![LoopData { kind: LoopKind::Outer, coedges: bad }], embedded_edges: vec![] });
    assert!(matches!(r, Err(TopoError::LoopNotClosed { .. })));
}

#[test]
fn location_instancing_and_mirrors_reverse_orientation() {
    let (mut c, _g, b) = one_brick();
    let mirror = Location(Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: v(0.0, 0.0, 0.0) });
    let shift = Location(Transform { m: Transform::IDENTITY.m, t: v(5.0, 0.0, 0.0) });
    assert!(mirror.flips_orientation() && !shift.flips_orientation());
    // composition and inverse
    let both = shift.compose(&mirror);
    assert_eq!(both.0.apply_point(p(1.0, 2.0, 3.0)), p(4.0, 2.0, 3.0));
    let back = both.inverse().unwrap();
    assert!((back.compose(&both).0.apply_point(p(1.0, 2.0, 3.0)) - p(1.0, 2.0, 3.0)).norm() < 1e-15);
    assert_eq!(Sense::Reversed.compose(Sense::Reversed), Sense::Forward);

    c.begin_op(OpId(2)).unwrap();
    let lattice = c.make_instance(ShapeRef { id: b.body.id(), sense: Sense::Forward, location: shift }, None).unwrap();
    let cell = c.make_instance(ShapeRef { id: b.body.id(), sense: Sense::Forward, location: mirror }, Some(lattice)).unwrap();
    assert!(c.make_instance(ShapeRef { id: b.faces[0].id(), sense: Sense::Forward, location: shift }, None).is_err(), "only bodies/volumes are instanced");
    c.end_op().unwrap();
    // a mirrored instance sees every outward face use as inward, without touching the definition
    let inst = c.instance(cell).unwrap();
    assert_eq!(inst.target.effective_sense(), Sense::Reversed);
    let fu = c.volume(b.volume).unwrap().shells[0].uses[0];
    assert_eq!(fu.sense, Sense::Forward);
    assert_eq!(c.instance_location(cell).unwrap().0.apply_point(p(1.0, 0.0, 0.0)), p(4.0, 0.0, 0.0));
    assert_eq!(mirror.orient(fu.sense), Sense::Reversed);
    assert_eq!(c.count(EntityKind::Instance), 2);
    assert_eq!(c.count(EntityKind::TopoBody), 1, "one shared definition, two uses");
}

#[test]
fn virtual_overlay_leaves_the_host_untouched() {
    let (mut c, _g, a, b) = two_bricks();
    merge_shared_face(&mut c, &a, &b);
    let before = topology_hash(&c);
    let mut ov = OverlayTable::new();
    // present the two coplanar top faces as one virtual face
    let comp = ov.composite_faces(&c, &[a.faces[5], b.faces[5]]).unwrap();
    assert_eq!(comp.kind(), EntityKind::PartitionCell);
    assert!(matches!(ov.composite_faces(&c, &[a.faces[5]]), Err(OverlayError::AlreadyComposite(_))));
    let parts = ov.partition_volume(&c, a.volume, &[a.faces[1].id()], 2).unwrap();
    assert_eq!(parts.len(), 2);

    let view = WithOverlay { host: &c, overlay: &ov };
    assert_eq!(topology_hash(&view), before, "host content is identical with the overlay attached");
    assert!(c.virtual_overlay().is_none());
    let o = view.virtual_overlay().unwrap();
    assert_eq!(o.pieces().len(), 3);
    assert_eq!(o.covering(a.faces[5].id())[0].id, comp);
    assert_eq!(o.covering(a.volume.id()).len(), 2);
    assert_eq!(o.piece(comp).unwrap().kind, VirtualKind::CompositeFace);
    assert_eq!(validate(&view), validate(&c));
    // host faces are still individually queryable
    assert_eq!(view.face_role(a.faces[5]), Some(FaceRole::Boundary));
}

#[test]
fn views_and_editors_are_object_safe() {
    fn faces(v: &dyn TopologyView) -> usize {
        v.entity_ids(EntityKind::TopoFace).len()
    }
    fn add_vertex(e: &mut dyn TopologyEdit) -> EntityId {
        e.make_vertex(p(0.0, 0.0, 0.0), 1e-6).unwrap().id()
    }
    let (mut c, _g, _b) = one_brick();
    assert_eq!(faces(&c), 6);
    c.begin_op(OpId(2)).unwrap();
    let id = add_vertex(&mut c);
    c.end_op().unwrap();
    assert!(c.is_alive(id));
}
