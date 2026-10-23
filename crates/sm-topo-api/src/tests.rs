//! Tests: invariants on the reference fixtures, lineage, overlay, errors, determinism.

use crate::reference::fixtures::*;
use crate::*;
use core::f64::consts::TAU;
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::{ModelTolerance, Sense, ShapeStore};

fn assert_valid(c: &dyn TopologyView) {
    let v = validate(c);
    assert!(v.is_empty(), "violations: {v:#?}");
}

fn single_brick() -> (RefComplex, Brick) {
    let mut c = new_model();
    let b = brick(&mut c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)).unwrap();
    (c, b)
}

#[test]
fn brick_counts_sides_and_rings() {
    let (c, b) = single_brick();
    assert_valid(&c);
    assert_eq!((c.vertices().len(), c.edges().len(), c.faces().len(), c.volumes().len(), c.bodies().len()), (8, 12, 6, 1, 1));
    assert_eq!(euler_characteristic(&c), 2);
    for &f in c.faces() {
        // every face: one side the volume, the other the exterior; the volume sees it with an aligned normal
        assert_eq!(c.face_sides(f), Some([Side::Volume(b.volume), Side::Exterior]));
        assert_eq!(c.face_class(f), Some(FaceClass::Boundary(b.volume)));
        let fu = c.face_use(FaceUse { face: f, side: FaceSide::Negative }).unwrap();
        assert_eq!((fu.sense, fu.owner), (UseSense::Aligned, UseOwner::Shell(b.shell)));
        assert_eq!(c.face_use(FaceUse { face: f, side: FaceSide::Positive }).unwrap().sense, UseSense::Opposed);
    }
    for &e in c.edges() {
        assert_eq!(c.edge_coedges(e).len(), 2);
        let ring = c.radial_ring(e);
        assert_eq!(ring.len(), 4, "two faces, two sides each");
        // exactly two wedges (one volume, one exterior) between four uses
        let sides: Vec<Side> = ring.iter().map(|&u| c.edge_use_side(u).unwrap()).collect();
        assert_eq!(sides.iter().filter(|s| matches!(s, Side::Volume(_))).count(), 2);
        // uses of one face are adjacent in the ring
        let n = ring.len();
        let adjacent_pairs = (0..n).filter(|&i| matches!((ring[i], ring[(i + 1) % n]), (EdgeUse::Coedge { coedge: a, .. }, EdgeUse::Coedge { coedge: b, .. }) if a.face == b.face)).count();
        assert_eq!(adjacent_pairs, 2);
    }
    for &v in c.vertices() {
        assert_eq!(c.vertex_uses(v).len(), 6, "3 faces x 2 sides");
    }
    assert_eq!(c.faces_of_volume(b.volume).len(), 6);
    assert!(c.neighbors_of_volume(b.volume).is_empty());
    assert!(c.lineage().records().iter().all(|r| matches!(r.event, LineageEvent::Created { .. })));
}

#[test]
fn ring_order_is_counter_clockwise_with_correct_wedges() {
    // Edge along z at the origin between the -x face (inward +y) and the -y face (inward +x).
    let (c, b) = single_brick();
    let e = b.edge(0, 4).unwrap();
    let ring = c.radial_ring(e);
    let owners: Vec<(FaceId, FaceSide, Side)> = ring
        .iter()
        .map(|&u| match u {
            EdgeUse::Coedge { coedge, side } => (coedge.face, side, c.edge_use_side(u).unwrap()),
            EdgeUse::Wire { .. } => unreachable!(),
        })
        .collect();
    let (mx, my) = (b.faces[0], b.faces[2]);
    let pos = owners.iter().position(|o| o.0 == my).unwrap();
    // CCW about +z starts at the -y face (angle 0 reference is the canonically first coedge, face -x):
    // find the cyclic sequence and check the interior wedge sits between the two interior uses.
    let interior: Vec<usize> = owners.iter().enumerate().filter(|(_, o)| matches!(o.2, Side::Volume(_))).map(|(i, _)| i).collect();
    assert_eq!(interior.len(), 2);
    assert!([1, 3].contains(&((interior[1] + 4 - interior[0]) % 4)), "interior uses are cyclic neighbours");
    assert!(owners.iter().any(|o| o.0 == mx) && pos < 4);
    // Going CCW about +z (x then y), the -y face (inward +x, angle 0) comes before the -x face (inward +y, angle 90):
    // so after the -y face's interior use comes the -x face's interior use.
    let iy = owners.iter().position(|o| o.0 == my && o.1 == FaceSide::Negative).unwrap();
    let ix = owners.iter().position(|o| o.0 == mx && o.1 == FaceSide::Negative).unwrap();
    assert_eq!((iy + 1) % 4, ix);
}

#[test]
fn two_bricks_share_one_face_with_opposite_use_senses() {
    let mut c = new_model();
    let t = two_bricks_merged(&mut c).unwrap();
    assert_valid(&c);
    assert_eq!((c.vertices().len(), c.edges().len(), c.faces().len()), (12, 20, 11));
    assert_eq!(euler_characteristic(&c), 3);
    let s = t.shared;
    assert_eq!(c.face_sides(s), Some([Side::Volume(t.a.volume), Side::Volume(t.b.volume)]));
    assert_eq!(c.face_class(s), Some(FaceClass::Interface(t.a.volume, t.b.volume)));
    assert_eq!(c.face_use(FaceUse { face: s, side: FaceSide::Negative }).unwrap().sense, UseSense::Aligned);
    assert_eq!(c.face_use(FaceUse { face: s, side: FaceSide::Positive }).unwrap().sense, UseSense::Opposed);
    assert_eq!(c.neighbors_of_volume(t.a.volume), vec![t.b.volume]);
    assert_eq!(c.neighbors_of_volume(t.b.volume), vec![t.a.volume]);
    // B's old face is retired, merged into the survivor, and the pair is recorded as Merged
    assert_eq!(c.face(t.b.faces[0]).unwrap().state, EntityState::Merged(OpId(3)));
    assert_eq!(c.coincidence_of(t.b.faces[0], s).unwrap().status, Coincidence::Merged);
    assert_eq!(c.lineage().current_of(t.b.faces[0].entity()), vec![s.entity()]);
    // the shared face's edges are T-junctions of three faces: shared + A's wall + B's wall
    for co in &c.face(s).unwrap().loops[0].coedges {
        let e = co.kind.edge().unwrap();
        assert_eq!(c.radial_ring(e).len(), 6);
    }
    // each volume still has six faces, the shared one counted in both
    assert_eq!(c.faces_of_volume(t.a.volume).len(), 6);
    assert_eq!(c.faces_of_volume(t.b.volume).len(), 6);
    // merged vertices keep the survivor id; the dropped one is retired
    assert_eq!(c.vertex(t.b.vertices[0]).unwrap().state, EntityState::Merged(OpId(3)));
}

#[test]
fn imprint_then_merge_progression_and_non_manifold_edge() {
    let mut c = new_model();
    let ib = two_bricks_imprint(&mut c).unwrap();
    assert_valid(&c);
    let (lower, upper) = ib.a_halves;
    let b_mx = ib.b.faces[0];
    let a_px = ib.a.faces[1];
    // after the imprint: the original is split, the halves exist, the lower half and B's face are
    // identical in boundary but still two faces
    assert_eq!(c.face(a_px).unwrap().state, EntityState::Split(OpId(3)));
    assert_eq!(c.coincidence_of(a_px, b_mx).unwrap().status, Coincidence::Overlapping);
    assert_eq!(c.coincidence_of(lower, b_mx).unwrap().status, Coincidence::ImprintedNotMerged);
    assert_eq!(c.face_class(lower), Some(FaceClass::Boundary(ib.a.volume)));
    assert_eq!(c.faces_of_volume(ib.a.volume).len(), 7);
    // the cut edge is used by the two halves only, so far
    assert_eq!(c.radial_ring(ib.cut_edge).len(), 4);

    merge_imprinted(&mut c, &ib).unwrap();
    assert_valid(&c);
    assert_eq!(c.coincidence_of(lower, b_mx).unwrap().status, Coincidence::Merged);
    assert_eq!(c.face_class(lower), Some(FaceClass::Interface(ib.a.volume, ib.b.volume)));
    assert_eq!(c.face_class(upper), Some(FaceClass::Boundary(ib.a.volume)));
    assert_eq!(c.neighbors_of_volume(ib.a.volume), vec![ib.b.volume]);
    // three faces meet at the cut edge: lower (shared), upper, and B's +y wall: a non-manifold radial ring
    let ring = c.radial_ring(ib.cut_edge);
    assert_eq!(ring.len(), 6);
    let sides: Vec<Side> = ring.iter().map(|&u| c.edge_use_side(u).unwrap()).collect();
    // wedges: volume A, volume B, exterior, each owning two ring positions (a wedge's two bounding uses)
    for want in [Side::Volume(ib.a.volume), Side::Volume(ib.b.volume), Side::Exterior] {
        assert_eq!(sides.iter().filter(|&&s| s == want).count(), 2, "{want:?} in {sides:?}");
    }
    // lineage: A's +x face became the two halves
    assert_eq!(c.lineage().current_of(a_px.entity()), {
        let mut v = vec![lower.entity(), upper.entity()];
        v.sort();
        v
    });
    // euler of the whole complex: V - E + F - C = 1 for two touching 3-cells
    let vs = c.vertices().len() as i64;
    let es = c.edges().len() as i64;
    let fs = c.faces().len() as i64;
    assert_eq!(vs - es + fs - c.volumes().len() as i64, 1);
}

#[test]
fn face_split_records_lineage_and_keeps_shell_valid() {
    let mut c = new_model();
    let s = split_top_face(&mut c).unwrap();
    assert_valid(&c);
    assert_eq!((c.vertices().len(), c.edges().len(), c.faces().len()), (10, 15, 7));
    assert_eq!(euler_characteristic(&c), 2);
    let [p, q] = [s.pieces.first, s.pieces.second];
    assert_eq!(c.face(s.original).unwrap().state, EntityState::Split(OpId(2)));
    assert_eq!(c.lineage().successors(s.original.entity()), vec![p.entity(), q.entity()]);
    assert_eq!(c.lineage().ancestors_of(p.entity()), vec![s.original.entity()]);
    assert_eq!(c.lineage().ancestors_of(q.entity()), vec![s.original.entity()]);
    let hist: Vec<_> = c.lineage().history_of(s.original.entity()).iter().map(|r| r.op).collect();
    assert_eq!(hist, vec![OpId(1), OpId(2)]);
    // the split edges are retired with children, and walls now have 5 coedges
    for es in &s.edge_splits {
        let parent = c.lineage().ancestors_of(es.first.entity());
        assert_eq!(parent.len(), 1);
        assert_eq!(c.edge(EdgeId::from_entity(parent[0]).unwrap()).unwrap().state, EntityState::Split(OpId(2)));
    }
    let wall = s.brick.faces[2];
    assert_eq!(c.face(wall).unwrap().loops[0].coedges.len(), 5);
    // the volume's shell lists both pieces and not the original
    let shell = c.shell(s.brick.shell).unwrap();
    assert!(shell.face_uses.contains(&FaceUse { face: p, side: FaceSide::Negative }));
    assert!(shell.face_uses.contains(&FaceUse { face: q, side: FaceSide::Negative }));
    assert!(!shell.face_uses.iter().any(|u| u.face == s.original));
    assert_eq!(c.radial_ring(s.cut).len(), 4);
    // ids are never reused: the originals keep their records
    assert_eq!(s.original.serial(), 5);
    assert!(p.serial() >= 6 && q.serial() > p.serial());
}

#[test]
fn cylinder_seam_is_one_edge_used_twice() {
    let mut c = new_model();
    c.begin_op(OpId(1)).unwrap();
    let cyl = cylinder_in_op(&mut c, 2.0, 3.0).unwrap();
    c.end_op().unwrap();
    assert_valid(&c);
    assert_eq!((c.vertices().len(), c.edges().len(), c.faces().len()), (2, 3, 3));
    assert_eq!(euler_characteristic(&c), 2);
    let [lat, top, bot] = cyl.faces;
    let seam = cyl.edges[2];
    let cos = c.edge_coedges(seam);
    assert_eq!(cos.len(), 2);
    assert!(cos.iter().all(|r| r.face == lat));
    let senses: Vec<Sense> = cos.iter().map(|&r| match c.coedge(r).unwrap().kind {
        CoedgeKind::Edge { sense, .. } => sense,
        CoedgeKind::Pole { .. } => unreachable!(),
    }).collect();
    assert!(senses.contains(&Sense::Forward) && senses.contains(&Sense::Reversed));
    // seam pcurves sit on the two opposite domain boundaries u = 2*pi and u = 0
    let lat_rec = c.face(lat).unwrap();
    let us: Vec<f64> = cos.iter().map(|r| match &lat_rec.loops[0].coedges[r.index as usize].pcurve {
        Some(sm_geom_api::PCurve::Segment { start, .. }) => start[0],
        _ => f64::NAN,
    }).collect();
    assert!(us.contains(&0.0) && us.iter().any(|&u| (u - TAU).abs() < 1e-15));
    // the lateral surface is periodic in u with a one-period domain
    let s = c.shapes().surface(lat_rec.surface).unwrap();
    assert_eq!(s.domain().u_period, Some(TAU));
    // the seam ring has four uses (two coedges x two sides) and both wedges are consistent
    assert_eq!(c.radial_ring(seam).len(), 4);
    // closed circular edges: start == end
    for &e in &cyl.edges[..2] {
        let r = c.edge(e).unwrap();
        assert_eq!(r.start, r.end);
        assert!((r.t1 - r.t0 - TAU).abs() < 1e-15);
    }
    assert_eq!(c.radial_ring(cyl.edges[0]).len(), 4);
    // classes
    assert!([lat, top, bot].iter().all(|&f| c.face_class(f) == Some(FaceClass::Boundary(cyl.volume))));
}

#[test]
fn sphere_poles_are_degenerate_coedges_not_edges() {
    let mut c = new_model();
    c.begin_op(OpId(1)).unwrap();
    let s = sphere_in_op(&mut c, 2.0).unwrap();
    c.end_op().unwrap();
    assert_valid(&c);
    assert_eq!((c.vertices().len(), c.edges().len(), c.faces().len()), (2, 1, 1));
    assert_eq!(euler_characteristic(&c), 2);
    assert_eq!(c.radial_ring(s.seam).len(), 4);
    let corners = c.vertex_uses(s.poles[0]).iter().filter(|u| matches!(u.site, VertexSite::Corner { .. })).count();
    assert!(corners >= 4, "pole corner appears for the pole coedge and the seam, from both sides");
    let sh = c.shapes().surface(c.face(s.face).unwrap().surface).unwrap();
    assert_eq!(sh.poles().len(), 2);
}

#[test]
fn baffle_and_wire_inside_a_volume() {
    let mut c = new_model();
    c.begin_op(OpId(1)).unwrap();
    let bb = baffle_block_in_op(&mut c).unwrap();
    c.end_op().unwrap();
    assert_valid(&c);
    let v = bb.brick.volume;
    assert_eq!(c.face_class(bb.plate), Some(FaceClass::Baffle(v)));
    assert_eq!(c.face_sides(bb.plate), Some([Side::Volume(v), Side::Volume(v)]));
    assert_eq!(c.volume(v).unwrap().shells.len(), 2);
    assert_eq!(c.faces_of_volume(v).len(), 7);
    // free-standing sheet: each edge ring is the two mated uses of the plate, both in the volume
    for &e in &bb.plate_edges {
        let ring = c.radial_ring(e);
        assert_eq!(ring.len(), 2);
        assert!(ring.iter().all(|&u| c.edge_use_side(u) == Some(Side::Volume(v))));
    }
    // the wire is a one-entry ring inside the volume
    assert_eq!(c.radial_ring(bb.wire), &[EdgeUse::Wire { edge: bb.wire, volume: Some(v) }]);
    assert!(c.vertex_uses(c.edge(bb.wire).unwrap().start).iter().any(|u| matches!(u.site, VertexSite::WireEnd { at_end: false, .. })));
    // no volume neighbours: a baffle does not separate volumes
    assert!(c.neighbors_of_volume(v).is_empty());
}

#[test]
fn loop_use_traverses_reversed_from_the_positive_side() {
    let (c, b) = single_brick();
    let f = c.face(b.faces[5]).unwrap();
    let neg = loop_use_edge_uses(b.faces[5], f, FaceSide::Negative, 0);
    let pos = loop_use_edge_uses(b.faces[5], f, FaceSide::Positive, 0);
    assert_eq!(neg.len(), 4);
    let mut rev = neg.clone();
    rev.reverse();
    let strip = |v: &[EdgeUse]| -> Vec<CoedgeRef> { v.iter().map(|u| if let EdgeUse::Coedge { coedge, .. } = u { *coedge } else { unreachable!() }).collect() };
    assert_eq!(strip(&pos), strip(&rev));
    // the tail of consecutive edge uses chain head to tail on both sides
    for uses in [&neg, &pos] {
        let ends: Vec<(VertexId, VertexId)> = uses.iter().map(|&u| {
            let tail = edge_use_tail(&c, u).unwrap();
            let next = edge_use_tail(&c, uses[(uses.iter().position(|x| *x == u).unwrap() + 1) % 4]).unwrap();
            (tail, next)
        }).collect();
        assert!(ends.iter().all(|(a, b)| a != b));
    }
}

#[test]
fn determinism_same_build_same_ids_and_hash() {
    let build = || {
        let mut c = new_model();
        let ib = two_bricks_imprint_merge(&mut c).unwrap();
        (state_hash(&c), c.faces().to_vec(), ib.cut_edge, c.radial_ring(ib.cut_edge).to_vec())
    };
    let a = build();
    let b = build();
    assert_eq!(a, b);
    // a different history hashes differently
    let mut c = new_model();
    two_bricks_merged(&mut c).unwrap();
    assert_ne!(state_hash(&c), a.0);
    // building on another thread (fixed 1 thread vs default) gives the same hash
    let t = std::thread::spawn(build).join().unwrap();
    assert_eq!(t, a);
    // ids carry kind and dense serials in creation order
    assert!(a.1.iter().all(|f| f.entity().kind() == EntityKind::TopoFace));
    assert_eq!(c.vertices().first().map(|v| v.serial()), Some(0));
}

#[test]
fn hash_is_sensitive_to_geometry_and_stable_under_clone() {
    let (c, _) = single_brick();
    let h = state_hash(&c);
    assert_eq!(state_hash(&c.clone()), h);
    let mut c2 = new_model();
    brick(&mut c2, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 2.0)).unwrap();
    assert_ne!(state_hash(&c2), h);
    // moving a vertex by less than the hash quantum does not change the hash
    let tiny = ModelTolerance::default().hash_quantum() * 0.1;
    let mut c3 = new_model();
    brick(&mut c3, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0 + tiny, 1.0, 1.0)).unwrap();
    assert_eq!(state_hash(&c3), h);
}

#[test]
fn overlay_composites_without_touching_the_host() {
    let mut c = new_model();
    let s = split_top_face(&mut c).unwrap();
    let before = state_hash(&c);
    let mut ov = SimpleOverlay::new("cleanup");
    let hosts = [s.pieces.first.entity(), s.pieces.second.entity()];
    let vf = ov.composite(&c, &hosts).unwrap();
    assert_eq!(state_hash(&c), before, "host untouched");
    assert_valid(&c);
    assert!(FaceId::from_entity(vf).unwrap().is_virtual());
    let vis = visible_faces(&c, &ov);
    assert_eq!(vis.len(), 6, "5 untouched faces + 1 composite");
    assert!(vis.contains(&vf) && !vis.contains(&hosts[0]));
    assert!(ov.is_hidden(hosts[0]));
    assert_eq!(ov.virtual_of(hosts[1]), vec![vf]);
    assert_eq!(ov.entity(vf).unwrap().hosts.len(), 2);
    // the cut edge is interior to the composite; a composite of edges can hide it
    assert!(visible_edges(&c, &ov).contains(&s.cut.entity()), "edges are shown until composited");
    // a second composite of an already hidden face is refused; disconnected faces are refused
    assert!(matches!(ov.composite(&c, &hosts), Err(TopoError::InUse(_))));
    let mut ov2 = SimpleOverlay::new("bad");
    let opposite = [s.brick.faces[0].entity(), s.brick.faces[1].entity()];
    assert!(matches!(ov2.composite(&c, &opposite), Err(TopoError::NotCoincident(_))));
    // partition: one face seen as two virtual pieces
    let pieces = ov2.partition(&c, s.brick.faces[4].entity(), 2).unwrap();
    assert_eq!(pieces.len(), 2);
    assert_ne!(ov.content_hash(), ov2.content_hash());
    // dropping the overlay restores the host view exactly
    drop(ov);
    assert_eq!(visible_faces(&c, &SimpleOverlay::new("empty")).len(), c.faces().len());
    assert_eq!(state_hash(&c), before);
}

#[test]
fn edit_protocol_and_error_paths() {
    let mut c = new_model();
    assert_eq!(c.make_body().unwrap_err(), TopoError::NoOpenOp);
    c.begin_op(OpId(5)).unwrap();
    assert!(matches!(c.begin_op(OpId(6)), Err(TopoError::BadOp(_))));
    let b = c.make_body().unwrap();
    c.end_op().unwrap();
    assert!(matches!(c.begin_op(OpId(5)), Err(TopoError::BadOp(_))), "op ids must increase");
    c.begin_op(OpId(9)).unwrap();
    let v1 = c.make_vertex(Point3::new(0.0, 0.0, 0.0)).unwrap();
    let v2 = c.make_vertex(Point3::new(0.0, 0.0, 1e-3)).unwrap();
    assert!(matches!(c.merge_vertices(v1, v2), Err(TopoError::NotCoincident(_))));
    assert!(matches!(c.merge_vertices(v1, v1), Err(TopoError::Unsupported(_))));
    let v3 = c.make_vertex(Point3::new(0.0, 0.0, 1e-8)).unwrap();
    c.merge_vertices(v1, v3).unwrap();
    assert!(matches!(c.merge_vertices(v1, v3), Err(TopoError::NotActive(_))));
    assert!(matches!(c.kill_vertex(v1), Ok(())));
    assert!(matches!(c.make_edge(EdgeSpec { curve: EntityId::new(EntityKind::Curve, 99).unwrap(), t0: 0.0, t1: 1.0, start: v2, end: v2 }), Err(TopoError::UnknownShape(_))));
    // edges whose curve does not pass through the vertices are rejected
    let a = c.make_vertex(Point3::new(0.0, 0.0, 0.0)).unwrap();
    let z = c.make_vertex(Point3::new(0.0, 0.0, 2.0)).unwrap();
    assert!(line_edge(&mut c, a, z).is_ok());
    c.end_op().unwrap();
    let _ = b;
}

#[test]
fn bind_release_kill_and_inuse_checks() {
    let mut c = new_model();
    c.begin_op(OpId(1)).unwrap();
    let bb = baffle_block_in_op(&mut c).unwrap();
    let plate_neg = FaceUse { face: bb.plate, side: FaceSide::Negative };
    assert!(matches!(c.bind_face_use(plate_neg, bb.internal), Err(TopoError::SideOccupied(_))));
    assert!(matches!(c.kill_face(bb.plate), Err(TopoError::InUse(_))));
    assert!(matches!(c.kill_edge(bb.plate_edges[0]), Err(TopoError::InUse(_))));
    assert!(matches!(c.kill_volume(bb.brick.volume), Err(TopoError::InUse(_))));
    c.release_face_use(plate_neg).unwrap();
    assert!(matches!(c.release_face_use(plate_neg), Err(TopoError::NotBound(_))));
    c.release_face_use(plate_neg.mate()).unwrap();
    assert_eq!(c.face_class(bb.plate), Some(FaceClass::Sheet));
    // a sheet can be registered with a body, then deleted: lineage records the deletion
    c.add_sheet(bb.brick.body, bb.plate).unwrap();
    assert!(matches!(c.bind_face_use(plate_neg, bb.internal), Ok(())), "binding removes it from the sheets");
    c.release_face_use(plate_neg).unwrap();
    c.kill_face(bb.plate).unwrap();
    for e in bb.plate_edges {
        c.kill_edge(e).unwrap();
    }
    c.end_op().unwrap();
    assert_valid(&c);
    assert!(c.lineage().is_deleted(bb.plate.entity()));
    assert!(c.lineage().current_of(bb.plate.entity()).is_empty());
    assert_eq!(c.faces().len(), 6);
    assert_eq!(c.face(bb.plate).unwrap().state, EntityState::Deleted(OpId(1)));
}

#[test]
fn merge_and_split_preconditions() {
    let mut c = new_model();
    let a = brick(&mut c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let b = brick(&mut c, OpId(2), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0)).unwrap();
    c.begin_op(OpId(3)).unwrap();
    // faces cannot merge before their edges are shared
    assert!(matches!(c.merge_faces(a.faces[1], b.faces[0], Sense::Reversed), Err(TopoError::NotCoincident(_))));
    // vertices must be identical before edges merge
    assert!(matches!(c.merge_edges(a.edge(1, 3).unwrap(), b.edge(0, 2).unwrap()), Err(TopoError::NotCoincident(_))));
    // curves that do not coincide are refused
    assert!(matches!(c.merge_edges(a.edge(0, 1).unwrap(), b.edge(0, 2).unwrap()), Err(TopoError::NotCoincident(_))));
    // split parameter must be strictly inside
    assert!(matches!(c.split_edge(a.edge(0, 1).unwrap(), SplitAt::NewVertex(0.0)), Err(TopoError::InvalidLoop(_))));
    assert!(matches!(c.split_edge(a.edge(0, 1).unwrap(), SplitAt::NewVertex(5.0)), Err(TopoError::InvalidLoop(_))));
    // an existing vertex must lie on the curve
    assert!(matches!(c.split_edge(a.edge(0, 1).unwrap(), SplitAt::Vertex { t: 0.5, vertex: a.vertices[7] }), Err(TopoError::NotCoincident(_))));
    // a cut edge must join the two corners
    let bad = FaceCut { from: 0, to: 2, edge: a.edge(0, 1).unwrap(), inner_loops_to_first: vec![] };
    assert!(matches!(c.split_face(a.faces[5], bad), Err(TopoError::InvalidLoop(_))));
    let oob = FaceCut { from: 2, to: 9, edge: a.edge(0, 1).unwrap(), inner_loops_to_first: vec![] };
    assert!(matches!(c.split_face(a.faces[5], oob), Err(TopoError::InvalidLoop(_))));
    // a face with an unclosed loop cannot be created
    let surf = c.face(a.faces[5]).unwrap().surface;
    let open = FaceSpec {
        surface: surf,
        surface_sense: Sense::Forward,
        loops: vec![LoopRec { kind: LoopKind::Outer, coedges: vec![CoedgeRec { kind: CoedgeKind::Edge { edge: a.edge(0, 1).unwrap(), sense: Sense::Forward }, pcurve: None }] }],
    };
    assert!(matches!(c.make_face(open), Err(TopoError::InvalidLoop(_))));
    c.end_op().unwrap();
    assert_valid(&c);
}

#[test]
fn merge_faces_refuses_an_occupied_side() {
    // Merge two bricks' faces with the wrong relative orientation: B's volume would land on A's occupied side.
    let mut c = new_model();
    let a = brick(&mut c, OpId(1), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let b = brick(&mut c, OpId(2), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 1.0, 1.0)).unwrap();
    c.begin_op(OpId(3)).unwrap();
    for k in 0..4 {
        c.merge_vertices(a.vertices[[1, 3, 5, 7][k]], b.vertices[[0, 2, 4, 6][k]]).unwrap();
    }
    for (ea, eb) in [((1, 3), (0, 2)), ((5, 7), (4, 6)), ((1, 5), (0, 4)), ((3, 7), (2, 6))] {
        c.merge_edges(a.edge(ea.0, ea.1).unwrap(), b.edge(eb.0, eb.1).unwrap()).unwrap();
    }
    // Same-direction normals would put B's volume on A's occupied (Negative) side.
    let err = c.merge_faces(a.faces[1], b.faces[0], Sense::Forward).unwrap_err();
    assert!(matches!(err, TopoError::SideOccupied(FaceUse { side: FaceSide::Negative, .. })));
    // the failed attempt changed nothing; the right orientation succeeds
    c.merge_faces(a.faces[1], b.faces[0], Sense::Reversed).unwrap();
    c.end_op().unwrap();
    assert_valid(&c);
}

#[test]
fn ids_are_typed_and_virtual_range_is_reserved() {
    let f = FaceId::new(3).unwrap();
    assert_eq!(f.serial(), 3);
    assert_eq!(f.entity().kind(), EntityKind::TopoFace);
    assert!(EdgeId::from_entity(f.entity()).is_none());
    assert!(!f.is_virtual());
    assert!(FaceId::new(VIRTUAL_SERIAL_BIT | 1).unwrap().is_virtual());
    assert!(VertexId::new(EntityId::MAX_SERIAL + 1).is_none());
    assert_eq!(FaceSide::Negative.opposite(), FaceSide::Positive);
    assert_eq!(FaceSide::Negative.sense(), UseSense::Aligned);
    assert_eq!(FaceUse { face: f, side: FaceSide::Negative }.mate().side, FaceSide::Positive);
    assert_eq!(Sense::Reversed.compose(Sense::Reversed), Sense::Forward);
}

/// A view wrapper that corrupts one derived index, to prove the validator checks derived data against primary records.
struct Broken<'a> {
    inner: &'a RefComplex,
    drop_ring_entry: Option<EdgeId>,
    ring: Vec<EdgeUse>,
}

impl TopologyView for Broken<'_> {
    fn tolerance(&self) -> &ModelTolerance {
        self.inner.tolerance()
    }
    fn shapes(&self) -> &dyn ShapeStore {
        self.inner.shapes()
    }
    fn serial_count(&self, k: EntityKind) -> u64 {
        self.inner.serial_count(k)
    }
    fn bodies(&self) -> &[BodyId] {
        self.inner.bodies()
    }
    fn volumes(&self) -> &[VolumeId] {
        self.inner.volumes()
    }
    fn faces(&self) -> &[FaceId] {
        self.inner.faces()
    }
    fn edges(&self) -> &[EdgeId] {
        self.inner.edges()
    }
    fn vertices(&self) -> &[VertexId] {
        self.inner.vertices()
    }
    fn body(&self, id: BodyId) -> Option<&BodyRec> {
        self.inner.body(id)
    }
    fn volume(&self, id: VolumeId) -> Option<&VolumeRec> {
        self.inner.volume(id)
    }
    fn face(&self, id: FaceId) -> Option<&FaceRec> {
        self.inner.face(id)
    }
    fn edge(&self, id: EdgeId) -> Option<&EdgeRec> {
        self.inner.edge(id)
    }
    fn vertex(&self, id: VertexId) -> Option<&VertexRec> {
        self.inner.vertex(id)
    }
    fn radial_ring(&self, e: EdgeId) -> &[EdgeUse] {
        if self.drop_ring_entry == Some(e) { &self.ring } else { self.inner.radial_ring(e) }
    }
    fn edge_coedges(&self, e: EdgeId) -> &[CoedgeRef] {
        self.inner.edge_coedges(e)
    }
    fn vertex_uses(&self, v: VertexId) -> &[VertexUse] {
        self.inner.vertex_uses(v)
    }
    fn coincidences(&self) -> &[CoincidenceRecord] {
        self.inner.coincidences()
    }
    fn lineage(&self) -> &LineageLog {
        self.inner.lineage()
    }
}

#[test]
fn validator_detects_corrupt_rings_and_wedges() {
    let (c, b) = single_brick();
    let e = b.edge(0, 1).unwrap();
    let mut short = c.radial_ring(e).to_vec();
    short.pop();
    let broken = Broken { inner: &c, drop_ring_entry: Some(e), ring: short };
    let v = validate(&broken);
    assert!(v.iter().any(|x| x.code == ViolationCode::RadialRing && x.entity == Some(e.entity())), "{v:?}");
    // swapping two non-mate uses breaks wedge consistency
    let mut swapped = c.radial_ring(e).to_vec();
    swapped.swap(1, 2);
    let broken = Broken { inner: &c, drop_ring_entry: Some(e), ring: swapped };
    assert!(validate(&broken).iter().any(|x| x.code == ViolationCode::RadialRing));
}

#[test]
fn lineage_queries_on_a_handmade_log() {
    let mk = |s| EntityId::new(EntityKind::TopoFace, s).unwrap();
    let mut log = LineageLog::new();
    log.push(OpId(1), LineageEvent::Created { entity: mk(0) });
    log.push(OpId(2), LineageEvent::Split { parent: mk(0), children: vec![mk(1), mk(2)] });
    log.push(OpId(3), LineageEvent::Merged { survivor: mk(1), absorbed: mk(3), gap: 1e-9 });
    log.push(OpId(4), LineageEvent::Deleted { entity: mk(2) });
    assert_eq!(log.current_of(mk(0)), vec![mk(1)]);
    assert_eq!(log.current_of(mk(3)), vec![mk(1)]);
    assert!(log.current_of(mk(2)).is_empty());
    assert_eq!(log.ancestors_of(mk(1)), vec![mk(0), mk(3)]);
    assert_eq!(log.records()[3].seq, 0);
    log.push(OpId(4), LineageEvent::Deleted { entity: mk(1) });
    assert_eq!(log.records()[4].seq, 1, "seq counts within an op");
}
