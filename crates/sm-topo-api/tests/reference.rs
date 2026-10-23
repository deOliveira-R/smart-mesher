//! Reference-implementation tests: brick, merged bricks, imprint+merge, split with lineage,
//! cylinder seam, baffle, overlay, determinism, error atomicity.

use core::f64::consts::TAU;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{GeometryStore, Interval, Line, MemoryStore, Uv};
use sm_topo_api::fixtures::{self, BrickIds};
use sm_topo_api::*;

fn world() -> (CellComplex, MemoryStore) {
    (CellComplex::new(), MemoryStore::new())
}

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
}

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    Dim::ALL.map(|d| v.cells(d).len())
}

fn euler(v: &dyn TopologyView) -> i64 {
    let c = counts(v);
    c[0] as i64 - c[1] as i64 + c[2] as i64
}

#[test]
fn brick_is_a_valid_closed_cell() {
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0]).unwrap();
    assert_eq!(counts(&t), [8, 12, 6, 1, 1]);
    assert_eq!(euler(&t), 2);
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty(), "{:?}", validate_geometry(&t, &s));
    for f in b.faces {
        let sides = t.face_sides(f);
        assert_eq!(sides.left, Region::Volume(b.volume));
        assert_eq!(sides.right, Region::Exterior);
        assert!(t.is_boundary_face(f) && !t.is_baffle(f) && !t.is_interface_face(f));
        assert_eq!(t.loops(f).count(), 1);
        assert_eq!(t.coedges(f).count(), 4);
    }
    for e in b.edges {
        let star: Vec<_> = t.radial_uses(e).collect();
        assert_eq!(star.len(), 2, "each brick edge borders two faces");
        assert_ne!(star[0].sense, star[1].sense, "manifold edge: opposite coedge senses");
    }
    for v in b.vertices {
        assert_eq!(t.coboundary(v).len(), 3);
    }
    assert_eq!(t.face_uses(b.volume).count(), 6);
    assert!(t.face_uses(b.volume).all(|u| u.sense == Sense::Forward));
    assert_eq!(t.cells(Dim::Body), &[b.body]);
    assert_eq!(t.closure(b.volume).len(), 8 + 12 + 6 + 1);
    // Face normal from the carrier agrees with "volume on the left": +z face points up.
    let Carrier::Surface(sc) = t.carrier(b.faces[5]) else { panic!("no surface") };
    let n = s.surface(sc.surface).unwrap().normal(Uv::new(0.0, 0.0)).unwrap();
    assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
}

#[test]
fn two_bricks_imprinted_and_merged_share_one_face() {
    let (mut t, mut s) = world();
    let (a, b) = fixtures::two_bricks(&mut t, &mut s).unwrap();
    assert_eq!(counts(&t), [16, 24, 12, 2, 2]);
    assert_eq!(t.coincidence_between(a.faces[1], b.faces[0]), Some(Coincidence::CoincidentUnimprinted));
    assert!(t.is_boundary_face(a.faces[1]) && t.is_boundary_face(b.faces[0]));
    t.begin_op(OpId(5));
    fixtures::merge_touching_bricks(&mut t, &a, &b).unwrap();
    // 4 vertices, 4 edges and 1 face disappear into their coincident partners.
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty());

    let shared = a.faces[1];
    assert!(!t.is_alive(b.faces[0]));
    let sides = t.face_sides(shared);
    assert_eq!(sides.left, Region::Volume(a.volume), "normal points from the lower volume id");
    assert_eq!(sides.right, Region::Volume(b.volume));
    assert!(t.is_interface_face(shared));
    let uses = t.face_uses_of(shared);
    assert_eq!(uses.len(), 2);
    let sense_of = |vol: EntityId| uses.iter().find(|u| u.volume == vol).unwrap().sense;
    assert_eq!(sense_of(a.volume), Sense::Forward);
    assert_eq!(sense_of(b.volume), Sense::Reversed, "opposite FaceUse senses on the shared face");
    // Both volumes still close.
    assert_eq!(t.face_uses(a.volume).count(), 6);
    assert_eq!(t.face_uses(b.volume).count(), 6);

    // Imprint/merge status and lineage.
    assert_eq!(t.coincidence_between(a.faces[1], b.faces[0]), Some(Coincidence::Merged));
    let merged: Vec<_> = t.lineage().iter().filter(|r| r.kind == LineageKind::Merged).collect();
    assert_eq!(merged.len(), 4 + 4 + 1);
    let last = merged.last().unwrap();
    assert_eq!((last.op, last.from.as_slice(), last.to.as_slice()), (OpId(5), &[shared, b.faces[0]][..], &[shared][..]));
    assert_eq!(t.descendants(b.faces[0]), vec![shared]);
    // Shared edges now have four coedges (two per volume's face pair): radial star of 2 faces per volume, shared => 3 faces.
    let shared_edge = a.edges[5];
    assert_eq!(t.radial_uses(shared_edge).count(), 3);
}

#[test]
fn merge_orients_the_shared_face_from_lower_to_higher_volume() {
    // Build the right-hand brick first so it gets the lower volume id.
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let a = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    assert!(b.volume < a.volume);
    fixtures::merge_touching_bricks(&mut t, &a, &b).unwrap();
    assert_valid(&t);
    let sides = t.face_sides(a.faces[1]);
    assert_eq!(sides.left, Region::Volume(b.volume));
    assert_eq!(sides.right, Region::Volume(a.volume));
    // The face was reversed, so its carrier sense flipped and a Modified record exists.
    let Carrier::Surface(sc) = t.carrier(a.faces[1]) else { panic!() };
    assert_eq!(sc.sense, Sense::Reversed);
    assert!(t.lineage().iter().any(|r| r.kind == LineageKind::Modified));
    assert!(validate_geometry(&t, &s).is_empty());
}

#[test]
fn imprint_then_merge_a_smaller_brick() {
    let (mut t, mut s) = world();
    let a = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(1.0, 0.0, 0.0), [1.0, 1.0, 0.5]).unwrap();
    t.record_coincidence(a.faces[1], b.faces[0], Coincidence::Overlapping, Some(0.5)).unwrap();
    assert_eq!(t.coincidence_between(a.faces[1], b.faces[0]), Some(Coincidence::Overlapping));

    // Imprint B's footprint on A's +x face: split two z-edges at z = 0.5 and add a chord.
    t.begin_op(OpId(1));
    let m0 = t.make_vertex(Point3::new(1.0, 0.0, 0.5)).unwrap();
    let m1 = t.make_vertex(Point3::new(1.0, 1.0, 0.5)).unwrap();
    let [z0_lo, _z0_hi] = t.split_edge(a.edges[9], m0, 0.5).unwrap();
    let [z1_lo, _z1_hi] = t.split_edge(a.edges[11], m1, 0.5).unwrap();
    let line = s.add_curve(Box::new(Line::new(Point3::new(1.0, 0.0, 0.5), Vector3::new(0.0, 1.0, 0.0)).unwrap())).unwrap();
    let chord = t.make_edge(m0, m1, Some(CurveCarrier { curve: line, range: Interval::new(0.0, 1.0), sense: Sense::Forward })).unwrap();
    t.begin_op(OpId(2));
    let [f1, f2] = t.split_face(a.faces[1], chord).unwrap();
    assert_valid(&t);
    assert_eq!(euler(&t), 4, "two spheres; splits keep each Euler characteristic");
    assert_eq!(t.descendants(a.faces[1]), vec![f1, f2]);
    let lower = if t.coedges(f1).any(|c| c.edge == a.edges[5]) { f1 } else { f2 };
    t.record_coincidence(lower, b.faces[0], Coincidence::ImprintedNotMerged, None).unwrap();
    assert!(t.coincidences().iter().all(|p| p.status != Coincidence::Overlapping), "records of the split face are dropped");

    // Merge bottom-up.
    t.begin_op(OpId(3));
    for j in 0..2 {
        t.merge_vertices(a.vertices[1 + 2 * j], b.vertices[2 * j]).unwrap();
    }
    t.merge_vertices(m0, b.vertices[4]).unwrap();
    t.merge_vertices(m1, b.vertices[6]).unwrap();
    t.merge_edges(a.edges[5], b.edges[4], Sense::Forward).unwrap();
    t.merge_edges(chord, b.edges[6], Sense::Forward).unwrap();
    t.merge_edges(z0_lo, b.edges[8], Sense::Forward).unwrap();
    t.merge_edges(z1_lo, b.edges[10], Sense::Forward).unwrap();
    t.merge_faces(lower, b.faces[0], Sense::Reversed).unwrap();
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty(), "{:?}", validate_geometry(&t, &s));
    let sides = t.face_sides(lower);
    assert_eq!((sides.left, sides.right), (Region::Volume(a.volume), Region::Volume(b.volume)));
    let upper = if lower == f1 { f2 } else { f1 };
    assert!(t.is_boundary_face(upper));
    assert_eq!(t.coincidence_between(lower, b.faces[0]), Some(Coincidence::Merged));
    // A's shell has 7 faces now (5 original + 2 children), B has 6.
    assert_eq!(t.face_uses(a.volume).count(), 7);
    assert_eq!(t.face_uses(b.volume).count(), 6);
}

#[test]
fn face_split_records_lineage_and_ids_deterministically() {
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    // Diagonal chord across the +z face from corner 4 = (0,0,1) to corner 7 = (1,1,1).
    let diag = s.add_curve(Box::new(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())).unwrap();
    let chord = t.make_edge(b.vertices[4], b.vertices[7], Some(CurveCarrier { curve: diag, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward })).unwrap();
    t.begin_op(OpId(47));
    let [f1, f2] = t.split_face(b.faces[5], chord).unwrap();

    assert_eq!((f1.serial(), f2.serial()), (6, 7), "next unused face serials, in documented order");
    assert!(!t.is_alive(b.faces[5]) && t.is_alive(f1) && t.is_alive(f2));
    let rec = t.lineage().last().unwrap();
    assert_eq!(rec.kind, LineageKind::Split);
    assert_eq!(rec.op, OpId(47));
    assert_eq!((rec.from.as_slice(), rec.to.as_slice()), (&[b.faces[5]][..], &[f1, f2][..]));
    assert_eq!(t.lineage_of(b.faces[5]).len(), 1);
    assert_eq!(t.born(f1), Some(OpId(47)));
    assert_eq!(t.born(b.faces[0]), Some(OpId(0)));
    for f in [f1, f2] {
        assert_eq!(t.loops(f).next().unwrap().len(), 3, "triangle");
        assert_eq!(t.face_sides(f).left, Region::Volume(b.volume));
        assert_eq!(t.carrier(f), t.carrier(f1), "children share the parent's surface carrier");
    }
    let star: Vec<_> = t.radial_uses(chord).collect();
    assert_eq!(star.len(), 2);
    assert_ne!(star[0].sense, star[1].sense);
    assert_eq!(t.face_uses(b.volume).count(), 7);
    assert_eq!(euler(&t), 2);
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty());
    // Chord ends outside the loop are rejected, atomically.
    let before = t.canonical_hash();
    let stray = t.make_vertex(Point3::new(5.0, 5.0, 5.0)).unwrap();
    let bad = t.make_edge(stray, b.vertices[0], None).unwrap();
    let hash_with_stray = t.canonical_hash();
    assert_ne!(before, hash_with_stray);
    assert_eq!(t.split_face(f1, bad), Err(TopoError::ChordNotOnLoop));
    assert_eq!(t.canonical_hash(), hash_with_stray, "failed edits change nothing");
}

#[test]
fn cylinder_has_a_seam_used_twice_and_closed_edges() {
    let (mut t, mut s) = world();
    let c = fixtures::cylinder(&mut t, &mut s, 2.0, 5.0).unwrap();
    assert_eq!(counts(&t), [2, 3, 3, 1, 1]);
    assert_eq!(euler(&t), 2);
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty(), "{:?}", validate_geometry(&t, &s));

    let lateral = c.faces[2];
    assert_eq!(t.seam_edges(lateral).as_slice(), &[c.edges[2]]);
    let seam_uses: Vec<_> = t.coedges(lateral).filter(|ce| ce.edge == c.edges[2]).collect();
    assert_eq!(seam_uses.len(), 2);
    assert_ne!(seam_uses[0].sense, seam_uses[1].sense);
    assert_eq!(t.radial_uses(c.edges[2]).count(), 2, "both coedges belong to the same face");
    // Closed circles: one vertex, single Both entry.
    for e in [c.edges[0], c.edges[1]] {
        assert_eq!(t.boundary(e).len(), 1);
        assert_eq!(t.boundary(e)[0].side, Side::Both);
        let (a, b) = t.edge_ends(e).unwrap();
        assert_eq!(a, b);
    }
    assert!(t.seam_edges(c.faces[0]).is_empty() && t.loops(c.faces[0]).next().unwrap().len() == 1);
    // The carrier surface is periodic in u and its seam closes.
    let Carrier::Surface(sc) = t.carrier(lateral) else { panic!() };
    assert_eq!(sc.surface, c.lateral_surface);
    let surf = s.surface(sc.surface).unwrap();
    assert_eq!(surf.domain().u.period, Some(TAU));
    assert!((surf.eval(Uv::new(0.0, 1.0)) - surf.eval(Uv::new(TAU, 1.0))).norm() < 1e-12);
    // Bottom cap normal points down, consistent with the volume on its left.
    let Carrier::Surface(bot) = t.carrier(c.faces[0]) else { panic!() };
    let n = s.surface(bot.surface).unwrap().normal(Uv::new(0.0, 0.0)).unwrap();
    assert!((n - Vector3::new(0.0, 0.0, -1.0)).norm() < 1e-12);
    for f in c.faces {
        assert_eq!(t.face_sides(f).left, Region::Volume(c.volume));
    }
    // A cylinder edge split at the seam circle's opposite point keeps the loop closed.
    t.begin_op(OpId(1));
    let mid = t.make_vertex(Point3::new(-2.0, 0.0, 0.0)).unwrap();
    let [e1, e2] = t.split_edge(c.edges[0], mid, core::f64::consts::PI).unwrap();
    assert_valid(&t);
    assert_eq!(t.edge_ends(e1), Some((c.vertices[0], mid)));
    assert_eq!(t.edge_ends(e2), Some((mid, c.vertices[0])));
    assert_eq!(t.loops(c.faces[0]).next().unwrap().len(), 2);
    assert!(validate_geometry(&t, &s).is_empty(), "{:?}", validate_geometry(&t, &s));
}

#[test]
fn baffle_is_a_face_with_the_same_volume_on_both_sides() {
    let (mut t, mut s) = world();
    let (b, baffle) = fixtures::brick_with_baffle(&mut t, &mut s).unwrap();
    assert_valid(&t);
    assert!(validate_geometry(&t, &s).is_empty());
    let sides = t.face_sides(baffle.face);
    assert_eq!(sides.left, Region::Volume(b.volume));
    assert_eq!(sides.right, Region::Volume(b.volume));
    assert!(t.is_baffle(baffle.face) && !t.is_boundary_face(baffle.face) && !t.is_interface_face(baffle.face));
    let e = t.coboundary(baffle.face);
    assert_eq!(e.len(), 1);
    assert_eq!(e[0].side, Side::Both);
    let uses = t.face_uses_of(baffle.face);
    assert_eq!(uses.len(), 2);
    assert!(uses.iter().all(|u| u.volume == b.volume));
    assert_ne!(uses[0].sense, uses[1].sense);
    assert_eq!(t.face_uses(b.volume).count(), 8, "6 boundary uses + 2 baffle uses");
    // Boundary faces are unaffected.
    for f in b.faces {
        assert!(t.is_boundary_face(f));
    }
    // Removing it restores a plain brick.
    t.remove_face_use(b.volume, baffle.face).unwrap();
    assert!(t.is_free_face(baffle.face));
    assert_valid(&t);
    // A free sheet can be put in a body and a side cannot be taken twice.
    assert_eq!(t.make_volume(&[&[FaceUseSpec { face: b.faces[0], side: Side::Left }]]), Err(TopoError::SideOccupied { face: b.faces[0] }));
}

fn build_merged() -> (CellComplex, MemoryStore, BrickIds, BrickIds) {
    let (mut t, mut s) = world();
    let (a, b) = fixtures::two_bricks(&mut t, &mut s).unwrap();
    t.begin_op(OpId(9));
    fixtures::merge_touching_bricks(&mut t, &a, &b).unwrap();
    (t, s, a, b)
}

#[test]
fn building_twice_gives_identical_ids_and_hashes() {
    let (t1, _, a1, b1) = build_merged();
    let (t2, _, a2, b2) = build_merged();
    assert_eq!((a1, b1), (a2, b2));
    for d in Dim::ALL {
        assert_eq!(t1.cells(d), t2.cells(d));
    }
    assert_eq!(t1.canonical_hash(), t2.canonical_hash());
    assert_eq!(t1.lineage(), t2.lineage());
    // Independent of threads: build on several threads at once.
    let hashes: Vec<u64> = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..4).map(|_| sc.spawn(|| build_merged().0.canonical_hash())).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    assert!(hashes.iter().all(|h| *h == t1.canonical_hash()));
    // The hash is sensitive: a different op number or geometry changes it.
    let (mut t3, mut s3) = world();
    let (a3, b3) = fixtures::two_bricks(&mut t3, &mut s3).unwrap();
    t3.begin_op(OpId(10));
    fixtures::merge_touching_bricks(&mut t3, &a3, &b3).unwrap();
    assert_ne!(t1.canonical_hash(), t3.canonical_hash());
    // Cloning preserves the hash; the derived index is rebuilt lazily.
    assert_eq!(t1.clone().canonical_hash(), t1.canonical_hash());
}

#[test]
fn random_edge_splits_keep_the_complex_valid() {
    let (mut t, mut s) = world();
    fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0]).unwrap();
    let mut state = 12345u64;
    let mut next = move || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 33) as usize
    };
    for step in 0..30u64 {
        t.begin_op(OpId(step));
        let edges = t.cells(Dim::Edge).to_vec();
        let e = edges[next() % edges.len()];
        let Carrier::Curve(cc) = t.carrier(e) else { panic!() };
        let frac = 0.2 + 0.6 * ((next() % 100) as f64 / 100.0);
        let param = cc.range.lo + frac * cc.range.len();
        let curve = s.curve(cc.curve).unwrap();
        let v = t.make_vertex(curve.eval(param)).unwrap();
        t.split_edge(e, v, param).unwrap();
        assert_valid(&t);
        assert_eq!(euler(&t), 2);
    }
    assert!(validate_geometry(&t, &s).is_empty());
    assert_eq!(counts(&t)[1], 12 + 30);
    assert_eq!(t.lineage().len(), 30);
}

#[test]
fn overlay_composites_faces_without_touching_the_host() {
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let diag = s.add_curve(Box::new(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())).unwrap();
    let chord = t.make_edge(b.vertices[4], b.vertices[7], Some(CurveCarrier { curve: diag, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward })).unwrap();
    let [f1, f2] = t.split_face(b.faces[5], chord).unwrap();
    let host_hash = t.canonical_hash();
    {
        let mut o = Overlay::new(&t);
        assert_eq!(o.cells(Dim::Face).len(), 7, "empty overlay equals the host");
        let vf = o.composite_faces(&[f1, f2]).unwrap();
        assert_eq!(vf.kind(), sm_core::EntityKind::PartitionCell);
        assert_eq!(o.dim_of(vf), Some(Dim::Face));
        assert_eq!(o.cells(Dim::Face).len(), 6);
        assert!(!o.is_alive(f1) && !o.is_alive(f2) && !o.is_alive(chord));
        assert_eq!(o.virtual_members(vf), &[f1, f2][..]);
        assert_eq!((o.resolve(f1), o.resolve(chord), o.resolve(b.faces[0])), (vf, vf, b.faces[0]));
        assert_eq!(o.coedges(vf).count(), 4);
        assert_eq!(o.face_sides(vf).left, Region::Volume(b.volume));
        assert_eq!(o.face_uses(b.volume).count(), 6);
        assert!(o.face_uses(b.volume).any(|u| u.face == vf));
        assert_eq!(o.carrier(vf), t.carrier(f1));
        assert_eq!(o.coboundary(b.vertices[4]).len(), 3, "the hidden chord no longer touches its vertices");
        assert_eq!(o.cells(Dim::Edge).len(), 12);
        assert_eq!(validate(&o), vec![], "overlay is a valid complex");
        assert!(validate_geometry(&o, &s).is_empty());
        // Host view through the overlay is unchanged.
        assert_eq!(o.host().canonical_hash(), host_hash);
        o.clear();
        assert_eq!(o.cells(Dim::Face).len(), 7);
    }
    assert_eq!(t.canonical_hash(), host_hash);
    assert_eq!(t.cells(Dim::Face).len(), 7);
    // Composite needs shared edges.
    let o = Overlay::new(&t);
    let mut o = o;
    assert!(matches!(o.composite_faces(&[b.faces[0], b.faces[1]]), Err(TopoError::InvalidParameter(_))));
}

#[test]
fn edits_are_checked_and_atomic() {
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let h = t.canonical_hash();
    // Open loop.
    let l = [CoedgeSpec { edge: b.edges[0], sense: Sense::Forward }, CoedgeSpec { edge: b.edges[1], sense: Sense::Forward }];
    assert!(matches!(t.make_face(&[&l], None), Err(TopoError::OpenLoop { .. })));
    assert_eq!(t.make_face(&[], None), Err(TopoError::EmptyLoop));
    // Wrong ranks.
    assert!(matches!(t.make_edge(b.faces[0], b.vertices[0], None), Err(TopoError::WrongDim { .. })));
    // Far-apart vertices do not merge; same-cell merge refused.
    assert!(matches!(t.merge_vertices(b.vertices[0], b.vertices[7]), Err(TopoError::NotCoincident(_))));
    assert!(matches!(t.merge_vertices(b.vertices[0], b.vertices[0]), Err(TopoError::InvalidParameter(_))));
    // Cells in use cannot be deleted; split parameter must be inside the range.
    assert_eq!(t.kill(b.faces[0]), Err(TopoError::StillReferenced(b.faces[0])));
    let m = t.make_vertex(Point3::new(0.5, 0.0, 0.0)).unwrap();
    assert!(matches!(t.split_edge(b.edges[0], m, 5.0), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.split_edge(b.edges[0], b.vertices[0], 0.5), Err(TopoError::InvalidParameter(_))));
    assert!(matches!(t.set_tolerance(b.vertices[0], Some(-1.0)), Err(TopoError::InvalidParameter(_))));
    // Failed edits changed nothing except the vertex we made on purpose.
    t.kill(m).unwrap();
    assert_ne!(t.canonical_hash(), h, "kill leaves a lineage record");
    assert_eq!(t.lineage().last().unwrap().kind, LineageKind::Deleted);
    // Dead cells answer with empty rows and are rejected by edits.
    assert!(t.boundary(m).is_empty() && !t.is_alive(m));
    assert_eq!(t.kill(m), Err(TopoError::Dead(m)));
    assert_eq!(t.kill(EntityId::new(sm_core::EntityKind::TopoVertex, 999).unwrap()), Err(TopoError::NoSuchCell(EntityId::new(sm_core::EntityKind::TopoVertex, 999).unwrap())));
}

#[test]
fn tolerance_is_global_with_sparse_overrides() {
    let (mut t, mut s) = world();
    let b = fixtures::brick(&mut t, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let global = t.model_tolerance().linear;
    assert_eq!(t.tolerance(b.vertices[0]), global);
    assert_eq!(t.tolerance_override(b.vertices[0]), None);
    t.set_tolerance(b.vertices[0], Some(1e-3)).unwrap();
    assert_eq!(t.tolerance(b.vertices[0]), 1e-3);
    // A tolerant vertex can absorb a slightly displaced neighbour; the survivor keeps the larger tolerance.
    let near = t.make_vertex(Point3::new(5e-4, 0.0, 0.0)).unwrap();
    t.merge_vertices(b.vertices[0], near).unwrap();
    assert_eq!(t.tolerance(b.vertices[0]), 1e-3);
    t.set_tolerance(b.vertices[0], None).unwrap();
    assert_eq!(t.tolerance(b.vertices[0]), global);
}

#[test]
fn view_is_object_safe_and_edit_is_usable_through_dyn() {
    let (mut t, mut s) = world();
    let dynedit: &mut dyn TopologyEdit = &mut t;
    let b = fixtures::brick(dynedit, &mut s, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0]).unwrap();
    let view: &dyn TopologyView = &t;
    assert_eq!(view.cells(Dim::Volume), &[b.volume]);
    assert_eq!(view.face_sides(b.faces[0]).left, Region::Volume(b.volume));
    let g: &dyn GeometryStore = &s;
    assert!(g.surface(EntityId::new(sm_core::EntityKind::Surface, 0).unwrap()).is_some());
}
