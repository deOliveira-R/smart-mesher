//! Reference-implementation tests: brick, imprinted+merged bricks, webcut with lineage, cylinder
//! seam, sphere poles, baffle, non-manifold fin, CSG volume, atomicity, tolerance and gap
//! recording, determinism and hash quantization, performance, and every semantics query.

use core::f64::consts::{PI, TAU};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CsgCell, Crossing, CurveDef, Frame, HalfSpace, ImplicitDef, Interval, Line, ModelTolerance, Quadric, RayHit, Sign, Sphere, SurfaceDef, Uv};
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use sm_topo_api::*;

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
    let bad = validate_geometry(v);
    assert!(bad.is_empty(), "geometry violations: {bad:#?}");
}

fn counts(v: &dyn TopologyView) -> [usize; 5] {
    Dim::ALL.map(|d| v.cells(d).len())
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn unit_brick(t: &mut CellComplex, op: u64, min: [f64; 3], size: [f64; 3]) -> BrickIds {
    with_op(t, OpId(op), |t| fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size)).unwrap()
}

// ---------------------------------------------------------------- brick and semantics queries

#[test]
fn brick_is_a_valid_closed_cell_and_answers_every_semantics_query() {
    let mut t = CellComplex::new();
    let b = unit_brick(&mut t, 1, [0.0, 0.0, 0.0], [2.0, 3.0, 4.0]);
    assert_eq!(counts(&t), [8, 12, 6, 1, 1]);
    assert_eq!(euler_characteristic(&t), 2);
    assert_valid(&t);
    // volumes / faces / volume_faces / face_edges / face_ordinal
    assert_eq!(t.volumes(), &[b.volume]);
    assert_eq!(t.faces(), &b.faces);
    assert_eq!(t.volume_faces(b.volume), &b.faces[..], "ascending, index-backed slice");
    assert_eq!(t.volume_faces(EntityId::new(sm_core::EntityKind::TopoVolume, 99).unwrap()), &[] as &[EntityId], "unknown id gives an empty slice");
    for (i, f) in b.faces.iter().enumerate() {
        assert_eq!(t.face_ordinal(*f), Some(i));
        let edges = t.face_edges(*f);
        assert_eq!(edges.len(), 4);
        assert!(edges.windows(2).all(|w| w[0] < w[1]));
        let sides = t.face_sides(*f).unwrap();
        assert_eq!((sides.left, sides.right), (Region::Volume(b.volume), Region::Exterior));
        assert_eq!(t.face_class(*f), Some(FaceClass::Boundary));
        assert_eq!(t.loops(*f).count(), 1);
        assert_eq!(t.coedges(*f).count(), 4);
        assert_eq!(t.face_outline(*f).len(), 4);
        assert_eq!(t.face_use_in(*f, b.volume).as_slice(), &[Sense::Forward]);
    }
    assert_eq!(t.face_ordinal(b.volume), None);
    assert_eq!(t.boundary_faces().count(), 6);
    // Face moments: areas, outward unit normals.
    let area: f64 = b.faces.iter().map(|f| t.face_moments(*f).unwrap().area).sum();
    assert!(close(area, 2.0 * (6.0 + 8.0 + 12.0), 1e-9));
    let fm = t.face_moments(b.faces[5]).unwrap(); // +z, 2 x 3 at z = 4
    assert!(close(fm.area, 6.0, 1e-10));
    let c = fm.centroid().unwrap();
    assert!(close(c.x, 1.0, 1e-10) && close(c.y, 1.5, 1e-10) && close(c.z, 4.0, 1e-10));
    let (n, d) = fm.plane(1e-9).expect("planar");
    assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-10 && close(d, 4.0, 1e-10));
    assert!(close(fm.vector_area.z, 6.0, 1e-10));
    assert!(close(fm.second[0][0], 4.0 / 3.0 * 3.0 * 2.0, 1e-9), "int x^2 dA = a^3 b / 3 = 8");
    // Volume moments against the analytic box [0,2]x[0,3]x[0,4].
    let m = t.volume_moments(b.volume).unwrap();
    assert!(close(m.volume, 24.0, 1e-9));
    let cen = m.centroid().unwrap();
    assert!(close(cen.x, 1.0, 1e-9) && close(cen.y, 1.5, 1e-9) && close(cen.z, 2.0, 1e-9));
    assert!(close(m.second[0][0], 32.0, 1e-8) && close(m.second[1][1], 72.0, 1e-8) && close(m.second[2][2], 128.0, 1e-8));
    assert!(close(m.second[0][1], 36.0, 1e-8) && close(m.second[0][2], 48.0, 1e-8) && close(m.second[1][2], 72.0, 1e-8));
    let cs = m.central_second();
    assert!(close(cs[0][0], 24.0 * 4.0 / 12.0, 1e-8) && close(cs[1][1], 24.0 * 9.0 / 12.0, 1e-8) && close(cs[0][1], 0.0, 1e-8));
    // Each brick edge has two faces around it with opposite coedge senses.
    for e in b.edges {
        let ring = t.radial_uses(e);
        assert_eq!(ring.len(), 2);
        assert_ne!(ring[0].sense, ring[1].sense);
        assert!(ring[0].face < ring[1].face, "ring starts at the lowest face id");
    }
    // face_across_edge: turning around the x-edge at (j=0,k=0) inside the brick.
    let e = b.edges[0];
    let ring = t.radial_uses(e);
    let (f0, f1) = (ring[0].face, ring[1].face);
    assert_eq!(t.face_across_edge(f0, e, b.volume), Some(f1));
    assert_eq!(t.face_across_edge(f1, e, b.volume), Some(f0));
    assert_eq!(t.face_across_edge(f0, e, EntityId::new(sm_core::EntityKind::TopoVolume, 7).unwrap()), None);
    for v in b.vertices {
        assert_eq!(t.coboundary(v).len(), 3);
    }
    assert_eq!(t.face_uses(b.volume).count(), 6);
    assert_eq!(t.cells(Dim::Body), &[b.body]);
    assert_eq!(t.closure(b.volume).len(), 8 + 12 + 6 + 1);
    assert_eq!(t.state(b.volume), Some(EntityState::Live));
    assert_eq!(t.born(b.volume), Some(OpId(1)));
    assert_eq!(t.revision(), OpId(1));
    // Face normal from the carrier agrees with "volume on the left": +z face points up.
    let Carrier::Surface(sc) = t.carrier(b.faces[5]) else { panic!("no surface") };
    let n = t.shapes().surface(sc.surface).unwrap().normal(Uv::new(0.0, 0.0)).unwrap();
    assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    // classify_point_on_face on the +z face (plane frame u,v = x,y from the corner).
    assert_eq!(t.classify_point_on_face(b.faces[5], Uv::new(1.0, 1.0)).unwrap(), PointClass::In);
    assert_eq!(t.classify_point_on_face(b.faces[5], Uv::new(5.0, 1.0)).unwrap(), PointClass::Out);
    assert_eq!(t.classify_point_on_face(b.faces[5], Uv::new(2.0, 1.5)).unwrap(), PointClass::On);
}

// ---------------------------------------------------------------- imprint + merge

#[test]
fn two_bricks_imprinted_and_merged_share_one_face() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    assert_eq!(counts(&t), [16, 24, 12, 2, 2]);
    assert_eq!(t.coincidence(a.faces[1], b.faces[0]).map(|p| p.status), Some(Coincidence::CoincidentUnimprinted));
    assert_eq!(t.coincident_faces(a.faces[1]).len(), 1);
    assert!(t.is_boundary_face(a.faces[1]) && t.is_boundary_face(b.faces[0]));
    with_op(&mut t, OpId(5), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    // 4 vertices, 4 edges and 1 face disappear into their coincident partners.
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    assert_valid(&t);

    let shared = a.faces[1];
    assert_eq!(t.state(b.faces[0]), Some(EntityState::Merged(OpId(5))));
    let sides = t.face_sides(shared).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(a.volume), Region::Volume(b.volume)), "left = lower volume id");
    assert_eq!(t.face_class(shared), Some(FaceClass::Interface));
    // Two sides: sideset w.r.t. a volume picks the right side.
    let uses = t.face_uses_of(shared);
    assert_eq!(uses.len(), 2);
    assert_eq!((uses[0].volume, uses[0].sense), (a.volume, Sense::Forward), "A is on the left (normal out of A)");
    assert_eq!((uses[1].volume, uses[1].sense), (b.volume, Sense::Reversed), "B is on the right (normal into B)");
    assert_eq!(t.face_use_in(shared, a.volume).as_slice(), &[Sense::Forward]);
    assert_eq!(t.face_use_in(shared, b.volume).as_slice(), &[Sense::Reversed]);
    // Ray tracking helper: crossing the shared face along +x enters B; along -x enters A.
    let Carrier::Surface(sc) = t.carrier(shared) else { panic!() };
    let n = t.shapes().surface(sc.surface).unwrap().normal(Uv::new(0.0, 0.0)).unwrap();
    let hit = |crossing| RayHit { t: 1.0, point: Point3::new(1.0, 0.5, 0.5), normal: n, uv: None, crossing };
    assert_eq!(t.entered_region(shared, &hit(Crossing::AlongNormal)), Some(Region::Volume(b.volume)));
    assert_eq!(t.entered_region(shared, &hit(Crossing::AgainstNormal)), Some(Region::Volume(a.volume)));
    assert_eq!(t.entered_region(shared, &hit(Crossing::Tangent)), None);
    // Both volumes still close.
    assert_eq!(t.face_uses(a.volume).count(), 6);
    assert_eq!(t.face_uses(b.volume).count(), 6);
    assert!(t.volume_faces(a.volume).contains(&shared) && t.volume_faces(b.volume).contains(&shared));
    // Imprint/merge status and lineage.
    let pair = t.coincidence(a.faces[1], b.faces[0]).unwrap();
    assert_eq!((pair.status, pair.op), (Coincidence::Merged, OpId(5)));
    assert_eq!(t.coincident_faces(shared).len(), 1);
    let merged: Vec<_> = t.lineage().records().iter().filter(|r| matches!(r.kind, LineageKind::Merged { .. })).collect();
    assert_eq!(merged.len(), 4 + 4 + 1);
    let last = merged.last().unwrap();
    assert_eq!((last.op, last.entity), (OpId(5), b.faces[0]));
    assert!(matches!(last.kind, LineageKind::Merged { into, gap } if into == shared && gap == 0.0));
    assert_eq!(t.live_descendants(b.faces[0]), vec![shared]);
    assert_eq!(t.lineage().parents(shared).as_slice(), &[b.faces[0]]);
    // The shared edges have three faces around them.
    assert_eq!(t.radial_uses(a.edges[5]).len(), 3);
    // Moments of the two volumes add up to the union box [0,2]x[0,1]x[0,1].
    let total = t.volume_moments(a.volume).unwrap().add(&t.volume_moments(b.volume).unwrap());
    assert!(close(total.volume, 2.0, 1e-9) && close(total.centroid().unwrap().x, 1.0, 1e-9));
}

#[test]
fn removing_a_use_reapplies_the_orientation_rule_and_logs_it() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    let shared = a.faces[1];
    // Remove A's use of the shared face: B alone remains, on the RIGHT, so the face flips at once
    // (D12). Then give A its use back from the right: end_op flips it again, lower volume first.
    with_op(&mut t, OpId(3), |t| {
        t.remove_face_use(a.volume, shared, Side::Left)?;
        let s = t.face_sides(shared).unwrap();
        assert_eq!((s.left, s.right), (Region::Volume(b.volume), Region::Exterior));
        let Carrier::Surface(sc) = t.carrier(shared) else { panic!() };
        assert_eq!(sc.sense, Sense::Reversed);
        t.add_face_use(a.volume, 0, FaceUseSpec { face: shared, side: Side::Right })?;
        Ok(())
    })
    .unwrap();
    let s = t.face_sides(shared).unwrap();
    assert_eq!((s.left, s.right), (Region::Volume(a.volume), Region::Volume(b.volume)));
    let flips = t.lineage().for_op(OpId(3)).iter().filter(|r| r.entity == shared && r.kind == LineageKind::Modified(ModKind::Reversed)).count();
    assert_eq!(flips, 2, "both flips are logged");
    assert_valid(&t);
    // A volume that claims every face of a brick from the right is flipped at end_op: the lone
    // occupant of a face always ends up on its left.
    let mut t2 = CellComplex::new();
    let b2 = unit_brick(&mut t2, 1, [0.0; 3], [1.0; 3]);
    with_op(&mut t2, OpId(2), |t| {
        for f in b2.faces {
            t.remove_face_use(b2.volume, f, Side::Left)?;
        }
        for f in b2.faces {
            t.add_face_use(b2.volume, 0, FaceUseSpec { face: f, side: Side::Right })?;
        }
        Ok(())
    })
    .unwrap();
    for f in b2.faces {
        let s = t2.face_sides(f).unwrap();
        assert_eq!((s.left, s.right), (Region::Volume(b2.volume), Region::Exterior));
        let Carrier::Surface(sc) = t2.carrier(f) else { panic!() };
        assert_eq!(sc.sense, Sense::Reversed);
    }
    assert_valid(&t2);
}

#[test]
fn merge_orients_the_shared_face_lowest_volume_first_after_an_imprint() {
    // Smaller brick B imprinted onto A's +x face through a closed loop (ClosedLoop), then merged.
    // B's face id is lower than the imprint disk, so B's face survives; the survivor then has B
    // (the higher volume) on its left and must be reversed.
    let mut t = CellComplex::new();
    let a = unit_brick(&mut t, 1, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let b = unit_brick(&mut t, 2, [1.0, 0.25, 0.25], [1.0, 0.5, 0.5]);
    assert!(a.volume < b.volume);
    let (disk, annulus) = with_op(&mut t, OpId(3), |t| {
        // Imprint vertices/edges on A's face x = 1, mirroring B's footprint.
        let pts = [(0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)];
        let vs: Vec<EntityId> = pts.iter().map(|&(y, z)| t.make_vertex(Point3::new(1.0, y, z))).collect::<Result<_, _>>().map_err(fixtures::FixtureError::from)?;
        let mut es = Vec::new();
        for i in 0..4 {
            let (p, q) = (pts[i], pts[(i + 1) % 4]);
            let d = Vector3::new(0.0, q.0 - p.0, q.1 - p.1);
            let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(1.0, p.0, p.1), d).unwrap())).unwrap();
            es.push(t.make_edge(vs[i], vs[(i + 1) % 4], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?);
        }
        // The loop that bounds the disk counter-clockwise seen from +x (the face's right side).
        let lp: Vec<CoedgeSpec> = es.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
        let [annulus, disk] = t.split_face(a.faces[1], FaceCut::ClosedLoop { edges: &lp }, InnerLoopAssign::None)?;
        // Merge the imprint into B's -x face: vertices, edges, then the face.
        for (i, v) in vs.iter().enumerate() {
            // B's -x corners (i=0): vertices[2j+4k] at (y,z) = (0.25 + 0.5 j, 0.25 + 0.5 k).
            let (j, k) = match i {
                0 => (0, 0),
                1 => (1, 0),
                2 => (1, 1),
                _ => (0, 1),
            };
            t.merge_vertices(b.vertices[2 * j + 4 * k], *v, None)?;
        }
        // Edge i of the imprint loop joins corners i and i+1: y-edges (4+2k) and z-edges (8+2j).
        let pairs = [(b.edges[4], es[0], Sense::Forward), (b.edges[8 + 2], es[1], Sense::Forward), (b.edges[4 + 2], es[2], Sense::Reversed), (b.edges[8], es[3], Sense::Reversed)];
        for (keep, drop, rel) in pairs {
            t.merge_edges(keep, drop, rel)?;
        }
        t.merge_faces(b.faces[0], disk, Sense::Reversed)?;
        Ok((disk, annulus))
    })
    .unwrap();
    assert_valid(&t);
    assert_eq!(t.state(a.faces[1]), Some(EntityState::Split(OpId(3))));
    assert_eq!(t.lineage().children(a.faces[1]).as_slice(), &[annulus, disk]);
    assert_eq!(t.loops(annulus).count(), 2, "the annulus keeps the outer loop and gains the hole");
    let am = t.face_moments(annulus).unwrap();
    assert!(close(am.area, 0.75, 1e-9), "annulus area {}", am.area);
    let shared = b.faces[0];
    let sides = t.face_sides(shared).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(a.volume), Region::Volume(b.volume)), "flipped so the lower volume is on the left");
    assert!(t.lineage().for_op(OpId(3)).iter().any(|r| r.entity == shared && r.kind == LineageKind::Modified(ModKind::Reversed)));
    assert!(close(t.face_moments(shared).unwrap().area, 0.25, 1e-9));
    assert_eq!(t.coincidence(shared, disk).map(|p| p.status), Some(Coincidence::Merged));
}

#[test]
fn make_inner_loop_and_kill_inner_loop_are_inverse_and_chord_split_honors_inner_loops() {
    let mut t = CellComplex::new();
    let a = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let top = a.faces[5]; // z = 1, corners 4..8
    let before = t.boundary(top).to_vec();
    // A small square footprint in the upper-left triangle (y > x of the top face).
    let pts = [(0.1, 0.5), (0.3, 0.5), (0.3, 0.7), (0.1, 0.7)];
    let es = with_op(&mut t, OpId(2), |t| {
        let vs: Vec<EntityId> = pts.iter().map(|&(x, y)| t.make_vertex(Point3::new(x, y, 1.0))).collect::<Result<_, _>>().map_err(fixtures::FixtureError::from)?;
        let mut es = Vec::new();
        for i in 0..4 {
            let (p, q) = (pts[i], pts[(i + 1) % 4]);
            let d = Vector3::new(q.0 - p.0, q.1 - p.1, 0.0);
            let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(p.0, p.1, 1.0), d).unwrap())).unwrap();
            es.push(t.make_edge(vs[i], vs[(i + 1) % 4], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?);
        }
        // MEKR then KEMR: the inner loop appears as ring 1 and removing it restores the row.
        let hole: Vec<CoedgeSpec> = es.iter().rev().map(|&edge| CoedgeSpec { edge, sense: Sense::Reversed }).collect();
        assert_eq!(t.make_inner_loop(top, &hole)?, 1);
        assert_eq!(t.loops(top).count(), 2);
        assert!(t.kill_inner_loop(top, 0).is_err(), "the outer loop cannot be removed");
        t.kill_inner_loop(top, 1)?;
        assert_eq!(t.boundary(top), &before[..]);
        assert!(t.kill_inner_loop(top, 1).is_err());
        Ok(es)
    })
    .unwrap();
    // Add the hole again together with the disk that fills it (same op, so the shell closes).
    with_op(&mut t, OpId(3), |t| {
        let hole: Vec<CoedgeSpec> = es.iter().rev().map(|&edge| CoedgeSpec { edge, sense: Sense::Reversed }).collect();
        t.make_inner_loop(top, &hole)?;
        let disk_loop: Vec<CoedgeSpec> = es.iter().map(|&edge| CoedgeSpec { edge, sense: Sense::Forward }).collect();
        let Carrier::Surface(sc) = t.carrier(top).clone() else { unreachable!() };
        let disk = t.make_face(&[&disk_loop], Some(sc))?;
        t.add_face_use(a.volume, 0, FaceUseSpec { face: disk, side: Side::Left })?;
        Ok(())
    })
    .unwrap();
    assert_valid(&t);
    assert!(close(t.face_moments(top).unwrap().area, 0.96, 1e-9), "unit square minus a 0.2 x 0.2 hole");
    // Split the face by the corner-to-corner chord: the hole goes to the child that contains it,
    // found by a point-in-face test.
    let [c0, c1] = with_op(&mut t, OpId(4), |t| {
        let line = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())).unwrap();
        let chord = t.make_edge(a.vertices[4], a.vertices[7], Some(CurveCarrier { curve: line, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward }))?;
        Ok(t.split_face(top, FaceCut::Chord { edge: chord }, InnerLoopAssign::ByPointTest)?)
    })
    .unwrap();
    assert_valid(&t);
    let (m0, m1) = (t.face_moments(c0).unwrap(), t.face_moments(c1).unwrap());
    let holes = [t.loops(c0).count(), t.loops(c1).count()];
    assert_eq!(holes.iter().sum::<usize>(), 3, "exactly one child got the inner loop");
    let (holed, plain) = if holes[0] == 2 { (m0, m1) } else { (m1, m0) };
    assert!(close(holed.area, 0.5 - 0.04, 1e-9), "{}", holed.area);
    assert!(close(plain.area, 0.5, 1e-9));
}

// ---------------------------------------------------------------- webcut and lineage

#[test]
fn webcut_splits_volume_and_faces_with_lineage_and_state() {
    let mut t = CellComplex::new();
    let b = unit_brick(&mut t, 1, [0.0, 0.0, 0.0], [1.0, 2.0, 4.0]);
    let v0 = t.volume_moments(b.volume).unwrap();
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.25)).unwrap();
    assert_valid(&t);
    // Volume 1 -> 1a, 1b with Split lineage; the retired id keeps resolving.
    assert_eq!(t.state(b.volume), Some(EntityState::Split(OpId(2))));
    assert!(!t.is_alive(b.volume));
    assert_eq!(t.lineage().children(b.volume).as_slice(), &w.volumes);
    assert_eq!(t.lineage().parents(w.volumes[0]).as_slice(), &[b.volume]);
    assert_eq!(t.live_descendants(b.volume), w.volumes.to_vec());
    assert!(w.volumes[0] < w.volumes[1]);
    // Face 3 (+y) -> 3a, 3b, and the three other side faces likewise.
    for (i, face) in [b.faces[0], b.faces[1], b.faces[2], b.faces[3]].into_iter().enumerate() {
        assert_eq!(t.state(face), Some(EntityState::Split(OpId(2))));
        assert_eq!(t.lineage().children(face).as_slice(), &w.split_faces[i]);
        assert_eq!(t.live_descendants(face), w.split_faces[i].to_vec());
    }
    // The cut cap is an interface between the two parts, left = first child, and generated on the tool plane.
    let sides = t.face_sides(w.cap).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(w.volumes[0]), Region::Volume(w.volumes[1])));
    let out = t.lineage().cut_outcome(OpId(2));
    assert_eq!(out.caps, vec![CapFace { face: w.cap, from_surface: w.plane }]);
    assert!(out.lineage.contains(&Lineage { old: b.volume, new: w.volumes.iter().copied().collect() }));
    assert!(out.lineage.contains(&Lineage { old: b.faces[3], new: w.split_faces[3].iter().copied().collect() }));
    // Edge lineage too: each vertical edge became two.
    assert_eq!(t.lineage().children(b.edges[8]).as_slice(), &w.split_edges[0]);
    // The parts sum to the original and have the right sizes: the piece with the cap on its left is
    // the one containing the lowest face id (the -x face half below the cut).
    let (m0, m1) = (t.volume_moments(w.volumes[0]).unwrap(), t.volume_moments(w.volumes[1]).unwrap());
    let total = m0.add(&m1);
    assert!(close(total.volume, v0.volume, 1e-9) && close(total.first.z, v0.first.z, 1e-9));
    let vols = {
        let mut v = [m0.volume, m1.volume];
        v.sort_by(f64::total_cmp);
        v
    };
    assert!(close(vols[0], 2.0, 1e-9) && close(vols[1], 6.0, 1e-9), "{vols:?}");
    // Body keeps both pieces.
    assert_eq!(t.boundary(b.body).iter().map(|e| e.cell).collect::<Vec<_>>(), w.volumes.to_vec());
    // V = 8 + 4, E = 12 - 4 + 8 + 4, F = 6 - 4 + 8 + 1; two balls glued on the cap: V - E + F - C = 1.
    assert_eq!(counts(&t), [12, 20, 11, 2, 1]);
    assert_eq!(euler_characteristic(&t), 3);
}

// ---------------------------------------------------------------- periodic seam and poles

#[test]
fn cylinder_has_a_seam_listed_left_and_right_and_closed_edges() {
    let mut t = CellComplex::new();
    let (r, h) = (1.5, 2.0);
    let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, r, h)).unwrap();
    assert_eq!(counts(&t), [2, 3, 3, 1, 1]);
    assert_eq!(euler_characteristic(&t), 2);
    assert_valid(&t);
    let [e_bot, _e_top, e_seam] = c.edges;
    // Closed circles: one vertex, a single Both row.
    assert_eq!(t.edge_ends(e_bot), Some((c.vertices[0], c.vertices[0])));
    assert_eq!(t.boundary(e_bot).len(), 1);
    assert_eq!(t.boundary(e_bot)[0].side, Side::Both);
    // The seam edge is listed twice in the lateral loop: once Left, once Right.
    let lat = c.faces[2];
    let seam_uses: Vec<Side> = t.coedges(lat).filter(|ce| ce.key.edge == e_seam).map(|ce| ce.key.side).collect();
    assert_eq!(seam_uses, vec![Side::Left, Side::Right]);
    assert_eq!(t.seam_edges(lat).as_slice(), &[e_seam]);
    let ring = t.radial_uses(e_seam);
    assert_eq!(ring.len(), 2);
    assert_eq!(ring[0].face, ring[1].face);
    assert_eq!(ring.iter().map(|u| u.sense).collect::<Vec<_>>(), vec![Sense::Forward, Sense::Reversed]);
    // CoedgeKeys are unique and stable; the vertex at the seam appears twice in the loop (CornerKey).
    let keys: Vec<CoedgeKey> = t.coedges(lat).map(|ce| ce.key).collect();
    assert_eq!(keys.len(), 4);
    let mut sorted = keys.clone();
    sorted.sort();
    sorted.dedup();
    assert_eq!(sorted.len(), 4);
    let corners: Vec<CornerKey> = keys.iter().map(|k| CornerKey { face: lat, incoming: *k }).collect();
    assert_eq!(corners.len(), 4);
    let seam_vertex_uses = t.loop_vertices(lat, 0).filter(|v| *v == c.vertices[0]).count();
    assert_eq!(seam_vertex_uses, 2, "the seam vertex appears twice in the loop: bottom circle and seam start");
    // Pcurves at u = 0 and u = 2 pi.
    let pc_l = t.pcurve(CoedgeKey { face: lat, edge: e_seam, side: Side::Left }).unwrap();
    let pc_r = t.pcurve(CoedgeKey { face: lat, edge: e_seam, side: Side::Right }).unwrap();
    assert!(close(pc_l.eval(0.5).u, TAU, 1e-14) && close(pc_r.eval(0.5).u, 0.0, 1e-14));
    assert!(close(pc_l.eval(0.5).v, 0.5, 1e-14));
    // Moments by quadrature against analytic values.
    let lm = t.face_moments(lat).unwrap();
    assert!(close(lm.area, TAU * r * h, 1e-9), "lateral area {}", lm.area);
    assert!(close(lm.centroid().unwrap().z, h / 2.0, 1e-9));
    assert!(lm.vector_area.norm() < 1e-9, "closed lateral surface has zero vector area");
    assert!(lm.plane(1e-3).is_none(), "curved face is not planar");
    let cap = t.face_moments(c.faces[1]).unwrap();
    assert!(close(cap.area, PI * r * r, 1e-9) && close(cap.vector_area.z, PI * r * r, 1e-9));
    let bottom = t.face_moments(c.faces[0]).unwrap();
    assert!(close(bottom.vector_area.z, -PI * r * r, 1e-9), "bottom normal points down");
    let vm = t.volume_moments(c.volume).unwrap();
    assert!(close(vm.volume, PI * r * r * h, 1e-9), "volume {}", vm.volume);
    assert!(close(vm.centroid().unwrap().z, h / 2.0, 1e-9));
    assert!(close(vm.second[0][0], PI * r.powi(4) / 4.0 * h, 1e-8), "int x^2 dV = pi r^4 h / 4");
    assert!(close(vm.second[2][2], PI * r * r * h * h * h / 3.0, 1e-8));
    // Point classification on the periodic face: seam-straddling queries wrap.
    assert_eq!(t.classify_point_on_face(lat, Uv::new(1.0, 0.5)).unwrap(), PointClass::In);
    assert_eq!(t.classify_point_on_face(lat, Uv::new(1.0 + TAU, 0.5)).unwrap(), PointClass::In);
    assert_eq!(t.classify_point_on_face(lat, Uv::new(1.0, h + 0.5)).unwrap(), PointClass::Out);
    assert_eq!(t.classify_point_on_face(lat, Uv::new(2.0, h)).unwrap(), PointClass::On);
}

#[test]
fn sphere_has_pole_coedges_left_out_of_euler_counts_and_rings() {
    let mut t = CellComplex::new();
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, 2.0)).unwrap();
    assert_valid(&t);
    assert_eq!(euler_characteristic(&t), 2, "poles excluded: V=2, E=1, F=1");
    let kinds: Vec<CoedgeKind> = t.coedges(s.face).map(|c| c.kind).collect();
    assert_eq!(kinds, vec![CoedgeKind::Edge, CoedgeKind::Pole, CoedgeKind::Edge, CoedgeKind::Pole], "canonical start is the seam, then a pole, the seam again, the other pole");
    assert!(t.radial_uses(s.edges[0]).is_empty() && t.radial_uses(s.edges[2]).is_empty(), "pole edges have no ring");
    assert_eq!(t.radial_uses(s.edges[1]).len(), 2);
    // The pole vertex is reached by the pole coedge but skipped in outlines.
    assert_eq!(t.loop_vertices(s.face, 0).count(), 2, "pole coedges skipped");
    assert_eq!(t.face_outline(s.face).len(), 2);
    assert!(matches!(t.face_moments(s.face), Err(QueryError::Unsupported(_))), "faces with poles are documented as unsupported");
    assert!(t.edge_ends(s.edges[0]).is_some_and(|(a, b)| a == b));
    assert!(t.flags(s.edges[0]).contains(CellFlags::DEGENERATE));
    // A pole edge that is not closed or has a curve is rejected at end_op.
    let err = with_op(&mut t, OpId(2), |t| {
        let v0 = t.make_vertex(Point3::new(5.0, 0.0, 0.0)).map_err(fixtures::FixtureError::from)?;
        let v1 = t.make_vertex(Point3::new(6.0, 0.0, 0.0)).map_err(fixtures::FixtureError::from)?;
        let e = t.make_edge(v0, v1, None)?;
        t.set_flags(e, CellFlags::DEGENERATE)?;
        Ok(())
    });
    assert!(matches!(err, Err(fixtures::FixtureError::Topo(TopoError::OpInvalid(_)))), "{err:?}");
    assert_eq!(t.cells(Dim::Vertex).len(), 2, "the failed operation was rolled back");
}

// ---------------------------------------------------------------- baffle and non-manifold fin

#[test]
fn baffle_lives_in_an_internal_shell_and_is_a_face_with_the_same_volume_on_both_sides() {
    let mut t = CellComplex::new();
    let (b, f) = with_op(&mut t, OpId(1), fixtures::brick_with_baffle).unwrap();
    assert_valid(&t);
    assert_eq!(t.shell_kind(b.volume, 0), Some(ShellKind::Outer));
    assert_eq!(t.shell_kind(b.volume, 1), Some(ShellKind::Internal));
    assert_eq!(t.shell_kind(b.volume, 2), None);
    let sides = t.face_sides(f.face).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(b.volume), Region::Volume(b.volume)));
    assert_eq!(t.face_class(f.face), Some(FaceClass::Baffle));
    assert!(t.is_baffle(f.face));
    assert_eq!(t.face_use_in(f.face, b.volume).as_slice(), &[Sense::Forward, Sense::Reversed], "two entries = baffle");
    let uses = t.face_uses_of(f.face);
    assert_eq!(uses.len(), 2);
    assert!(uses.iter().all(|u| u.shell == 1));
    assert_eq!(t.volume_faces(b.volume).len(), 7, "the baffle appears once");
    // Baffles cancel in the volume moments.
    let m = t.volume_moments(b.volume).unwrap();
    assert!(close(m.volume, 1.0, 1e-9));
    assert!(close(t.face_moments(f.face).unwrap().area, 0.25, 1e-9));
    // A baffle in an Outer (or Void) shell is rejected everywhere.
    let mut t2 = CellComplex::new();
    t2.begin_op(OpId(1)).unwrap();
    let b2 = fixtures::brick(&mut t2, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).unwrap();
    let err = t2.add_face_use(b2.volume, 0, FaceUseSpec { face: b2.faces[0], side: Side::Both });
    assert!(matches!(err, Err(TopoError::BaffleInClosedShell)), "{err:?}");
    let both = [FaceUseSpec { face: b2.faces[2], side: Side::Both }];
    let r = t2.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &both }]);
    assert!(matches!(r, Err(TopoError::BaffleInClosedShell)), "{r:?}");
    let r = t2.add_shell(b2.volume, ShellSpec { kind: ShellKind::Void, uses: &both });
    assert!(matches!(r, Err(TopoError::BaffleInClosedShell)), "{r:?}");
    t2.abort_op();
}

#[test]
fn non_manifold_edge_has_an_angular_radial_ring() {
    let mut t = CellComplex::new();
    let (b, fin) = with_op(&mut t, OpId(1), fixtures::brick_with_fin).unwrap();
    assert_valid(&t);
    assert_eq!(t.face_class(fin.face), Some(FaceClass::Free));
    let e = b.edges[8];
    let ring = t.radial_uses(e);
    // Faces around the z-edge at the origin: -x (inward +y, 90 deg), -y (inward +x, 0 deg) and the fin
    // (inward (-1,-1), 225 deg). Counter-clockwise from the lowest face id (-x): fin at 135, -y at 270.
    let faces: Vec<EntityId> = ring.iter().map(|u| u.face).collect();
    assert_eq!(faces, vec![b.faces[0], fin.face, b.faces[2]]);
    assert_eq!(ring[0].sense, Sense::Forward);
    // Turning around the edge inside the brick goes from -x to -y (the wedge between 0 and 90 degrees).
    assert_eq!(t.face_across_edge(b.faces[0], e, b.volume), Some(b.faces[2]));
    assert_eq!(t.face_across_edge(b.faces[2], e, b.volume), Some(b.faces[0]));
    assert_eq!(t.face_across_edge(fin.face, e, b.volume), None, "the fin does not border the brick");
    // Euler count still sees a valid complex of a brick plus a sheet: V - E + F = 2 + 1.
    assert_eq!(euler_characteristic(&t), 8 + 2 - 12 - 3 + 6 + 1);
}

// ---------------------------------------------------------------- CSG volume

#[test]
fn csg_carrier_volume_with_general_quadric_validates() {
    let mut t = CellComplex::new();
    let v = with_op(&mut t, OpId(1), |t| {
        let sphere = t.shapes_mut().unwrap().add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 1.0).unwrap())).unwrap();
        let gq = t.shapes_mut().unwrap().add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [0.25, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5] }))).unwrap();
        let vol = t.make_volume(&[])?;
        t.set_carrier(vol, Carrier::Csg(CsgCell::new([HalfSpace { surface: sphere, sign: Sign::Negative }, HalfSpace { surface: gq, sign: Sign::Negative }])))?;
        Ok(vol)
    })
    .unwrap();
    assert_valid(&t);
    assert!(matches!(t.carrier(v), Carrier::Csg(c) if c.halfspaces.len() == 2));
    let Carrier::Csg(cell) = t.carrier(v) else { unreachable!() };
    assert_eq!(cell.contains(t.shapes(), Point3::new(0.0, 0.0, 0.0), 1e-9).unwrap(), sm_geom_api::Containment::Inside);
    // A half-space over a missing surface is flagged.
    let bad = with_op(&mut t, OpId(2), |t| {
        let ghost = EntityId::new(sm_core::EntityKind::Surface, 42).unwrap();
        let vol = t.make_volume(&[])?;
        t.set_carrier(vol, Carrier::Csg(CsgCell::new([HalfSpace { surface: ghost, sign: Sign::Negative }])))?;
        Ok(vol)
    })
    .unwrap();
    assert!(validate_geometry(&t).iter().any(|i| i.cell == bad && i.rule == Rule::MissingGeometry));
    // Wrong carrier kind for the rank is rejected.
    t.begin_op(OpId(3)).unwrap();
    assert!(matches!(t.set_carrier(v, Carrier::Point(Point3::default())), Err(TopoError::WrongDim { .. })));
    t.abort_op();
}

#[test]
fn large_magnitude_quadric_coefficients_do_not_collide_in_the_hash() {
    // Sphere centred at (4000,0,0): constant term ~ L^2 = 1.6e7; r=1 vs r=10 must differ.
    for tol in [ModelTolerance::DEFAULT, ModelTolerance::for_extent(1000.0)] {
        let hash_for = |c: f64| {
            let mut t = CellComplex::with_tolerance(tol);
            with_op(&mut t, OpId(1), |t| {
                let gq = t.shapes_mut().unwrap().add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, -8000.0, 0.0, 0.0, c] }))).unwrap();
                let vol = t.make_volume(&[])?;
                t.set_carrier(vol, Carrier::Csg(CsgCell::new([HalfSpace { surface: gq, sign: Sign::Negative }])))?;
                Ok(vol)
            })
            .unwrap();
            t.canonical_hash()
        };
        assert_ne!(hash_for(1.6e7 - 1.0), hash_for(1.6e7 - 100.0));
        assert_eq!(hash_for(1.6e7 - 1.0), hash_for(1.6e7 - 1.0));
    }
}

// ---------------------------------------------------------------- virtual dummy for later modules

// ---------------------------------------------------------------- atomicity, lifecycle, errors

#[test]
fn failed_edit_leaves_the_hash_unchanged_and_abort_restores_everything() {
    let mut t = CellComplex::new();
    let a = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let h0 = t.canonical_hash();
    // Failing operators inside an open operation change nothing.
    t.begin_op(OpId(2)).unwrap();
    let h_open = t.canonical_hash();
    assert_ne!(h_open, h0, "the revision is part of the hash");
    let ghost = EntityId::new(sm_core::EntityKind::TopoVertex, 999).unwrap();
    assert!(t.make_edge(a.vertices[0], ghost, None).is_err());
    assert!(t.merge_vertices(a.vertices[0], a.vertices[7], None).is_err(), "far apart");
    assert!(matches!(t.merge_vertices(a.vertices[7], a.vertices[0], None), Err(TopoError::KeepNotLowest { .. })));
    assert!(t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &[FaceUseSpec { face: a.faces[0], side: Side::Left }] }]).is_err(), "side occupied");
    assert!(t.split_face(a.faces[0], FaceCut::Chord { edge: a.edges[0] }, InnerLoopAssign::None).is_err());
    assert!(t.split_edge(a.edges[0], a.vertices[0], 0.5).is_err());
    assert!(t.raise_tolerance(a.vertices[0], 5.0).is_err());
    assert!(t.raise_tolerance(a.vertices[0], 1e-9).is_err(), "tolerances only grow");
    assert_eq!(t.canonical_hash(), h_open, "failed edits are atomic");
    // A partial multi-step edit rolled back by abort_op restores the exact prior hash.
    let m = t.make_vertex(Point3::new(0.5, 0.0, 0.0)).unwrap();
    let [e1, _e2] = t.split_edge(a.edges[0], m, 0.5).unwrap();
    assert!(t.is_alive(e1));
    assert_ne!(t.canonical_hash(), h_open);
    t.abort_op();
    assert_eq!(t.canonical_hash(), h0);
    assert_eq!(t.open_op(), None);
    assert_eq!(t.revision(), OpId(1));
    assert_eq!(t.cells(Dim::Vertex).len(), 8);
    assert!(t.radial_uses(a.edges[0]).len() == 2);
    // Lifecycle errors.
    assert!(matches!(t.make_vertex(Point3::default()), Err(TopoError::NoOpenOp)));
    t.begin_op(OpId(2)).unwrap();
    assert!(matches!(t.begin_op(OpId(3)), Err(TopoError::OpAlreadyOpen)));
    t.end_op().unwrap();
    assert!(matches!(t.begin_op(OpId(2)), Err(TopoError::OpNotIncreasing { .. })));
    assert!(matches!(t.end_op(), Err(TopoError::NoOpenOp)));
    // end_op validates the touched cells and rolls back an invalid operation.
    let h = t.canonical_hash();
    let r = with_op(&mut t, OpId(10), |t| {
        // A volume whose shell is not closed: five of the six faces of a new brick.
        let b2 = fixtures::brick(t, Point3::new(9.0, 0.0, 0.0), [1.0; 3])?;
        t.remove_face_use(b2.volume, b2.faces[5], Side::Left)?;
        Ok(())
    });
    assert!(matches!(r, Err(fixtures::FixtureError::Topo(TopoError::OpInvalid(ref issues))) if issues.iter().any(|i| i.rule == Rule::ShellOpen)), "{r:?}");
    assert_eq!(t.canonical_hash(), h);
    assert_eq!(t.revision(), OpId(2), "a rolled-back operation does not advance the revision");
    // The next operation may reuse the id of the rolled-back one.
    with_op(&mut t, OpId(10), |t| fixtures::brick(t, Point3::new(9.0, 0.0, 0.0), [1.0; 3])).unwrap();
}

#[test]
fn kill_cascade_removes_orphans_top_down_and_keeps_shared_cells() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    let killed = with_op(&mut t, OpId(3), |t| Ok(t.kill_cascade(a.volume)?)).unwrap();
    assert_eq!(killed[0], a.volume);
    // Faces ascending, then edges ascending, then vertices ascending.
    let dims: Vec<Dim> = killed.iter().map(|c| t.dim_of(*c).unwrap()).collect();
    assert!(dims.windows(2).all(|w| w[0] >= w[1]), "top-down order");
    for d in [Dim::Face, Dim::Edge, Dim::Vertex] {
        let ids: Vec<EntityId> = killed.iter().copied().filter(|c| t.dim_of(*c) == Some(d)).collect();
        assert!(ids.windows(2).all(|w| w[0] < w[1]));
    }
    // A's five unshared faces die; the shared face survives because B still uses it.
    assert_eq!(killed.iter().filter(|c| t.dim_of(**c) == Some(Dim::Face)).count(), 5);
    assert!(t.is_alive(a.faces[1]));
    assert_eq!(t.state(a.volume), Some(EntityState::Deleted(OpId(3))));
    assert_eq!(t.lineage().for_op(OpId(3)).iter().filter(|r| r.kind == LineageKind::Deleted).count(), killed.len());
    assert_eq!(t.face_class(a.faces[1]), Some(FaceClass::Boundary));
    assert_eq!(t.face_sides(a.faces[1]).unwrap().left, Region::Volume(b.volume), "the survivor was re-oriented");
    assert_valid(&t);
}


#[test]
fn record_split_is_allowed_only_for_a_parent_deleted_in_the_same_operation() {
    let mut t = CellComplex::new();
    let (b, fin) = with_op(&mut t, OpId(1), fixtures::brick_with_fin).unwrap();
    t.begin_op(OpId(2)).unwrap();
    // Still alive: refused. Unknown ids and mismatched ranks: refused.
    assert!(t.record_split(fin.face, &[b.faces[0]]).is_err());
    t.kill(fin.face).unwrap();
    assert!(t.record_split(fin.face, &[b.vertices[0]]).is_err(), "children must have the parent's rank");
    assert!(t.record_split(fin.face, &[]).is_err());
    t.record_split(fin.face, &[b.faces[0], b.faces[1]]).unwrap();
    assert_eq!(t.state(fin.face), Some(EntityState::Split(OpId(2))));
    assert!(t.record_split(fin.face, &[b.faces[0]]).is_err(), "already split");
    t.end_op().unwrap();
    assert_eq!(t.lineage().children(fin.face).as_slice(), &[b.faces[0], b.faces[1]]);
    // A parent retired in an earlier operation cannot be declared later.
    with_op(&mut t, OpId(3), |t| {
        assert!(t.record_split(fin.face, &[b.faces[2]]).is_err());
        Ok(())
    })
    .unwrap();
}

// ---------------------------------------------------------------- tolerance and gap recording

#[test]
fn merge_with_max_gap_records_the_discarded_gap_and_grows_the_tolerance() {
    let mut t = CellComplex::new();
    let gap = 5e-5;
    let a = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let b = unit_brick(&mut t, 2, [1.0 + gap, 0.0, 0.0], [1.0; 3]);
    // Without an op-level gap the global tolerance (1e-6) refuses the merge.
    t.begin_op(OpId(3)).unwrap();
    assert!(matches!(t.merge_vertices(a.vertices[1], b.vertices[0], None), Err(TopoError::NotCoincident(_))));
    assert!(matches!(t.merge_vertices(a.vertices[1], b.vertices[0], Some(0.5)), Err(TopoError::ToleranceTooLarge { .. })), "above linear_max");
    t.abort_op();
    with_op(&mut t, OpId(3), |t| fixtures::merge_touching_bricks_gap(t, &a, &b, Some(1e-4))).unwrap();
    assert_valid(&t);
    // The discarded gap is in the lineage of the absorbed vertex ...
    let rec = t.lineage().of(b.vertices[0]).next().unwrap();
    let LineageKind::Merged { into, gap: g } = &rec.kind else { panic!("{rec:?}") };
    assert_eq!(*into, a.vertices[1]);
    assert!(close(*g, gap, 1e-12), "{g}");
    // ... the survivor's tolerance grew by the gap plus the absorbed tolerance, inside the cap ...
    let tol = t.tolerance_override(a.vertices[1]).unwrap();
    assert!(close(tol, gap + 1e-6, 1e-12) && tol <= t.tolerance().linear_max, "{tol}");
    assert!(t.lineage().of(a.vertices[1]).any(|r| r.kind == LineageKind::Modified(ModKind::Tolerance)));
    // ... edges and faces inherited the op-level tolerance, and the face merge recorded its measured gap.
    let face_rec = t.lineage().of(b.faces[0]).next().unwrap();
    assert!(matches!(face_rec.kind, LineageKind::Merged { into, gap: g } if into == a.faces[1] && close(g, gap, 1e-9)), "{face_rec:?}");
    assert!(t.tolerance_override(a.faces[1]).is_none(), "face tolerance is max(own, other) = global here");
    assert_eq!(t.coincidence(a.faces[1], b.faces[0]).unwrap().status, Coincidence::Merged);
}

// ---------------------------------------------------------------- determinism and hash quantization

fn build_model() -> (CellComplex, Vec<EntityId>) {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    with_op(&mut t, OpId(2), |t| fixtures::merge_touching_bricks(t, &a, &b)).unwrap();
    let w = with_op(&mut t, OpId(3), |t| fixtures::webcut_brick(t, &a, 0.5)).unwrap();
    let c = with_op(&mut t, OpId(4), |t| fixtures::cylinder(t, 1.0, 3.0)).unwrap();
    let mut ids: Vec<EntityId> = w.volumes.to_vec();
    ids.extend(c.faces);
    ids.push(w.cap);
    (t, ids)
}

#[test]
fn building_twice_and_across_threads_gives_identical_ids_and_hashes() {
    let (t1, ids1) = build_model();
    let (t2, ids2) = build_model();
    assert_eq!((ids1.clone(), t1.canonical_hash()), (ids2, t2.canonical_hash()));
    assert_eq!(t1.lineage(), t2.lineage());
    let results: Vec<_> = std::thread::scope(|s| {
        let hs: Vec<_> = (0..4).map(|_| s.spawn(|| {
            let (t, ids) = build_model();
            (ids, t.canonical_hash(), t.lineage().len(), Dim::ALL.map(|d| t.cells(d).to_vec()))
        })).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let reference = (ids1, t1.canonical_hash(), t1.lineage().len(), Dim::ALL.map(|d| t1.cells(d).to_vec()));
    for r in results {
        assert_eq!(r, reference, "identical ids, lineage and hash on every thread");
    }
}

#[test]
fn hash_quantization_rule_is_documented_and_enforced() {
    let q = ModelTolerance::DEFAULT.quantum();
    assert_eq!(q, ModelTolerance::DEFAULT.linear * 2f64.powi(-10), "quantum = linear * 2^-10");
    let hash_at = |dx: f64| {
        let mut t = CellComplex::new();
        with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(dx, 0.0, 0.0), [1.0; 3])).unwrap();
        t.canonical_hash()
    };
    let base = hash_at(0.0);
    assert_eq!(hash_at(0.3 * q), base, "a perturbation well under half a step does not change the hash");
    assert_eq!(hash_at(-0.3 * q), base);
    assert_ne!(hash_at(5.0 * q), base, "several steps do");
    // Per-entity tolerance overrides are quantized with the same rule.
    let tol_hash = |tol: f64| {
        let mut t = CellComplex::new();
        let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
        with_op(&mut t, OpId(2), |t| Ok(t.raise_tolerance(b.vertices[0], tol)?)).unwrap();
        t.canonical_hash()
    };
    assert_eq!(tol_hash(1e-5), tol_hash(1e-5 + 0.2 * q));
    assert_ne!(tol_hash(1e-5), tol_hash(1e-5 + 4.0 * q));
    // Pcurves are hashed by their contents.
    let pc_hash = |v: f64| {
        let mut t = CellComplex::new();
        let c = with_op(&mut t, OpId(1), |t| fixtures::cylinder(t, 1.0, 1.0)).unwrap();
        let key = CoedgeKey { face: c.faces[2], edge: c.edges[2], side: Side::Left };
        with_op(&mut t, OpId(2), |t| {
            Ok(t.set_pcurve(key, Some(sm_geom_api::PCurve::Polyline2(vec![Uv::new(0.0, 0.0), Uv::new(v, 1.0)])))?)
        })
        .unwrap();
        t.canonical_hash()
    };
    assert_ne!(pc_hash(1.0), pc_hash(2.0));
    assert_eq!(pc_hash(1.0), pc_hash(1.0));
}

// ---------------------------------------------------------------- performance

#[test]
fn building_many_bricks_is_near_linear() {
    let n: usize = if cfg!(debug_assertions) { 1_000 } else { 10_000 };
    let start = std::time::Instant::now();
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    for i in 0..n {
        let x = (i % 100) as f64 * 2.0;
        let y = ((i / 100) % 100) as f64 * 2.0;
        fixtures::brick(&mut t, Point3::new(x, y, 0.0), [1.0; 3]).unwrap();
    }
    t.end_op().unwrap();
    let dt = start.elapsed();
    assert_eq!(t.cells(Dim::Volume).len(), n);
    assert_eq!(t.cells(Dim::Face).len(), 6 * n);
    if !cfg!(debug_assertions) {
        assert!(dt.as_secs_f64() < 2.0, "{n} bricks took {dt:?}");
    }
    // Queries stay O(1)/O(log n): the last volume's faces and a radial ring.
    let last = *t.cells(Dim::Volume).last().unwrap();
    assert_eq!(t.volume_faces(last).len(), 6);
}

// ---------------------------------------------------------------- object safety

#[test]
fn view_and_edit_are_object_safe() {
    let mut t = CellComplex::new();
    let e: &mut dyn TopologyEdit = &mut t;
    e.begin_op(OpId(1)).unwrap();
    let b = fixtures::brick(e, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).unwrap();
    e.end_op().unwrap();
    let v: &dyn TopologyView = &t;
    assert_eq!(v.cells(Dim::Face).len(), 6);
    assert_eq!(v.face_sides(b.faces[0]).unwrap().left, Region::Volume(b.volume));
    assert_valid(v);
}

#[cfg(feature = "serde")]
#[test]
fn public_data_types_are_serde() {
    fn is_serde<T: serde::Serialize + serde::de::DeserializeOwned>() {}
    is_serde::<Dim>();
    is_serde::<Side>();
    is_serde::<Incidence>();
    is_serde::<Carrier>();
    is_serde::<LineageRecord>();
    is_serde::<LineageKind>();
    is_serde::<Lineage>();
    is_serde::<CutOutcome>();
    is_serde::<FacePair>();
    is_serde::<EntityState>();
    is_serde::<CoedgeKey>();
    is_serde::<Moments>();
    is_serde::<FaceMoments>();
    is_serde::<TopoIssue>();
    is_serde::<EdgeUse>();
}

// ---------------------------------------------------------------- remaining contract clauses

#[test]
fn coincidence_records_move_to_split_children_and_status_changes_are_logged() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    assert!(t.lineage().of(a.faces[1]).any(|r| r.op == OpId(1) && r.kind == LineageKind::Modified(ModKind::Coincidence)), "status change logged under the op");
    assert_eq!(t.coincidence(a.faces[1], b.faces[0]).unwrap().op, OpId(1));
    // Split A's +x face along a diagonal (corners 1 -> 7).
    let [c0, c1] = with_op(&mut t, OpId(2), |t| {
        let (p, q) = (Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let d = q - p;
        let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(p, d).unwrap())).unwrap();
        let chord = t.make_edge(a.vertices[1], a.vertices[7], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?;
        Ok(t.split_face(a.faces[1], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
    })
    .unwrap();
    assert!(t.coincident_faces(a.faces[1]).is_empty(), "the retired face keeps no records");
    for c in [c0, c1] {
        let ps = t.coincident_faces(c);
        assert_eq!(ps.len(), 1);
        assert_eq!((ps[0].other(c), ps[0].status), (Some(b.faces[0]), Coincidence::CoincidentUnimprinted));
    }
    assert_eq!(t.coincident_faces(b.faces[0]).len(), 2);
    assert!(t.coincidences().windows(2).all(|w| (w[0].a, w[0].b) < (w[1].a, w[1].b)), "sorted by (min, max)");
    // Status change under a later op is recorded and overwrites the status.
    with_op(&mut t, OpId(3), |t| Ok(t.record_coincidence(c0, b.faces[0], Coincidence::Overlapping)?)).unwrap();
    assert_eq!(t.coincidence(c0, b.faces[0]).unwrap().status, Coincidence::Overlapping);
    assert!(t.lineage().for_op(OpId(3)).iter().any(|r| r.kind == LineageKind::Modified(ModKind::Coincidence)));
}

#[test]
fn merge_faces_checks_loop_correspondence_and_edge_merges_check_curves() {
    let mut t = CellComplex::new();
    let (a, b) = with_op(&mut t, OpId(1), fixtures::two_bricks).unwrap();
    let h = t.canonical_hash();
    // Edges were not merged first, so the loops differ.
    t.begin_op(OpId(2)).unwrap();
    assert!(matches!(t.merge_faces(a.faces[1], b.faces[0], Sense::Reversed), Err(TopoError::NotCoincident(_))));
    // Edges whose end vertices are not merged yet.
    assert!(matches!(t.merge_edges(a.edges[5], b.edges[4], Sense::Forward), Err(TopoError::NotCoincident(_))));
    let h_open = t.canonical_hash();
    t.abort_op();
    assert_eq!(t.canonical_hash(), h);
    assert_ne!(h_open, h);
    // Curves that are far apart are refused even with merged end vertices (here: the curve behind
    // the dropped edge is replaced by one that runs 1e-2 off in z, above linear_max).
    with_op(&mut t, OpId(2), |t| {
        for j in 0..2 {
            for k in 0..2 {
                t.merge_vertices(a.vertices[1 + 2 * j + 4 * k], b.vertices[2 * j + 4 * k], None)?;
            }
        }
        Ok(())
    })
    .unwrap();
    t.begin_op(OpId(3)).unwrap();
    let Carrier::Curve(cc) = t.carrier(b.edges[4]).clone() else { panic!() };
    t.replace_curve(cc.curve, CurveDef::Line(Line::new(Point3::new(1.0, 0.0, 0.01), Vector3::new(0.0, 1.0, 0.0)).unwrap())).unwrap();
    assert!(matches!(t.merge_edges(a.edges[5], b.edges[4], Sense::Forward), Err(TopoError::NotCoincident(_))));
    t.abort_op();
}

#[test]
fn geometry_edits_are_logged_as_modified_and_validate_geometry_sees_them() {
    let mut t = CellComplex::new();
    let b = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    assert_valid(&t);
    with_op(&mut t, OpId(2), |t| {
        t.set_vertex_point(b.vertices[0], Point3::new(0.1, 0.0, 0.0))?;
        Ok(())
    })
    .unwrap();
    assert!(t.lineage().for_op(OpId(2)).iter().any(|r| r.entity == b.vertices[0] && r.kind == LineageKind::Modified(ModKind::Point)));
    assert!(validate_geometry(&t).iter().any(|i| i.rule == Rule::GeometryMismatch), "the moved vertex no longer sits on its curves");
    // Move it back and re-store the same definitions (heal): logged on the handles.
    let Carrier::Curve(c8) = t.carrier(b.edges[8]).clone() else { panic!() };
    with_op(&mut t, OpId(3), |t| {
        t.set_vertex_point(b.vertices[0], Point3::new(0.0, 0.0, 0.0))?;
        t.replace_curve(c8.curve, CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap()))?;
        let Carrier::Surface(sc) = t.carrier(b.faces[5]).clone() else { panic!() };
        let def = t.shapes().surface_def(sc.surface).cloned().unwrap();
        t.replace_surface(sc.surface, def)?;
        t.set_carrier(b.faces[5], Carrier::Surface(sc))?;
        Ok(())
    })
    .unwrap();
    let ops3 = t.lineage().for_op(OpId(3));
    assert!(ops3.iter().any(|r| r.entity == c8.curve && r.kind == LineageKind::Modified(ModKind::Curve)));
    assert!(ops3.iter().any(|r| r.kind == LineageKind::Modified(ModKind::Surface)));
    assert!(ops3.iter().any(|r| r.entity == b.faces[5] && r.kind == LineageKind::Modified(ModKind::Carrier)));
    assert_valid(&t);
    // Tolerance growth is logged, grow-only and capped.
    with_op(&mut t, OpId(4), |t| Ok(t.raise_tolerance(b.edges[0], 2e-6)?)).unwrap();
    assert_eq!(t.tolerance_override(b.edges[0]), Some(2e-6));
    assert_eq!(t.tolerance_of(b.edges[0]), 2e-6);
    assert_eq!(t.tolerance_of(b.edges[1]), t.tolerance().linear);
    assert!(t.lineage().for_op(OpId(4)).iter().any(|r| r.kind == LineageKind::Modified(ModKind::Tolerance)));
}

#[test]
fn typed_ids_and_kinds() {
    let mut t = CellComplex::new();
    let b = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let v = VolumeId::new(b.volume).unwrap();
    assert_eq!(EntityId::from(v), b.volume);
    assert_eq!(VolumeId::try_from(b.volume), Ok(v));
    assert_eq!(VolumeId::try_from(b.faces[0]), Err(WrongKind(b.faces[0])));
    assert_eq!(FaceId::new(b.faces[0]).map(FaceId::get), Some(b.faces[0]));
    assert_eq!(t.volume_ids().collect::<Vec<_>>(), vec![v]);
    assert_eq!(t.face_ids().count(), 6);
    assert_eq!(std::mem::size_of::<VolumeId>(), std::mem::size_of::<EntityId>(), "zero-cost wrapper");
    // Virtual kinds map to ranks.
    for (kind, dim) in [(EntityKind::PartitionVertex, Dim::Vertex), (EntityKind::PartitionEdge, Dim::Edge), (EntityKind::PartitionFace, Dim::Face), (EntityKind::PartitionCell, Dim::Volume)] {
        assert_eq!(Dim::of_kind(kind), Some(dim));
        assert_eq!(Dim::of_real_kind(kind), None);
        assert_eq!(dim.virtual_kind(), Some(kind));
    }
    assert_eq!(Dim::of_kind(EntityKind::Surface), None);
    // The complex does not claim virtual ids.
    let virt = EntityId::new(EntityKind::PartitionFace, 0).unwrap();
    assert_eq!(t.dim_of(virt), None);
    assert_eq!(t.state(virt), None);
}

#[test]
fn wires_embed_in_a_volume_and_unembed() {
    let mut t = CellComplex::new();
    let b = unit_brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let wire = with_op(&mut t, OpId(2), |t| {
        let p = Point3::new(0.25, 0.25, 0.25);
        let q = Point3::new(0.75, 0.25, 0.25);
        let (v0, v1) = (t.make_vertex(p)?, t.make_vertex(q)?);
        let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(p, q - p).unwrap())).unwrap();
        let e = t.make_edge(v0, v1, Some(CurveCarrier { curve, range: Interval::new(0.0, 0.5), sense: Sense::Forward }))?;
        t.embed(b.volume, e)?;
        Ok(e)
    })
    .unwrap();
    assert_valid(&t);
    assert_eq!(t.wire_edges(b.volume).collect::<Vec<_>>(), vec![wire]);
    assert_eq!(t.volume_faces(b.volume).len(), 6, "wires are not faces");
    assert_eq!(t.coboundary(wire).iter().map(|e| (e.cell, e.ring)).collect::<Vec<_>>(), vec![(b.volume, Incidence::EMBEDDED)]);
    with_op(&mut t, OpId(3), |t| Ok(t.unembed(b.volume, wire)?)).unwrap();
    assert_eq!(t.wire_edges(b.volume).count(), 0);
    assert!(t.coboundary(wire).is_empty());
    assert_valid(&t);
    // A wire cannot be unembedded twice, and a body can hold wires too.
    t.begin_op(OpId(4)).unwrap();
    assert!(t.unembed(b.volume, wire).is_err());
    t.embed(b.body, wire).unwrap();
    t.end_op().unwrap();
    assert!(t.wire_edges(b.body).any(|e| e == wire));
}

// ---------------------------------------------------------------- review fixes

fn csg_model(coef_a: f64, radius: f64) -> CellComplex {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), |t| {
        let sphere = t.shapes_mut()?.add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), radius).unwrap()))?;
        let gq = t.shapes_mut()?.add_surface(SurfaceDef::Implicit(ImplicitDef::Quadric(Quadric { coeffs: [coef_a, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5] })))?;
        let vol = t.make_volume(&[])?;
        t.set_carrier(vol, Carrier::Csg(CsgCell::new([HalfSpace { surface: sphere, sign: Sign::Negative }, HalfSpace { surface: gq, sign: Sign::Negative }])))?;
        Ok(())
    })
    .unwrap();
    t
}

#[test]
fn canonical_hash_covers_geometry_definitions() {
    let base = csg_model(0.25, 1.0).canonical_hash();
    assert_eq!(csg_model(0.25, 1.0).canonical_hash(), base);
    assert_ne!(csg_model(4.0, 1.0).canonical_hash(), base, "quadric coefficient");
    assert_ne!(csg_model(0.25, 7.0).canonical_hash(), base, "sphere radius");
    // replace_surface on an existing handle changes the hash, abort restores it.
    let mut t = csg_model(0.25, 1.0);
    let sid = EntityId::new(EntityKind::Surface, 0).unwrap();
    t.begin_op(OpId(2)).unwrap();
    t.replace_surface(sid, SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 3.0).unwrap())).unwrap();
    assert_ne!(t.canonical_hash(), base);
    t.abort_op();
    assert_eq!(t.canonical_hash(), base);
}

#[test]
fn shapes_mut_needs_an_open_op_and_is_append_only() {
    let mut t = CellComplex::new();
    assert!(matches!(t.shapes_mut().map(|_| ()), Err(TopoError::NoOpenOp)));
    t.begin_op(OpId(1)).unwrap();
    t.shapes_mut().unwrap().add_surface(SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 1.0).unwrap())).unwrap();
    t.abort_op();
    assert_eq!(t.shapes().surface_count(), 0, "additions roll back with the op");
}

#[test]
fn set_carrier_checks_handle_kinds() {
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    let wrong = EntityId::new(EntityKind::Curve, 0).unwrap();
    let vol = t.make_volume(&[]).unwrap();
    let cell = Carrier::Csg(CsgCell::new([HalfSpace { surface: wrong, sign: Sign::Negative }]));
    assert!(matches!(t.set_carrier(vol, cell), Err(TopoError::InvalidParameter(_))));
    t.abort_op();
}

#[test]
fn kill_then_record_split_yields_one_lineage_row() {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), |t| {
        let a = t.make_vertex(Point3::new(0.0, 0.0, 0.0))?;
        let b = t.make_vertex(Point3::new(1.0, 0.0, 0.0))?;
        t.kill(a)?;
        t.record_split(a, &[b])?;
        Ok(())
    })
    .unwrap();
    let out = t.lineage().cut_outcome(OpId(1));
    assert_eq!(out.lineage.len(), 1, "{:?}", out.lineage);
    assert_eq!(out.lineage[0].new.len(), 1);
}
