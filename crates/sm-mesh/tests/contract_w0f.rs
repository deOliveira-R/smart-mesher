//! W0f contract changes: serials start at 1, `MeshSet` / `MeshInstance` kinds, side sets and
//! set labels, the quantized geometry hash, `#[non_exhaustive]` set members and node slots.

mod common;
use common::*;
use sm_core::{EntityKind, Point3};
use common::shim::{ReferenceBuilder, hex_block};
use sm_mesh_api::*;

/// Two unit hexes sharing the face `x = 1`, classified, not finished. Returns the builder and
/// the second cell's signed use of the shared face (provisional).
fn two_hex_builder() -> (ReferenceBuilder, SignedFace) {
    let mut b = ReferenceBuilder::new();
    let pts = |dx: f64| -> Vec<Point3> { [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]].iter().map(|p| Point3::new(p[0] + dx, p[1], p[2])).collect() };
    let mut vs = Vec::new();
    for (i, p) in pts(0.0).into_iter().chain(pts(1.0).into_iter().filter(|p| p.x > 1.5)).enumerate() {
        let v = b.add_vertex(p).unwrap();
        b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
        vs.push(v);
    }
    // Second hex: shared x = 1 corners 1, 2, 6, 5 and the four new ones 8..11.
    let h1 = [vs[0], vs[1], vs[2], vs[3], vs[4], vs[5], vs[6], vs[7]];
    let h2 = [vs[1], vs[8], vs[9], vs[2], vs[5], vs[10], vs[11], vs[6]];
    let a = b.add_cell_from_vertices(CellShape::Hex, &h1).unwrap();
    let c = b.add_cell_from_vertices(CellShape::Hex, &h2).unwrap();
    let mut shared = None;
    for added in [&a, &c] {
        b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
        for (k, sf) in added.faces.iter().enumerate() {
            if added.reused[k] {
                shared = Some(*sf);
            } else if !(added.cell == c.cell && c.reused.iter().zip(&c.faces).any(|(r, f)| *r && f.face() == sf.face())) {
                b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 1 + sf.face().get() as u64)).unwrap();
            }
        }
    }
    (b, shared.expect("the second hex reuses the shared face"))
}

#[test]
fn serials_start_at_one_and_sets_use_mesh_kinds() {
    let mut alloc = SerialAlloc::new();
    assert_eq!(alloc.next_serial(EntityKind::MeshCell), 1);
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    assert_eq!(m.mesh_id().serial(), 1);
    assert_eq!(m.id(MeshEntityKind::Cell, 0), id(EntityKind::MeshCell, 1));
    let s = MeshEdit::add_set(&mut m, "all", SetMembers::Cells(vec![CellIx(0)]), &mut alloc).unwrap();
    assert_eq!(s, id(EntityKind::MeshSet, 1));
    assert_valid(&m);
}

#[test]
fn side_sets_are_resided_by_finish_and_validated() {
    let (mut b, shared) = two_hex_builder();
    // Both sides of the shared face, plus the outside of one boundary face.
    let other = SignedFace::new(shared.face(), shared.side().flipped());
    let mut sides = vec![shared, other];
    sides.sort();
    b.add_set("both", SetMembers::Sides(sides)).unwrap();
    b.add_set("cell1_side", SetMembers::Sides(vec![shared])).unwrap();
    b.set_label("both", Some(7)).unwrap();
    b.set_label("cell1_side", Some(8)).unwrap();
    let m = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
    assert_valid(&m);
    let interior = interior_faces(&m);
    assert_eq!(interior.len(), 1);
    let f = interior[0];
    let s1 = m.set_by_name("cell1_side").unwrap();
    // The second cell is the higher id, so it sits on the face's right after finish.
    assert_eq!(m.face_right(f), Some(CellIx(1)));
    assert_eq!(s1.members, SetMembers::Sides(vec![SignedFace::new(f, Side::Right)]));
    assert_eq!(s1.label, Some(8));
    assert_eq!(s1.members.kind(), MeshEntityKind::Face);
    let both = m.set_by_name("both").unwrap();
    assert_eq!(both.members, SetMembers::Sides(vec![SignedFace::new(f, Side::Left), SignedFace::new(f, Side::Right)]));
    assert!(both.members.contains(f.0));
    assert!(!both.members.contains(f.0 + 1));
    // A side without a cell is rejected at finish.
    let (mut b, shared) = two_hex_builder();
    let lonely = (0..b.face_count()).map(FaceIx::new).find(|x| *x != shared.face()).unwrap();
    b.add_set("bad", SetMembers::Sides(vec![SignedFace::new(lonely, Side::Right)])).unwrap();
    assert!(b.finish(&mut SerialAlloc::new()).is_err());
    // Labels are unique per members kind.
    let (mut b, shared) = two_hex_builder();
    b.add_set("a", SetMembers::Sides(vec![shared])).unwrap();
    b.add_set("b", SetMembers::Faces(vec![shared.face()])).unwrap();
    b.add_set("c", SetMembers::Cells(vec![CellIx(0)])).unwrap();
    b.set_label("a", Some(1)).unwrap();
    b.set_label("b", Some(1)).unwrap();
    b.set_label("c", Some(1)).unwrap();
    assert!(b.set_label("nope", Some(1)).is_err());
    assert!(b.finish(&mut SerialAlloc::new()).is_err(), "a and b are both face sets with label 1");
}

#[test]
fn edit_side_sets_labels_and_hashes() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let f = interior_faces(&m)[0];
    let bnd = (0..m.count(MeshEntityKind::Face)).map(FaceIx::new).find(|x| m.face_right(*x).is_none()).unwrap();
    let h0 = topology_hash(&m);
    // Right side of a one-cell face: rejected, allocator untouched.
    let before: Vec<_> = alloc.state().collect();
    assert!(MeshEdit::add_set(&mut m, "x", SetMembers::Sides(vec![SignedFace::new(bnd, Side::Right)]), &mut alloc).is_err());
    assert_eq!(alloc.state().collect::<Vec<_>>(), before);
    // Unsorted: rejected.
    assert!(MeshEdit::add_set(&mut m, "x", SetMembers::Sides(vec![SignedFace::new(f, Side::Right), SignedFace::new(f, Side::Left)]), &mut alloc).is_err());
    MeshEdit::add_set(&mut m, "s", SetMembers::Sides(vec![SignedFace::new(bnd, Side::Left)]), &mut alloc).unwrap();
    let h1 = topology_hash(&m);
    assert_ne!(h0, h1);
    // A face set with the same face differs from the side set in the hash.
    let mut m2 = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    MeshEdit::add_set(&mut m2, "s", SetMembers::Faces(vec![bnd]), &mut SerialAlloc::new()).unwrap();
    assert_ne!(topology_hash(&m2), h1);
    // Labels enter the topology hash and must be unique per members kind.
    MeshEdit::set_label(&mut m, "s", Some(3)).unwrap();
    assert_ne!(topology_hash(&m), h1);
    MeshEdit::add_set(&mut m, "t", SetMembers::Faces(vec![f]), &mut alloc).unwrap();
    assert!(MeshEdit::set_label(&mut m, "t", Some(3)).is_err());
    MeshEdit::add_set(&mut m, "u", SetMembers::Cells(vec![CellIx(0)]), &mut alloc).unwrap();
    MeshEdit::set_label(&mut m, "u", Some(3)).unwrap();
    assert_valid(&m);
    MeshEdit::set_label(&mut m, "s", None).unwrap();
    assert_eq!(m.set_by_name("s").unwrap().label, None);
}

#[test]
fn quantized_geometry_hash_ignores_sub_quantum_noise() {
    let q = 1e-6;
    let quanta = HashQuanta::new(q, 1e-12);
    let m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let base = geometry_hash_quantized(&m, &quanta);
    let moved = |dx: f64| {
        let mut m2 = m.clone();
        let p = m2.coords();
        let p0 = Point3::new(p.x[0] + dx, p.y[0], p.z[0]);
        MeshEdit::set_point(&mut m2, PointIx(0), p0).unwrap();
        m2
    };
    // 0.3 of a step: same quantized hash, different raw hash.
    let small = moved(0.3 * q);
    assert_eq!(geometry_hash_quantized(&small, &quanta), base);
    assert_ne!(geometry_hash(&small), geometry_hash(&m));
    // 5 steps: different.
    assert_ne!(geometry_hash_quantized(&moved(5.0 * q), &quanta), base);
    // Both quanta are part of the hash.
    assert_ne!(geometry_hash_quantized(&m, &HashQuanta::new(2.0 * q, 1e-12)), base);
    assert_ne!(geometry_hash_quantized(&m, &HashQuanta::new(q, 2e-12)), base);
    // Rounding rule of the standalone helper: half away from zero, -0 folded, NaN marked,
    // saturation (the hash itself never uses a saturated code).
    assert_eq!(quantize_word(2.5, 1.0), 3);
    assert_eq!(quantize_word(-2.5, 1.0), -3);
    assert_eq!(quantize_word(-0.0, 1.0), 0);
    assert_eq!(quantize_word(-0.4, 1.0), 0);
    assert_eq!(quantize_word(f64::NAN, 1.0), i64::MIN);
    assert_eq!(quantize_word(1.0, 0.0), i64::MIN);
    assert_eq!(quantize_word(f64::INFINITY, 1.0), i64::MAX);
}

/// W0g r2 (`geometry-quantized.v2`): real tag values are quantized scale-free, so large values
/// never saturate; out-of-range coordinates hash by raw bits instead of a saturated code.
#[test]
fn quantized_geometry_hash_large_values_and_golden() {
    // Q2 quanta for linear = 1e-6, parametric = 1e-9.
    let quanta = HashQuanta::new(1e-6 / 1024.0, 1e-9 / 1024.0);
    let base = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let tagged = |v: f64| {
        let mut m = base.clone();
        MeshEdit::set_tag(&mut m, "temperature", MeshEntityKind::Cell, TagValues::F64(vec![v])).unwrap();
        m
    };
    let h = |v: f64| geometry_hash_quantized(&tagged(v), &quanta);
    // Under v1 (one length quantum, saturating codes) these two hashed equal.
    assert_ne!(h(1e12), h(2e12));
    assert_ne!(h(1e300), h(-1e300));
    assert_ne!(h(f64::INFINITY), h(f64::MAX));
    // Relative noise far below the 2^-40 step is invisible; signed zero folds; tiny real values do not.
    assert_eq!(h(1e12), h(1e12 * (1.0 + 1e-15)));
    assert_eq!(h(0.0), h(-0.0));
    assert_ne!(h(1e-20), h(0.0));
    assert_ne!(h(1e-24), h(1e-18));
    assert_eq!(h(5e-324), h(0.0));
    // Coordinates beyond the i64 code range: distinct, not saturated.
    let far = |x: f64| {
        let mut m = base.clone();
        MeshEdit::set_point(&mut m, PointIx(0), Point3::new(x, 0.0, 0.0)).unwrap();
        geometry_hash_quantized(&m, &quanta)
    };
    assert_ne!(far(1e20), far(2e20));
    // Golden: the v2 encoding is frozen.
    let got = (geometry_hash_quantized(&base, &quanta), h(1e12));
    assert_eq!(got, (GOLDEN_PLAIN, GOLDEN_TAGGED), "got ({:#018x}, {:#018x})", got.0, got.1);
}

const GOLDEN_PLAIN: u64 = 0x8848f9b9f10893bd;
const GOLDEN_TAGGED: u64 = 0x3b43766046757a10;

#[test]
fn non_exhaustive_enums_need_wildcards_downstream() {
    let m = SetMembers::Sides(vec![]);
    let n = match &m {
        SetMembers::Vertices(_) | SetMembers::Faces(_) | SetMembers::Cells(_) => 0,
        SetMembers::Sides(_) => 1,
        _ => 2,
    };
    assert_eq!(n, 1);
    assert_eq!(m.code(), 3);
    let slots = node_layout(CellShape::Hex, 2, Basis::LagrangeEquispaced).unwrap();
    assert!(slots.iter().all(|s| !matches!(s, NodeSlot::Corner(c) if *c > 7)));
}
