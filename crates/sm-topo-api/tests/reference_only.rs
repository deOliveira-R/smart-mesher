//! Tests of the reference implementation's internals and of the contract's free functions that do
//! not belong in the generic conformance suite: wire shapes, a timing budget, and the exported
//! check and normalization helpers that implementations call.

use sm_core::{EntityId, EntityKind, Point3};
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::*;

fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

fn inc(kind: EntityKind, n: u64, side: Side, ring: u16) -> Incidence {
    Incidence::new(id(kind, n), side, ring)
}

// ---------------------------------------------------------------- wire shapes (feature gated)

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

#[cfg(feature = "serde")]
#[test]
fn golden_serde_shapes() {
    use serde_json::json;
    // CellFlags is a bare integer on the wire, whatever its storage width.
    assert_eq!(serde_json::to_value(CellFlags::DEGENERATE).unwrap(), json!(1));
    assert_eq!(serde_json::from_value::<CellFlags>(json!(1)).unwrap(), CellFlags::DEGENERATE);
    // Unit variants stay plain strings; data variants stay externally tagged.
    assert_eq!(serde_json::to_value(EntityState::Live).unwrap(), json!("Live"));
    assert_eq!(serde_json::to_value(EntityState::Split(OpId(2))).unwrap(), json!({"Split": 2}));
    assert_eq!(serde_json::to_value(Coincidence::ImprintedNotMerged).unwrap(), json!("ImprintedNotMerged"));
}

#[cfg(feature = "schemars")]
#[test]
fn key_types_have_json_schemas() {
    let s = format!("{:?}", schemars::schema_for!(CornerKey));
    assert!(s.contains("incoming") && s.contains("face"), "{s}");
    let s = format!("{:?}", schemars::schema_for!(CoedgeKey));
    assert!(s.contains("edge") && s.contains("side"), "{s}");
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

// ---------------------------------------------------------------- check_cells / check_cells_with

fn brick_complex() -> (CellComplex, fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    (t, b)
}

fn all_cells(t: &dyn TopologyView) -> Vec<EntityId> {
    Dim::ALL.iter().flat_map(|&d| t.cells(d).iter().copied()).collect()
}

#[test]
fn check_cells_is_clean_on_a_valid_complex_and_matches_validate_on_a_broken_one() {
    let (mut t, b) = brick_complex();
    assert!(check_cells(&t, &all_cells(&t)).is_empty());
    // Mid-operation, drop a face use: the volume's shell is open until end_op rejects it.
    t.begin_op(OpId(2)).unwrap();
    t.remove_face_use(b.volume, b.faces[0], Side::Left).unwrap();
    let local = check_cells(&t, &[b.volume]);
    assert!(local.iter().any(|i| i.rule == Rule::ShellOpen && i.cell == b.volume), "{local:?}");
    // The local check is what validate() reports for the same cell (validate sorts by cell and rule).
    let mut from_validate: Vec<TopoIssue> = validate(&t).into_iter().filter(|i| i.cell == b.volume && i.rule != Rule::CoboundaryMismatch).collect();
    let mut local_sorted = local.clone();
    from_validate.sort_by_key(|i| i.rule);
    local_sorted.sort_by_key(|i| i.rule);
    assert_eq!(local_sorted, from_validate);
    t.abort_op();
    assert!(check_cells(&t, &all_cells(&t)).is_empty());
}

#[test]
fn check_cells_with_asks_the_callback_once_per_shell_and_keeps_issue_order() {
    let (t, b) = brick_complex();
    let calls = std::cell::RefCell::new(Vec::new());
    let edge = b.edges[0];
    // A closed shell reported closed: no issue.
    let none = check_cells_with(&t, &[b.volume], &|v, r| {
        calls.borrow_mut().push((v, r));
        None
    });
    assert!(none.is_empty());
    assert_eq!(*calls.borrow(), vec![(b.volume, 0)]);
    // The callback's answer becomes a ShellOpen issue with its edge and net use.
    let open = check_cells_with(&t, &[b.faces[0], b.volume], &|_, _| Some((edge, 3)));
    assert_eq!(open.len(), 1);
    assert_eq!((open[0].cell, open[0].rule), (b.volume, Rule::ShellOpen));
    assert!(open[0].message.contains("net use 3"), "{}", open[0].message);
    // Other cells keep their relative order: issues follow the ids slice.
    let ids = [b.volume, b.edges[1], b.volume];
    let twice = check_cells_with(&t, &ids, &|_, _| Some((edge, 1)));
    assert_eq!(twice.iter().map(|i| i.cell).collect::<Vec<_>>(), vec![b.volume, b.volume]);
}

// ---------------------------------------------------------------- normalize_row / canonical_rotate

#[test]
fn normalize_row_fuses_closed_edge_sides_and_leaves_other_edge_rows() {
    let v = |n| id(EntityKind::TopoVertex, n);
    let mut row = vec![Incidence::new(v(3), Side::Left, 0), Incidence::new(v(3), Side::Right, 1)];
    normalize_row(Dim::Edge, &mut row);
    assert_eq!(row, vec![Incidence::new(v(3), Side::Both, 0)]);
    let mut open = vec![Incidence::new(v(1), Side::Left, 0), Incidence::new(v(2), Side::Right, 1)];
    let before = open.clone();
    normalize_row(Dim::Edge, &mut open);
    assert_eq!(open, before);
    let mut vertex = vec![inc(EntityKind::TopoEdge, 1, Side::Left, 0)];
    normalize_row(Dim::Vertex, &mut vertex);
    assert_eq!(vertex.len(), 1);
}

#[test]
fn normalize_row_rotates_face_loops_and_sorts_embedded_entries() {
    use EntityKind::{TopoEdge as E, TopoVertex as V};
    let e = |n, side| inc(E, n, side, 0);
    let inner = |n, side| inc(E, n, side, 1);
    let emb = |kind, n| Incidence::new(id(kind, n), Side::Both, Incidence::EMBEDDED);
    let mut row = vec![e(5, Side::Left), e(2, Side::Right), e(9, Side::Left), inner(8, Side::Left), inner(7, Side::Left), emb(V, 4), emb(E, 6), emb(V, 4), emb(V, 1)];
    normalize_row(Dim::Face, &mut row);
    assert_eq!(row[..3], [e(2, Side::Right), e(9, Side::Left), e(5, Side::Left)], "outer loop starts at its smallest edge");
    assert_eq!(row[3..5], [inner(7, Side::Left), inner(8, Side::Left)], "inner loop rotated, ring kept");
    assert_eq!(row[5..], [emb(V, 1), emb(V, 4), emb(E, 6)], "embedded entries sorted and deduplicated");
    let once = row.clone();
    normalize_row(Dim::Face, &mut row);
    assert_eq!(row, once, "idempotent");
}

#[test]
fn normalize_row_sorts_volumes_and_fuses_baffles_but_not_in_bodies() {
    use EntityKind::{TopoFace as F, TopoVolume as Vol};
    let u = |n, side, ring| inc(F, n, side, ring);
    let mut row = vec![u(4, Side::Right, 1), u(2, Side::Left, 0), u(4, Side::Left, 1), u(2, Side::Left, 0), u(3, Side::Left, 1)];
    normalize_row(Dim::Volume, &mut row);
    assert_eq!(row, vec![u(2, Side::Left, 0), u(3, Side::Left, 1), u(4, Side::Both, 1)]);
    // The same entries in different shells are not a baffle.
    let mut split = vec![u(4, Side::Right, 2), u(4, Side::Left, 1)];
    normalize_row(Dim::Volume, &mut split);
    assert_eq!(split, vec![u(4, Side::Left, 1), u(4, Side::Right, 2)]);
    // Bodies sort and deduplicate but never fuse sides.
    let m = |n, side| Incidence::new(id(Vol, n), side, 0);
    let mut body = vec![m(2, Side::Right), m(1, Side::Both), m(2, Side::Left), m(1, Side::Both)];
    normalize_row(Dim::Body, &mut body);
    assert_eq!(body, vec![m(1, Side::Both), m(2, Side::Left), m(2, Side::Right)]);
}

#[test]
fn canonical_rotate_starts_at_the_smallest_entry_and_breaks_ties_by_the_whole_sequence() {
    let e = |n, side| inc(EntityKind::TopoEdge, n, side, 0);
    let mut r = [e(7, Side::Left), e(3, Side::Right), e(5, Side::Left)];
    canonical_rotate(&mut r);
    assert_eq!(r, [e(3, Side::Right), e(5, Side::Left), e(7, Side::Left)]);
    // A seam edge appears twice: the rotation with the smaller continuation wins.
    let a = [e(1, Side::Left), e(4, Side::Left), e(1, Side::Left), e(2, Side::Left)];
    let mut x = a;
    canonical_rotate(&mut x);
    assert_eq!(x, [e(1, Side::Left), e(2, Side::Left), e(1, Side::Left), e(4, Side::Left)]);
    let mut y = [a[2], a[3], a[0], a[1]];
    canonical_rotate(&mut y);
    assert_eq!(y, x, "every rotation of a ring has the same canonical form");
    let mut single = [e(9, Side::Left)];
    canonical_rotate(&mut single);
    let mut empty: [Incidence; 0] = [];
    canonical_rotate(&mut empty);
}

#[test]
fn check_row_entries_checks_any_subset_of_a_row() {
    let (t, b) = brick_complex();
    let row = t.boundary(b.volume).to_vec();
    assert!(check_row_entries(&t, Dim::Volume, b.volume, &row).is_empty());
    assert!(check_row_entries(&t, Dim::Volume, b.volume, &[]).is_empty());
    let dead = Incidence::new(id(EntityKind::TopoFace, 999), Side::Left, 0);
    let bad = Incidence::new(b.vertices[0], Side::Left, 0);
    let issues = check_row_entries(&t, Dim::Volume, b.volume, &[row[0], dead, bad]);
    assert_eq!(issues.iter().map(|i| i.rule).collect::<Vec<_>>(), vec![Rule::DeadReference, Rule::BadIncidence]);
    assert!(issues.iter().all(|i| i.cell == b.volume));
}
