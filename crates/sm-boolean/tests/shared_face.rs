//! Acceptance 13: a unite/subtract input shares a merged face with a volume outside the op.

mod common;

use std::collections::BTreeMap;

use common::*;
use sm_boolean::{BoolOptions, BoolRule, MergeOptions, boolean, merge};
use sm_core::EntityId;
use sm_topo_api::{CellComplex, Coincidence, Dim, FacePair, Incidence, LineageKind, ModKind, OpId, Region, TopologyView, TopologyViewExt};

fn row() -> (CellComplex, [sm_topo_api::fixtures::BrickIds; 3]) {
    let mut t = CellComplex::new();
    let x = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let y = brick(&mut t, 2, [1.0, 0.0, 0.0], [1.0; 3]);
    let z = brick(&mut t, 3, [2.0, 0.0, 0.0], [1.0; 3]);
    in_op(&mut t, 4, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    (t, [x, y, z])
}

type Rows = BTreeMap<EntityId, Vec<Incidence>>;

/// Boundary row of every cell in the closure of `v` (the volume included).
fn rows_of(t: &CellComplex, v: EntityId) -> Rows {
    t.closure(v).iter().map(|&c| (c, t.boundary(c).to_vec())).collect()
}

fn sorted(mut v: Vec<(EntityId, u16, sm_topo_api::Side)>) -> Vec<(EntityId, u16, sm_topo_api::Side)> {
    v.sort();
    v
}

/// Z must be untouched by the op except for the Y|Z face use: only the `yz` row and Z's volume row
/// differ, both only in `side`. The orientation rule (lone volume left, two volumes `left < right`)
/// re-normalises the interface face because the new volume has a higher id than Z; decision 12
/// asks that the interface stays two-sided with its id and `Merged` record intact, which it does.
fn assert_only_the_interface_side_changed(t: &CellComplex, z: EntityId, yz: EntityId, before: &Rows, op: u64) {
    let after = rows_of(t, z);
    assert_eq!(before.keys().collect::<Vec<_>>(), after.keys().collect::<Vec<_>>(), "Z keeps every cell");
    let differ: Vec<EntityId> = before.keys().copied().filter(|c| before[c] != after[c]).collect();
    let mut expect = vec![yz, z];
    expect.sort();
    assert_eq!(differ, expect, "only the Y|Z face row and Z's volume row differ");
    // Z's volume row: same length, one entry differs, it names yz and only the side flipped.
    let (b, a) = (&before[&z], &after[&z]);
    assert_eq!(b.len(), a.len());
    let d: Vec<_> = b.iter().zip(a).filter(|(x, y)| x != y).collect();
    assert_eq!(d.len(), 1, "{d:?}");
    let (x, y) = d[0];
    assert_eq!((x.cell, x.ring), (yz, y.ring));
    assert_eq!(y.cell, yz);
    assert_eq!(y.side, x.side.flip());
    // The face row: same coedges (cell, ring), every loop side flipped.
    let flipped = sorted(before[&yz].iter().map(|e| (e.cell, e.ring, if e.ring == Incidence::EMBEDDED { e.side } else { e.side.flip() })).collect());
    assert_eq!(sorted(after[&yz].iter().map(|e| (e.cell, e.ring, e.side)).collect()), flipped, "Y|Z face: same coedges, sides flipped");
    // The reversal is logged on the face.
    let logged = t.lineage().for_op(OpId(op)).iter().any(|r| r.entity == yz && r.kind == LineageKind::Modified(ModKind::Reversed));
    assert!(logged, "Modified(Reversed) record on the Y|Z face");
}

fn merged_pairs(t: &CellComplex, f: EntityId) -> Vec<FacePair> {
    t.coincident_faces(f).iter().copied().filter(|p| p.status == Coincidence::Merged).collect()
}

#[test]
fn unite_x_y_keeps_the_y_z_interface() {
    let (mut t, [x, y, z]) = row();
    assert_eq!(counts(&t)[2], 18 - 2);
    let z_before = rows_of(&t, z.volume);
    let yz = y.faces[1];
    let pairs_before = merged_pairs(&t, yz);
    assert_eq!(pairs_before.len(), 1);
    let out = in_op(&mut t, 5, |t| boolean(t, BoolRule::Unite, &[x.body], &[y.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert_eq!(out.volumes.len(), 1);
    let u = out.volumes[0];
    assert!((volume_of(&t, u) - 2.0).abs() < 1e-12);
    assert_eq!(t.volume_faces(u).len(), 6, "regularized union is a box");
    // The union is V8 E12 F6; with Z (4 shared vertices, 4 shared edges, 1 shared face): 12/20/11.
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    // Union box 2x1x1 has area 10, Z adds 5 free faces.
    assert!((total_area(&t) - 15.0).abs() < 1e-12, "{}", total_area(&t));
    // The Y|Z face survives, two-sided between Z and the union.
    assert!(t.is_alive(yz));
    let sides = t.face_sides(yz).unwrap();
    let regions = [sides.left, sides.right];
    assert!(regions.contains(&Region::Volume(z.volume)) && regions.contains(&Region::Volume(u)), "{sides:?}");
    assert!(t.is_interface_face(yz));
    assert_eq!(merged_pairs(&t, yz), pairs_before, "the Merged coincidence record is kept");
    assert_only_the_interface_side_changed(&t, z.volume, yz, &z_before, 5);
    assert!(t.is_alive(z.volume));
    assert_eq!(t.cells(Dim::Volume).len(), 2);
}

#[test]
fn subtract_x_from_y_keeps_the_y_z_interface() {
    let (mut t, [x, y, z]) = row();
    let z_before = rows_of(&t, z.volume);
    let yz = y.faces[1];
    let pairs_before = merged_pairs(&t, yz);
    let out = in_op(&mut t, 5, |t| boolean(t, BoolRule::Subtract, &[y.body], &[x.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert_eq!(out.volumes.len(), 1);
    let r = out.volumes[0];
    assert!((volume_of(&t, r) - 1.0).abs() < 1e-12);
    assert!(t.is_alive(yz));
    let sides = t.face_sides(yz).unwrap();
    assert!([sides.left, sides.right].contains(&Region::Volume(z.volume)));
    assert!([sides.left, sides.right].contains(&Region::Volume(r)));
    assert_eq!(t.volume_faces(r).len(), 6);
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    assert_eq!(merged_pairs(&t, yz), pairs_before);
    assert_only_the_interface_side_changed(&t, z.volume, yz, &z_before, 5);
}
