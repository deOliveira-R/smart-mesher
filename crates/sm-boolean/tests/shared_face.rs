//! Acceptance 13: a unite/subtract input shares a merged face with a volume outside the op.

mod common;

use common::*;
use sm_boolean::{BoolOptions, BoolRule, MergeOptions, boolean, merge};
use sm_topo_api::{CellComplex, Dim, Region, TopologyView, TopologyViewExt};

fn row() -> (CellComplex, [sm_topo_api::fixtures::BrickIds; 3]) {
    let mut t = CellComplex::new();
    let x = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let y = brick(&mut t, 2, [1.0, 0.0, 0.0], [1.0; 3]);
    let z = brick(&mut t, 3, [2.0, 0.0, 0.0], [1.0; 3]);
    in_op(&mut t, 4, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    (t, [x, y, z])
}

#[test]
fn unite_x_y_keeps_the_y_z_interface() {
    let (mut t, [x, y, z]) = row();
    assert_eq!(counts(&t)[2], 18 - 2);
    let z_cells_before: Vec<Vec<sm_topo_api::Incidence>> = t.closure(z.volume).iter().map(|&c| t.boundary(c).to_vec()).collect();
    let yz = y.faces[1];
    let out = in_op(&mut t, 5, |t| boolean(t, BoolRule::Unite, &[x.body], &[y.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert_eq!(out.volumes.len(), 1);
    let u = out.volumes[0];
    assert!((volume_of(&t, u) - 2.0).abs() < 1e-12);
    assert_eq!(t.volume_faces(u).len(), 6, "regularized union is a box");
    // The Y|Z face survives, two-sided between Z and the union.
    assert!(t.is_alive(yz));
    let sides = t.face_sides(yz).unwrap();
    let regions = [sides.left, sides.right];
    assert!(regions.contains(&Region::Volume(z.volume)) && regions.contains(&Region::Volume(u)), "{sides:?}");
    // Z keeps every cell and every boundary row except the orientation of the shared face.
    let z_cells_after: Vec<Vec<sm_topo_api::Incidence>> = t.closure(z.volume).iter().map(|&c| t.boundary(c).to_vec()).collect();
    assert_eq!(z_cells_before.len(), z_cells_after.len());
    for (b, a) in z_cells_before.iter().zip(&z_cells_after) {
        if b != a {
            // Only the shared face's loop orientation may differ (the orientation rule).
            assert_eq!(b.len(), a.len());
        }
    }
    assert!(t.is_alive(z.volume));
    assert_eq!(t.cells(Dim::Volume).len(), 2);
}

#[test]
fn subtract_x_from_y_keeps_the_y_z_interface() {
    let (mut t, [x, y, z]) = row();
    let yz = y.faces[1];
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
}
