//! Strip check: every linking face of a sweep pair must be a strip (one loop, at least two
//! column coedges). Pairs that are not prisms are rejected at plan time.

use sm_core::{EntityId, Point3};
use sm_scheme_sweep::discover_columns;
use sm_testkit::topo::hollow_cylinder;
use sm_topo_api::fixtures::{brick, brick_with_void, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyView};

fn faces_of(t: &CellComplex, v: EntityId) -> Vec<EntityId> {
    let mut f: Vec<EntityId> = t
        .boundary(v)
        .iter()
        .filter(|i| t.dim_of(i.cell) == Some(Dim::Face))
        .map(|i| i.cell)
        .collect();
    f.sort_unstable();
    f.dedup();
    f
}

#[test]
fn plain_brick_pairs_still_pass() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| {
        brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0])
    })
    .unwrap();
    let members = [(b.volume, faces_of(&t, b.volume))];
    let ok = (0..6)
        .flat_map(|i| (i + 1..6).map(move |j| (i, j)))
        .filter(|&(i, j)| discover_columns(&t, &members, b.faces[i], b.faces[j]).is_ok())
        .count();
    assert_eq!(ok, 3, "three opposite pairs");
}

#[test]
fn cavity_brick_has_no_prism_pair() {
    let mut t = CellComplex::new();
    let vb = with_op(&mut t, OpId(1), |t| {
        brick_with_void(
            t,
            Point3::new(0.0, 0.0, 0.0),
            [10.0; 3],
            Point3::new(3.0, 3.0, 3.0),
            [4.0; 3],
        )
    })
    .unwrap();
    let members = [(vb.outer.volume, faces_of(&t, vb.outer.volume))];
    let all = &members[0].1;
    let mut reasons = Vec::new();
    for (i, &s) in all.iter().enumerate() {
        for &g in &all[i + 1..] {
            match discover_columns(&t, &members, s, g) {
                Ok(_) => panic!("pair ({s}, {g}) accepted"),
                Err(e) => reasons.push(e.to_string()),
            }
        }
    }
    assert!(
        reasons.iter().any(|r| r.contains("0 column coedges")),
        "{reasons:?}"
    );
}

#[test]
fn hollow_cylinder_annulus_pair_is_valid() {
    let fx = hollow_cylinder(1.0, 2.0, 2.0).unwrap();
    let t = &fx.topo.complex;
    let members = [(fx.volume, faces_of(t, fx.volume))];
    let c = discover_columns(t, &members, fx.source, fx.target).unwrap();
    assert_eq!(c.linking.len(), 2);
    assert_eq!(c.columns.len(), 2);
}
