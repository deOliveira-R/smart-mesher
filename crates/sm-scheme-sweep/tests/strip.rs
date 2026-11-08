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

/// A brick with a through cylindrical hole (`instanced_core`): of the 21 face pairs of the holed
/// brick exactly one is a valid sweep, annulus to annulus. The side-to-side pairs look like
/// prisms but their linking faces have no column coedges.
#[test]
fn through_hole_brick_has_exactly_the_annulus_pair() {
    let fx = sm_testkit::topo::instanced_core(1).unwrap();
    let t = &fx.complex;
    let holed = *t
        .cells(Dim::Volume)
        .iter()
        .find(|&&v| faces_of(t, v).len() == 7)
        .expect("holed brick volume with 7 faces");
    let faces = faces_of(t, holed);
    let members = [(holed, faces.clone())];
    let n_edges = |f: EntityId| t.face_edges(f).len();
    let annuli: Vec<EntityId> = faces.iter().copied().filter(|&f| n_edges(f) == 5).collect();
    assert_eq!(annuli.len(), 2, "two annulus faces");
    let sides: Vec<EntityId> = faces.iter().copied().filter(|&f| n_edges(f) == 4).collect();
    assert_eq!(sides.len(), 4, "four side faces");

    let mut valid = Vec::new();
    let mut rejected = Vec::new();
    for (i, &s) in faces.iter().enumerate() {
        for &g in &faces[i + 1..] {
            match discover_columns(t, &members, s, g) {
                Ok(_) => valid.push((s, g)),
                Err(e) => rejected.push(((s, g), e.to_string())),
            }
        }
    }
    assert_eq!(valid.len() + rejected.len(), 21);
    assert_eq!(valid.len(), 1, "valid pairs: {valid:?}");
    let (a, b) = valid[0];
    assert!(annuli.contains(&a) && annuli.contains(&b) && a != b, "the valid pair is the two annuli");

    // Opposite side faces share no edge.
    let shares = |x: EntityId, y: EntityId| t.face_edges(x).iter().any(|e| t.face_edges(y).contains(e));
    let mut opposite = 0;
    for (i, &s) in sides.iter().enumerate() {
        for &g in &sides[i + 1..] {
            if shares(s, g) {
                continue;
            }
            opposite += 1;
            let (_, why) = rejected
                .iter()
                .find(|((x, y), _)| (*x, *y) == (s, g) || (*x, *y) == (g, s))
                .expect("side pair rejected");
            assert!(why.contains("0 column coedges"), "{why}");
        }
    }
    assert_eq!(opposite, 2, "two opposite side pairs");
}
