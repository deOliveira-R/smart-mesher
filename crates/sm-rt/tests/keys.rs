//! Cache keys of mesh inputs are pinned (W3b-1 backlog: the mesh part of `LeafInput::content_hash`
//! duplicates the builder's `mesh_topology_hash`).
//!
//! The two encodings differ today (the leaf hash carries a variant tag, length-prefixes the
//! complement name and mixes the boundary conditions; the builder keeps the boundary hash in its
//! own key field), so one implementation cannot serve both without moving a key. Sharing needs a
//! contract helper that reproduces the builder's bytes (see the `api_change_request` of the
//! W3b-RT-DELTA report). Until then this test pins both, so any change to either is deliberate.
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, LeafInput, MeshInput, RegionRule, TrackerBuilder};

fn key_of(rule: &RegionRule, comp: &ComplementPolicy, n: u32) -> (u128, u128, u128) {
    let mesh = hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let bnd = BoundarySpec::default();
    let mi = MeshInput::new(&mesh, rule, &bnd, comp);
    let k = MeshTrackerBuilder.cache_key(&BuildInput::Mesh(mi), &BuildOptions::default()).unwrap();
    (k.topology_hash.0, k.boundary_hash.0, LeafInput::Mesh(mi).content_hash().0)
}

#[test]
fn mesh_cache_keys_and_leaf_hashes_are_pinned() {
    let by_vol = RegionRule::ByVolumeClassification;
    let by_sets = RegionRule::ByCellSets(vec!["a".into(), "b".into()]);
    let outside = ComplementPolicy::Outside;
    let region = ComplementPolicy::Region("graveyard".into());
    let cases = [
        key_of(&by_vol, &outside, 3),
        key_of(&by_sets, &outside, 3),
        key_of(&by_vol, &region, 3),
        key_of(&by_vol, &outside, 4),
    ];
    // The builder's topology key and the leaf hash are different functions (no shared bytes).
    for c in &cases {
        assert_ne!(c.0, c.2);
    }
    // Every input maps to its own key.
    for i in 0..cases.len() {
        for j in i + 1..cases.len() {
            assert_ne!(cases[i].0, cases[j].0);
            assert_ne!(cases[i].2, cases[j].2);
        }
    }
    let got: Vec<(u128, u128)> = cases.iter().map(|c| (c.0, c.2)).collect();
    assert_eq!(got, PINNED, "a cache key or leaf hash moved: {got:?}");
}

/// Recorded on the W3b-1 base (`77c47bb`): `(topology_hash, LeafInput::content_hash)`.
const PINNED: [(u128, u128); 4] = [
    (87396184293874216909798514515518247686, 231175839765998579380175238219405614736),
    (4849837397077667703481901500705790564, 3514921992750775242136945237691315111),
    (119658131966119604772337235026513648857, 292543862311484519816029752876465863896),
    (159402426575924001913847422593060348712, 38416219900610296450122534824296371693),
];
