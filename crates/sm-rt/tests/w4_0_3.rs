//! W4-0.3: `TrackerInfo::convex_cells` and `RegionRule::ByTaggedCellSets`.
use sm_core::Aabb;
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt::{MeshTrackerBuilder, read_smrt, write_smrt};
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, ContentHash, ContentHasher, MeshInput, RegionRule, RegionSet, Tracker, TrackerBuilder, TrackerInfo};
use sm_testkit::meshes::hex_block_two_volumes;

fn smrt_of(mesh: &dyn sm_mesh_api::MeshRead, rule: &RegionRule) -> (Vec<u8>, Box<dyn Tracker>) {
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(mesh, rule, &bnd, &comp));
    let (t, _) = MeshTrackerBuilder.build(&input, &BuildOptions::default()).unwrap();
    let mut bytes = Vec::new();
    write_smrt(&mut bytes, &t.export_cache().unwrap()).unwrap();
    (bytes, t)
}

#[test]
fn convex_cells_default_zero_and_counted_on_a_hex_brick() {
    assert_eq!(TrackerInfo::new(Aabb::empty(), 3, ContentHash(0)).convex_cells, 0);
    let mesh = hex_block([2; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let (_, t) = smrt_of(&mesh, &RegionRule::ByVolumeClassification);
    assert_eq!(t.info().cells, 8);
    assert_eq!(t.info().convex_cells, t.info().cells);
}

#[test]
fn tagged_cell_sets_name_tag_and_round_trip() {
    let fx = hex_block_two_volumes([2, 2, 2], [1.0; 3], 1).unwrap();
    let rule = RegionRule::ByTaggedCellSets(vec![
        RegionSet::new("volume_1", "fuel").with_tag("group", 7).with_tag("mat", -2),
        RegionSet::new("volume_2", "water").with_tag("group", 8),
    ]);
    let (bytes, t) = smrt_of(&fx.mesh, &rule);
    let check = |t: &dyn Tracker| {
        let r = &t.regions().0;
        assert_eq!(r.len(), 2);
        assert_eq!((r[0].name.as_str(), r[1].name.as_str()), ("fuel", "water"));
        assert_eq!(r[0].tags, vec![("group".to_owned(), 7), ("mat".to_owned(), -2)]);
        assert_eq!(r[1].tags, vec![("group".to_owned(), 8)]);
    };
    check(&*t);
    let img = read_smrt(&bytes).unwrap();
    let bare = sm_rt::load_image(&img).unwrap();
    check(&bare);
    assert_eq!(bare.structure_hash(), t.structure_hash());
    // The rule is part of the cache key: a plain rule over the same sets keys differently.
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let plain = RegionRule::ByCellSets(vec!["volume_1".into(), "volume_2".into()]);
    let key = |r: &RegionRule| MeshInput::new(&fx.mesh, r, &bnd, &comp).topology_content_hash();
    assert_ne!(key(&rule), key(&plain));
    let other = RegionRule::ByTaggedCellSets(vec![RegionSet::new("volume_1", "fuel").with_tag("group", 7), RegionSet::new("volume_2", "water")]);
    assert_ne!(key(&rule), key(&other));
}

#[test]
fn plain_cell_sets_smrt_bytes_are_unchanged() {
    let fx = hex_block_two_volumes([2, 2, 2], [1.0; 3], 1).unwrap();
    let (bytes, t) = smrt_of(&fx.mesh, &RegionRule::ByCellSets(vec!["volume_1".into(), "volume_2".into()]));
    let mut h = ContentHasher::new();
    h.write_bytes(&bytes);
    // Recorded on the pre-change build (`69ba521`).
    assert_eq!(bytes.len(), 7488);
    assert_eq!(h.finish().0, 266479812825674296866060044566060512153);
    assert_eq!(t.info().structure_hash.0, 306809543046941270234524496043451439313);
    assert_eq!(t.regions().0[0].name, "volume_1");
}
