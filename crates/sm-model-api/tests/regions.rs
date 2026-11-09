//! Lattice frames and region tags (ACR-REGIONS, ruling D9): locate, lineage, hashing, serde.

use proptest::prelude::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::ModelTolerance;
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, Lineage, OpId};
use smallvec::smallvec;
use std::collections::BTreeMap;

const PITCH: f64 = 1.26;

fn vol(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoVolume, n).unwrap()
}

fn words(m: &Model) -> Vec<u64> {
    let mut v = Vec::new();
    m.canonical_visit(&ModelTolerance::DEFAULT, &mut |w| {
        v.push(match w {
            sm_topo_api::HashWord::Int(x) => x,
            sm_topo_api::HashWord::Float { q, .. } => q as u64,
        })
    });
    v
}

fn cart(origin: Point3) -> LatticeFrame {
    LatticeFrame::new(Layout::Cartesian { n: [3, 3, 1], pitch: Vector3::new(PITCH, PITCH, 1.0) }, origin, None)
}

fn hex(orient: HexOrient, origin: Point3) -> LatticeFrame {
    LatticeFrame::new(Layout::Hex { rings: 3, pitch: PITCH, orient, nz: 1, dz: 1.0 }, origin, None)
}

fn tag(frame: &str, ix: SlotIx) -> RegionTag {
    RegionTag::new([(frame.to_string(), ix)].into(), None, None)
}

fn lineage(old: EntityId, new: &[EntityId]) -> Lineage {
    Lineage { old, new: new.iter().copied().collect() }
}

fn topo() -> CellComplex {
    let mut t = CellComplex::new();
    with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 1.0, 1.0])).unwrap();
    t
}

// Pre-change outputs (commit 69ba521), pinned: an empty model and a model with one color.
const EMPTY_WORDS: &[u64] = &[1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0, 11, 0, 0, 0, 0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
const COLOR_WORDS: &[u64] = &[1, 1, 32, 2, 2, 1, 2305843009213693953, 2305843009213693953, 1, 3, 7627117, 1, 3, 0, 4, 0, 5, 0, 6, 0, 7, 0, 8, 0, 9, 0, 10, 0, 11, 0, 0, 0, 0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

#[test]
fn empty_model_word_stream_is_unchanged() {
    assert_eq!(words(&Model::new()), EMPTY_WORDS);
    let mut m = Model::new();
    m.add_color(1, "mat", Domain::Volume).unwrap();
    assert_eq!(words(&m), COLOR_WORDS);
}

#[cfg(feature = "serde")]
#[test]
fn empty_model_serde_is_unchanged() {
    const EMPTY_JSON: &str = r#"{"ids":{"next":{}},"colors":{},"volume_color":{},"face_color":{},"groups":{},"boundaries":{},"face_boundary":{},"symmetry":null,"provenance":[],"cuts":[],"instancing":{"prototypes":{},"lattices":{},"instances":{},"overrides":[],"clusters":{}},"decomposition":{"cells":{},"splits":[],"parent":{},"composites":[],"consumed":[],"by_volume":{},"by_face":{},"cut_serial":0,"whole_faces":{},"composite_faces":{},"realized":[],"face_cells":[],"split_cells":{}}}"#;
    const COLOR_JSON: &str = r#"{"ids":{"next":{"32":2}},"colors":{"Color#1":{"id":"Color#1","label":1,"name":"mat","domain":"Volume"}},"volume_color":{},"face_color":{},"groups":{},"boundaries":{},"face_boundary":{},"symmetry":null,"provenance":[],"cuts":[],"instancing":{"prototypes":{},"lattices":{},"instances":{},"overrides":[],"clusters":{}},"decomposition":{"cells":{},"splits":[],"parent":{},"composites":[],"consumed":[],"by_volume":{},"by_face":{},"cut_serial":0,"whole_faces":{},"composite_faces":{},"realized":[],"face_cells":[],"split_cells":{}}}"#;
    assert_eq!(serde_json::to_string(&Model::new()).unwrap(), EMPTY_JSON);
    let mut m = Model::new();
    m.add_color(1, "mat", Domain::Volume).unwrap();
    assert_eq!(serde_json::to_string(&m).unwrap(), COLOR_JSON);
    // Old documents without the new keys still load.
    let back: Model = serde_json::from_str(EMPTY_JSON).unwrap();
    assert_eq!(back, Model::new());
}

#[test]
fn locate_cartesian() {
    let f = cart(Point3::new(0.0, 0.0, 0.0));
    assert_eq!(f.locate(Point3::new(1.26, 2.52, 0.0)), Some(SlotIx::new(1, 2, 0)));
    assert_eq!(f.locate(Point3::new(0.0, 0.0, 0.0)), Some(SlotIx::new(0, 0, 0)));
    // Tie rule: a point on the boundary between two slots belongs to the lower index.
    assert_eq!(f.locate(Point3::new(1.89, 0.0, 0.0)), Some(SlotIx::new(1, 0, 0)));
    assert_eq!(f.locate(Point3::new(0.0, 0.63, 0.0)), Some(SlotIx::new(0, 0, 0)));
    assert_eq!(f.locate(Point3::new(1.9, 0.0, 0.0)), Some(SlotIx::new(2, 0, 0)));
    // Outside the layout.
    assert_eq!(f.locate(Point3::new(-0.7, 0.0, 0.0)), None);
    assert_eq!(f.locate(Point3::new(3.2 * PITCH, 0.0, 0.0)), None);
    assert_eq!(f.locate(Point3::new(0.0, 0.0, 1.0)), None);
    assert_eq!(f.locate(Point3::new(f64::NAN, 0.0, 0.0)), None);
    // A shifted origin.
    let g = cart(Point3::new(10.0, -5.0, 2.0));
    assert_eq!(g.locate(Point3::new(10.0 + PITCH, -5.0 + 2.0 * PITCH, 2.1)), Some(SlotIx::new(1, 2, 0)));
}

#[test]
fn locate_hex_round_trips_and_stays_in_slot() {
    const NEIGHBOURS: [(i32, i32); 6] = [(1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1)];
    for orient in [HexOrient::XRows, HexOrient::YRows] {
        for origin in [Point3::new(0.0, 0.0, 0.0), Point3::new(3.5, -2.25, 1.0)] {
            let f = hex(orient, origin);
            let slots = f.layout.indices();
            assert_eq!(slots.len(), 19);
            for ix in slots {
                let c = f.layout.offset(ix);
                let centre = Point3::new(origin.x + c.x, origin.y + c.y, origin.z + c.z);
                assert_eq!(f.locate(centre), Some(ix), "{orient:?} {ix:?}");
                for (dq, dr) in NEIGHBOURS {
                    let n = f.layout.offset(SlotIx::new(ix.i + dq, ix.j + dr, 0));
                    let p = Point3::new(centre.x + 0.49 * (n.x - c.x), centre.y + 0.49 * (n.y - c.y), centre.z);
                    assert_eq!(f.locate(p), Some(ix), "{orient:?} {ix:?} toward ({dq},{dr})");
                }
            }
            // The ring just outside is not in the layout.
            let c = f.layout.offset(SlotIx::new(3, 0, 0));
            assert_eq!(f.locate(Point3::new(origin.x + c.x, origin.y + c.y, origin.z)), None);
        }
    }
}

#[test]
fn locate_hex_layers() {
    let f = LatticeFrame::new(Layout::Hex { rings: 2, pitch: PITCH, orient: HexOrient::XRows, nz: 3, dz: 2.0 }, Point3::new(0.0, 0.0, 0.0), None);
    assert_eq!(f.locate(Point3::new(0.0, 0.0, 4.0)), Some(SlotIx::new(0, 0, 2)));
    assert_eq!(f.locate(Point3::new(0.0, 0.0, 6.5)), None);
}

#[test]
fn lineage_split_merge_equal_and_different() {
    let t = topo();
    let mut m = Model::new();
    m.define_lattice_frame("a", cart(Point3::new(0.0, 0.0, 0.0))).unwrap();
    // Split: one volume to three copies the tag to all three.
    let tg = tag("a", SlotIx::new(1, 2, 0));
    m.set_region_tag(vol(1), tg.clone()).unwrap();
    m.apply_lineage(&[lineage(vol(1), &[vol(10), vol(11), vol(12)])], &t).unwrap();
    assert!(m.region_tag(vol(1)).is_none());
    for v in [10, 11, 12] {
        assert_eq!(m.region_tag(vol(v)), Some(&tg));
    }
    // Merge of different tags: no tag.
    m.set_region_tag(vol(11), tag("a", SlotIx::new(0, 0, 0))).unwrap();
    m.apply_lineage(&[lineage(vol(10), &[vol(20)]), lineage(vol(11), &[vol(20)])], &t).unwrap();
    assert!(m.region_tag(vol(20)).is_none());
    assert_eq!(m.region_tag(vol(12)), Some(&tg));
    // Merge of equal tags keeps it (and a third, different tag afterwards still removes it).
    m.set_region_tag(vol(13), tg.clone()).unwrap();
    m.apply_lineage(&[lineage(vol(12), &[vol(30)]), lineage(vol(13), &[vol(30)])], &t).unwrap();
    assert_eq!(m.region_tag(vol(30)), Some(&tg));
    m.set_region_tag(vol(31), tag("a", SlotIx::new(2, 2, 0))).unwrap();
    m.set_region_tag(vol(32), tg.clone()).unwrap();
    m.apply_lineage(&[lineage(vol(30), &[vol(40)]), lineage(vol(31), &[vol(40)]), lineage(vol(32), &[vol(40)])], &t).unwrap();
    assert!(m.region_tag(vol(40)).is_none(), "a conflict stays a conflict");
    // A removed volume drops its tag; the model stays consistent.
    m.set_region_tag(vol(50), tg).unwrap();
    m.apply_lineage(&[Lineage { old: vol(50), new: smallvec![] }], &t).unwrap();
    assert!(m.region_tag(vol(50)).is_none());
    m.check_integrity().unwrap();
}

#[test]
fn word_stream_hashes_the_tables() {
    let empty = words(&Model::new());
    let mut m = Model::new();
    m.define_lattice_frame("a", cart(Point3::new(0.0, 0.0, 0.0))).unwrap();
    let with_frame = words(&m);
    assert_ne!(with_frame, empty);
    // The prefix up to the end of the old tables is untouched; the frame table is appended.
    assert_eq!(&with_frame[..empty.len()], &empty[..]);
    assert_eq!(with_frame[empty.len()], 13);
    m.set_region_tag(vol(1), tag("a", SlotIx::new(1, 1, 0))).unwrap();
    let with_tag = words(&m);
    assert_ne!(with_tag, with_frame);
    assert_eq!(&with_tag[..with_frame.len()], &with_frame[..]);
    // Every field matters.
    let mut m2 = m.clone();
    m2.set_region_tag(vol(1), RegionTag::new([("a".to_string(), SlotIx::new(1, 1, 0))].into(), Some(SubIndex::new(0, 2)), None)).unwrap();
    assert_ne!(words(&m2), with_tag);
    let mut m3 = Model::new();
    m3.define_lattice_frame("a", cart(Point3::new(0.5, 0.0, 0.0))).unwrap();
    m3.set_region_tag(vol(1), tag("a", SlotIx::new(1, 1, 0))).unwrap();
    assert_ne!(words(&m3), with_tag);
    // Removing everything restores the empty stream.
    m.clear_region_tag(vol(1));
    assert_eq!(words(&m), with_frame);
}

#[cfg(feature = "serde")]
#[test]
fn serde_round_trip_is_identity() {
    let mut m = Model::new();
    m.define_lattice_frame("core", cart(Point3::new(0.0, 0.0, 0.0))).unwrap();
    m.define_lattice_frame("asm", LatticeFrame::new(hex(HexOrient::YRows, Point3::new(1.0, 2.0, 3.0)).layout, Point3::new(1.0, 2.0, 3.0), Some("core".into()))).unwrap();
    let slots: BTreeMap<String, SlotIx> = [("core".to_string(), SlotIx::new(2, 1, 0)), ("asm".to_string(), SlotIx::new(-1, 2, 0))].into();
    m.set_region_tag(vol(3), RegionTag::new(slots, Some(SubIndex::new(1, 3)), Some(SubIndex::new(3, 4)))).unwrap();
    m.set_region_tag(vol(4), RegionTag::default()).unwrap();
    let s = serde_json::to_string(&m).unwrap();
    let back: Model = serde_json::from_str(&s).unwrap();
    assert_eq!(back, m);
    back.check_integrity().unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), s);
    // Bad data is refused at deserialization (integrity runs).
    let bad = s.replace("\"core\":{\"i\":2", "\"core\":{\"i\":3");
    assert_ne!(bad, s);
    assert!(serde_json::from_str::<Model>(&bad).is_err());
}

fn base_frames(swap: bool) -> Model {
    let mut m = Model::new();
    let a = (cart(Point3::new(0.0, 0.0, 0.0)), "a");
    let b = (hex(HexOrient::XRows, Point3::new(0.0, 0.0, 0.0)), "b");
    let order = if swap { [b, a] } else { [a, b] };
    for (f, n) in order {
        m.define_lattice_frame(n, f).unwrap();
    }
    m
}

fn tag_strategy() -> impl Strategy<Value = RegionTag> {
    let a = cart(Point3::new(0.0, 0.0, 0.0)).layout.indices();
    let b = hex(HexOrient::XRows, Point3::new(0.0, 0.0, 0.0)).layout.indices();
    let sub = (1u32..5).prop_flat_map(|c| (0..c, Just(c))).prop_map(|(i, c)| SubIndex::new(i, c));
    (proptest::option::of(proptest::sample::select(a)), proptest::option::of(proptest::sample::select(b)), proptest::option::of(sub.clone()), proptest::option::of(sub)).prop_map(|(sa, sb, ring, sector)| {
        let mut slots = BTreeMap::new();
        if let Some(x) = sa {
            slots.insert("a".to_string(), x);
        }
        if let Some(x) = sb {
            slots.insert("b".to_string(), x);
        }
        RegionTag::new(slots, ring, sector)
    })
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(64))]

    #[test]
    fn tag_tables_are_order_independent_and_round_trip(
        entries in proptest::collection::btree_map(1u64..40, tag_strategy(), 0..12),
        perm in any::<proptest::sample::Index>(),
        swap in any::<bool>(),
    ) {
        let mut forward = base_frames(false);
        for (k, t) in &entries {
            forward.set_region_tag(vol(*k), t.clone()).unwrap();
        }
        // A rotated insertion order and the other frame definition order.
        let mut list: Vec<_> = entries.iter().collect();
        if !list.is_empty() {
            let shift = perm.index(list.len());
            list.rotate_left(shift);
        }
        list.reverse();
        let mut other = base_frames(swap);
        for (k, t) in list {
            other.set_region_tag(vol(*k), t.clone()).unwrap();
        }
        prop_assert_eq!(&forward, &other);
        prop_assert_eq!(words(&forward), words(&other));
        forward.check_integrity().unwrap();
        #[cfg(feature = "serde")]
        {
            let s = serde_json::to_string(&forward).unwrap();
            let back: Model = serde_json::from_str(&s).unwrap();
            prop_assert_eq!(&back, &forward);
            prop_assert_eq!(words(&back), words(&forward));
        }
    }
}
