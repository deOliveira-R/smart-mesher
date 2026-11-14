//! `model.subdivide_pin` (W4-REGIONS-CUT, ruling D9): rings, sectors, tags, errors, determinism.

mod common;

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::Arc;

use common::rig::*;
use common::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_boolean::subdivide::{RingCount, RingMode, RingRadii, RingSpec, SectorSpec, SubdividePin};
use sm_core::{EntityId, Point3, Vector3};
use sm_layer_api::{LayerResultExt, model_txn};
use sm_model_api::{LatticeFrame, Layout, RegionTag, SlotIx, SubIndex};
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Hash256, OpError, Services, Session, TimelineError, TypedOp,
};

const Z: [f64; 3] = [0.0, 0.0, 1.0];

/// Defines the frame `pins` (3 x 3, pitch 1.26) and tags a volume with a slot (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct TagSlot {
    volume: EntityId,
    slot: [i32; 3],
}

impl TypedOp for TagSlot {
    const NAME: &'static str = "test.tag_slot";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |m, _t| {
                if m.lattice_frame("pins").is_none() {
                    let layout = Layout::Cartesian {
                        n: [3, 3, 1],
                        pitch: Vector3::new(1.26, 1.26, 1.0),
                    };
                    m.define_lattice_frame(
                        "pins",
                        LatticeFrame::new(layout, Point3::new(0.0, 0.0, 0.0), None),
                    )
                    .or_op()?;
                }
                let ix = SlotIx::new(self.slot[0], self.slot[1], self.slot[2]);
                m.set_region_tag(
                    self.volume,
                    RegionTag::new([("pins".to_string(), ix)].into(), None, None),
                )
                .or_op()
            },
            |_, _| Vec::new(),
        )
    }
}

/// Clears the region tag of a volume (test only).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct ClearTag {
    volume: EntityId,
}

impl TypedOp for ClearTag {
    const NAME: &'static str = "test.clear_tag";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        model_txn(
            ctx,
            |m, _t| {
                m.clear_region_tag(self.volume);
                Ok(())
            },
            |_, _| Vec::new(),
        )
    }
}

fn rig() -> Rig {
    let mut reg = common::rig::registry(cell_complex);
    reg.register::<TagSlot>().expect("tag slot");
    reg.register::<ClearTag>().expect("clear tag");
    let engine = Engine::new(
        Arc::new(reg),
        Services::new(),
        Arc::new(DerivedCache::default()),
    )
    .expect("engine");
    Rig {
        s: Session::new(engine),
    }
}

fn rings(count: u32, mode: Option<RingMode>) -> Option<RingSpec> {
    Some(RingSpec::Count(RingCount { count, mode }))
}

fn sectors(count: u32, angle0_deg: Option<f64>) -> Option<SectorSpec> {
    Some(SectorSpec { count, angle0_deg })
}

fn pin(v: EntityId, rings: Option<RingSpec>, sectors: Option<SectorSpec>) -> SubdividePin {
    SubdividePin {
        volumes: vec![v],
        axis: Z,
        center: [0.0; 3],
        rings,
        sectors,
    }
}

fn invalid(e: TimelineError) -> String {
    match e {
        TimelineError::Apply(sm_op_api::ApplyError::Op { source, .. }) => format!("{source}"),
        other => panic!("{other:?}"),
    }
}

fn tag_of(r: &Rig, v: EntityId) -> RegionTag {
    r.layer()
        .model()
        .region_tag(v)
        .cloned()
        .unwrap_or_else(|| panic!("{v} has no tag"))
}

fn vol(r: &Rig, v: EntityId) -> f64 {
    volume_of(r.topo(), v)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// A solid pin (`r_in = 0`) or an annulus, height 1 about the z axis through the origin.
fn make_pin(r: &mut Rig, r_in: f64, r_out: f64) -> EntityId {
    let outer = r.cylinder(1.0, r_out, [0.0; 3]);
    if r_in == 0.0 {
        return outer.volume;
    }
    // The annulus is the outer ring of a one-radius cut of the solid pin, with its tag cleared.
    // (The default-seam webcut at 0.418 in 0.475 works since W4-BOOL-TOOLS; this path keeps the
    // turned seam of `subdivide_pin`.)
    let d = r
        .run(&pin(
            outer.volume,
            Some(RingSpec::Radii(RingRadii { radii: vec![r_in] })),
            None,
        ))
        .unwrap();
    let ann = d.outputs["volumes"]
        .iter()
        .copied()
        .find(|&x| tag_of(r, x).ring.unwrap().index == 1)
        .unwrap();
    r.run(&ClearTag { volume: ann }).unwrap();
    ann
}

#[test]
fn rings_and_sectors_of_a_solid_pin() {
    let (r_out, h) = (0.4096, 1.0);
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, r_out);
    let d = r
        .run(&pin(
            v,
            rings(3, Some(RingMode::EqualArea)),
            sectors(4, None),
        ))
        .unwrap();
    let pieces = d.outputs["volumes"].clone();
    assert_eq!(pieces.len(), 12);
    assert_eq!(d.outputs["bodies"].len(), 12);
    let t = r.topo();
    assert_valid(t);
    let whole = PI * r_out * r_out * h;
    let mut sum = 0.0;
    let mut seen = BTreeMap::new();
    for &p in &pieces {
        let x = vol(&r, p);
        assert!(rel(x, whole / 12.0) < 1e-9, "{x} vs {}", whole / 12.0);
        sum += x;
        let tag = tag_of(&r, p);
        let (ring, sec) = (tag.ring.unwrap(), tag.sector.unwrap());
        assert_eq!((ring.count, sec.count), (3, 4));
        assert!(seen.insert((ring.index, sec.index), p).is_none());
        assert!(tag.slots.is_empty());
    }
    assert_eq!(seen.len(), 12);
    for i in 0..3 {
        for j in 0..4 {
            assert!(seen.contains_key(&(i, j)), "ring {i} sector {j}");
        }
    }
    assert!(rel(sum, whole) < 1e-12);
    // Geometry of the labels: ring 0 is innermost, sector s starts at angle 90 s degrees.
    let c_of = |p| r.topo().volume_moments(p).unwrap().centroid().unwrap();
    for (&(ring, sec), &p) in &seen {
        let c = c_of(p);
        let ang = c.y.atan2(c.x).rem_euclid(2.0 * PI).to_degrees();
        let want = 90.0 * f64::from(sec) + 45.0;
        assert!((ang - want).abs() < 1e-6, "sector {sec} centroid at {ang}");
        let rad = c.x.hypot(c.y);
        let _ = (ring, rad);
    }
    let rad_of = |ring| c_of(seen[&(ring, 0)]).x.hypot(c_of(seen[&(ring, 0)]).y);
    assert!(rad_of(0) < rad_of(1) && rad_of(1) < rad_of(2));
    // The source volume is gone; its tag table entry too.
    assert!(r.layer().model().region_tag(v).is_none() || pieces.contains(&v));
}

#[test]
fn annulus_two_equal_area_rings() {
    let (ri, ro) = (0.418, 0.475);
    let mut r = rig();
    let v = make_pin(&mut r, ri, ro);
    let d = r.run(&pin(v, rings(2, None), None)).unwrap();
    let p = d.outputs["volumes"].clone();
    assert_eq!(p.len(), 2);
    assert_valid(r.topo());
    let half = PI * (ro * ro - ri * ri) / 2.0;
    for &x in &p {
        assert!(rel(vol(&r, x), half) < 1e-9);
        assert!(tag_of(&r, x).sector.is_none());
    }
    let mut ix: Vec<_> = p.iter().map(|&x| tag_of(&r, x).ring).collect();
    ix.sort_by_key(|t| t.map(|s| s.index));
    assert_eq!(
        ix,
        vec![Some(SubIndex::new(0, 2)), Some(SubIndex::new(1, 2))]
    );
}

#[test]
fn equal_radius_and_explicit_radii() {
    let r_out = 0.4096;
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, r_out);
    let d = r
        .run(&pin(
            v,
            Some(RingSpec::Radii(RingRadii {
                radii: vec![0.2, 0.3],
            })),
            None,
        ))
        .unwrap();
    let p = d.outputs["volumes"].clone();
    assert_eq!(p.len(), 3);
    let want = [PI * 0.04, PI * (0.09 - 0.04), PI * (r_out * r_out - 0.09)];
    for (i, w) in want.iter().enumerate() {
        let x = p
            .iter()
            .copied()
            .find(|&x| tag_of(&r, x).ring.unwrap().index == i as u32)
            .unwrap();
        assert!(rel(vol(&r, x), *w) < 1e-9, "ring {i}");
    }
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.4);
    let d = r
        .run(&pin(v, rings(4, Some(RingMode::EqualRadius)), None))
        .unwrap();
    for &x in &d.outputs["volumes"] {
        let i = f64::from(tag_of(&r, x).ring.unwrap().index);
        let w = PI * (((i + 1.0) * 0.1_f64).powi(2) - (i * 0.1_f64).powi(2));
        assert!(rel(vol(&r, x), w) < 1e-9, "ring {i}");
    }
}

#[test]
fn sectors_only_with_an_offset_start_angle() {
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.5);
    let d = r.run(&pin(v, None, sectors(6, Some(30.0)))).unwrap();
    let p = d.outputs["volumes"].clone();
    assert_eq!(p.len(), 6);
    assert_valid(r.topo());
    for &x in &p {
        let t = tag_of(&r, x);
        assert!(t.ring.is_none());
        let s = t.sector.unwrap();
        assert_eq!(s.count, 6);
        assert!(rel(vol(&r, x), PI * 0.25 / 6.0) < 1e-9);
        let c = r.topo().volume_moments(x).unwrap().centroid().unwrap();
        let ang = c.y.atan2(c.x).to_degrees().rem_euclid(360.0);
        let want = (30.0 + 60.0 * f64::from(s.index) + 30.0) % 360.0;
        assert!(
            ((ang - want + 180.0).rem_euclid(360.0) - 180.0).abs() < 1e-6,
            "sector {} at {ang}",
            s.index
        );
    }
}

#[test]
fn counts_of_one_cut_nothing_and_tag_the_volume() {
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.5);
    let before = r.topo().cells(sm_topo_api::Dim::Volume).len();
    let d = r.run(&pin(v, rings(1, None), sectors(1, None))).unwrap();
    assert_eq!(d.outputs["volumes"], vec![v]);
    assert_eq!(r.topo().cells(sm_topo_api::Dim::Volume).len(), before);
    let t = tag_of(&r, v);
    assert_eq!(
        (t.ring, t.sector),
        (Some(SubIndex::new(0, 1)), Some(SubIndex::new(0, 1)))
    );
}

#[test]
fn a_lattice_slot_stays_on_every_piece() {
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.4);
    r.run(&TagSlot {
        volume: v,
        slot: [2, 1, 0],
    })
    .unwrap();
    let d = r.run(&pin(v, rings(2, None), sectors(2, None))).unwrap();
    assert_eq!(d.outputs["volumes"].len(), 4);
    for &p in &d.outputs["volumes"] {
        let t = tag_of(&r, p);
        assert_eq!(t.slots.get("pins"), Some(&SlotIx::new(2, 1, 0)));
        assert!(t.ring.is_some() && t.sector.is_some());
    }
    // The source tag is gone with the source volume; only pieces are tagged.
    assert_eq!(r.layer().model().region_tags().len(), 4);
}

#[test]
fn two_stacked_pins_on_an_off_origin_axis_in_one_call() {
    let mut r = rig();
    let a = r.cylinder(1.0, 0.4, [1.0, 2.0, 0.0]);
    let b = r.cylinder(1.0, 0.25, [1.0, 2.0, 5.0]);
    let op = SubdividePin {
        volumes: vec![a.volume, b.volume],
        axis: Z,
        center: [1.0, 2.0, 0.0],
        rings: rings(2, None),
        sectors: sectors(2, Some(10.0)),
    };
    let d = r.run(&op).unwrap();
    assert_eq!(d.outputs["volumes"].len(), 8);
    assert_valid(r.topo());
    let sum: f64 = d.outputs["volumes"].iter().map(|&p| vol(&r, p)).sum();
    assert!(rel(sum, PI * (0.16 + 0.0625)) < 1e-12);
    // Each pin has its own equal-area radius: pieces of the thin pin are a quarter of its area.
    let mut vols: Vec<f64> = d.outputs["volumes"].iter().map(|&p| vol(&r, p)).collect();
    vols.sort_by(f64::total_cmp);
    assert!(rel(vols[0], PI * 0.0625 / 4.0) < 1e-9 && rel(vols[7], PI * 0.16 / 4.0) < 1e-9);
}

#[test]
fn typed_errors_and_atomic_failure() {
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.4);
    let e = invalid(r.run(&pin(v, None, sectors(3, None))).unwrap_err());
    assert!(e.contains("1 or even"), "{e}");
    assert!(invalid(r.run(&pin(v, None, sectors(0, None))).unwrap_err()).contains("count"));
    assert!(invalid(r.run(&pin(v, None, None)).unwrap_err()).contains("rings"));
    // Radii outside (r_in, r_out), not increasing, empty.
    for bad in [
        vec![0.5],
        vec![0.0],
        vec![0.3, 0.2],
        vec![0.2, 0.2],
        vec![0.4],
        vec![],
    ] {
        let e = invalid(
            r.run(&pin(
                v,
                Some(RingSpec::Radii(RingRadii { radii: bad.clone() })),
                None,
            ))
            .unwrap_err(),
        );
        assert!(
            e.contains("ring radius") || e.contains("radii"),
            "{bad:?}: {e}"
        );
    }
    let ann = make_pin(&mut r, 0.2, 0.4);
    let e = invalid(
        r.run(&pin(
            ann,
            Some(RingSpec::Radii(RingRadii { radii: vec![0.15] })),
            None,
        ))
        .unwrap_err(),
    );
    assert!(e.contains("ring radius"), "{e}");
    // Wrong axis, wrong centre, a brick, a sector piece.
    let e = invalid(
        r.run(&SubdividePin {
            volumes: vec![v],
            axis: [1.0, 0.0, 0.0],
            center: [0.0; 3],
            rings: rings(2, None),
            sectors: None,
        })
        .unwrap_err(),
    );
    assert!(e.contains("coaxial"), "{e}");
    let e = invalid(
        r.run(&SubdividePin {
            volumes: vec![v],
            axis: Z,
            center: [0.1, 0.0, 0.0],
            rings: rings(2, None),
            sectors: None,
        })
        .unwrap_err(),
    );
    assert!(e.contains("coaxial"), "{e}");
    let b = r.brick([1.0; 3], [5.0; 3]);
    let e = invalid(r.run(&pin(b.volume, rings(2, None), None)).unwrap_err());
    assert!(e.contains("coaxial"), "{e}");
    // A body is not a volume.
    assert!(r.run(&pin(b.body, rings(2, None), None)).is_err());
    let d = r.run(&pin(v, None, sectors(2, None))).unwrap();
    let half = d.outputs["volumes"][0];
    let e = invalid(r.run(&pin(half, None, sectors(2, None))).unwrap_err());
    assert!(e.contains("coaxial") || e.contains("sector"), "{e}");
    // Re-cutting a ring piece with rings is refused by its tag.
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.4);
    let d = r.run(&pin(v, rings(2, None), None)).unwrap();
    let e = invalid(
        r.run(&pin(d.outputs["volumes"][1], rings(2, None), None))
            .unwrap_err(),
    );
    assert!(e.contains("already carries"), "{e}");
}

#[test]
fn a_failing_call_leaves_the_hash_unchanged() {
    let mut r = rig();
    let v = make_pin(&mut r, 0.0, 0.4);
    let h = r.hash();
    assert!(r.run(&pin(v, rings(2, None), sectors(5, None))).is_err());
    assert_eq!(r.hash(), h);
    assert!(
        r.run(&pin(
            v,
            Some(RingSpec::Radii(RingRadii {
                radii: vec![0.1, 0.9]
            })),
            None
        ))
        .is_err()
    );
    assert_eq!(r.hash(), h);
}

fn scenario(r: &mut Rig) -> Hash256 {
    let a = make_pin(r, 0.0, 0.4096);
    r.run(&TagSlot {
        volume: a,
        slot: [1, 1, 0],
    })
    .unwrap();
    r.run(&pin(a, rings(3, None), sectors(4, Some(15.0))))
        .unwrap();
    let b = make_pin(r, 0.418, 0.475);
    r.run(&pin(
        b,
        rings(2, Some(RingMode::EqualRadius)),
        sectors(2, None),
    ))
    .unwrap();
    r.hash()
}

#[test]
fn deterministic_across_runs_and_thread_counts() {
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| scenario(&mut rig()))
    };
    let h1 = run(1);
    assert_eq!(h1, run(1));
    assert_eq!(h1, run(4));
    assert_eq!(h1, scenario(&mut rig()));
}
