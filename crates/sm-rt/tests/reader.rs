//! W3b-RT-WORLD scope 2: the tracking-file reader path (`TrackerBuilder::load_image`), the Arc
//! builder entry (`Tracker::mesh`), region tags and `LostCause::SenseMismatch` / `retries`.
mod common;
use common::*;
use sm_core::{Point3, Vector3};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt::{BuildExtras, MeshTrackerBuilder, read_smrt, write_smrt};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, CacheImage, CellId, ComplementPolicy, LostCause, MeshInput, RecoveryPolicy, RegionId, RegionRule, RtError, Scratch, SectionTag, StepEnd,
    TraceOptions, TrackState, Tracker, TrackerBuilder,
};
use std::sync::Arc;

fn rays(name: &str, n: usize) -> Vec<sm_geom_api::Ray> {
    let mut r = rng(name);
    (0..n)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            let u = |r: &mut SplitMix64| -0.5 + 7.0 * (r.unit() + 1.0) / 2.0;
            sm_geom_api::Ray { origin: p3([u(&mut r), u(&mut r), u(&mut r)]), dir: v3(d) }
        })
        .collect()
}

#[test]
fn acc4_image_file_reader_round_trip_without_the_input() {
    let mesh = hex_block([6; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec, &comp));
    let opts = BuildOptions::default();
    let b = MeshTrackerBuilder;
    let (built, _) = b.build(&input, &opts).unwrap();
    // The handover file: bytes of the image, read back and opened with no input at all.
    let mut bytes = Vec::new();
    write_smrt(&mut bytes, &built.export_cache().unwrap()).unwrap();
    let image = read_smrt(&bytes).unwrap();
    let (opened, report) = b.load_image(&image).unwrap();
    assert!(report.from_cache);
    assert_eq!(opened.structure_hash(), built.structure_hash());
    assert_eq!(opened.info().cells, built.info().cells);
    assert_eq!(opened.regions(), built.regions());
    let rs = rays("acc4_reader", 100_000);
    let o = TraceOptions::default();
    let (x, y) = (built.trace_bundle(&rs, &o).unwrap(), opened.trace_bundle(&rs, &o).unwrap());
    assert_eq!(x, y);
    assert_eq!(x.content_hash(), y.content_hash());
    // A flipped byte anywhere in the file is a typed error (section hashes), never a panic.
    for k in (0..bytes.len()).step_by((bytes.len() / 300).max(1)) {
        let mut bad = bytes.clone();
        bad[k] ^= 0x41;
        match read_smrt(&bad) {
            Err(RtError::CacheCorrupt(_) | RtError::CacheStale(_)) => {}
            Err(e) => panic!("byte {k}: {e}"),
            Ok(img) => {
                // Padding bytes are not covered by a hash; the image must then still open to the same tracker.
                if let Ok((t, _)) = b.load_image(&img) {
                    assert_eq!(t.structure_hash(), built.structure_hash(), "byte {k}");
                }
            }
        }
    }
}

#[test]
fn arc_builder_keeps_the_mesh_and_tags_round_trip_through_the_image() {
    let mesh: Arc<dyn sm_mesh_api::MeshRead + Send + Sync> = Arc::new(hex_block([3; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap());
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let opts = BuildOptions::default();
    let b = MeshTrackerBuilder;
    let (t, _) = b.build_arc(mesh.clone(), &rules, &spec, &comp, &opts).unwrap();
    let m = t.mesh().expect("mesh() is Some through the Arc entry");
    assert_eq!(m.cell_count(), 27);
    assert_eq!(m.mesh_id(), mesh.mesh_id());
    // Same structure as the borrowed build.
    let input = BuildInput::Mesh(MeshInput::new(&*mesh, &rules, &spec, &comp));
    let (plain, _) = b.build(&input, &opts).unwrap();
    assert!(plain.mesh().is_none());
    assert_eq!(plain.structure_hash(), t.structure_hash());
    // The image of an Arc build has no mesh after loading, as before.
    let (opened, _) = b.load_image(&t.export_cache().unwrap()).unwrap();
    assert!(opened.mesh().is_none());

    // Tags: attached by region name, hashed, kept in the image, absent images stay byte-identical.
    let region_name = plain.regions().0[0].name.clone();
    let extras = BuildExtras { mesh: None, tags: vec![(region_name.clone(), "group".into(), 7), (region_name.clone(), "mat".into(), -3)] };
    let (tagged, _) = b.build_with(&input, &opts, &extras).unwrap();
    assert_eq!(tagged.regions().0[0].tags, vec![("group".to_string(), 7), ("mat".to_string(), -3)]);
    assert_ne!(tagged.structure_hash(), plain.structure_hash());
    let img = tagged.export_cache().unwrap();
    assert!(img.sections.iter().any(|(t, _)| matches!(t, SectionTag::Private(_))));
    let (back, _) = b.load_image(&read_smrt(&img.encode()).unwrap()).unwrap();
    assert_eq!(back.regions(), tagged.regions());
    assert_eq!(back.structure_hash(), tagged.structure_hash());
    assert_eq!(back.export_cache().unwrap().encode(), img.encode());
    // Tag-free images do not carry the section (their bytes are the pre-tags bytes).
    let plain_img = plain.export_cache().unwrap();
    assert_eq!(plain_img.sections.len(), 10);
    // The tagged key differs, so loading it against the untagged request is stale.
    assert!(matches!(b.load(&input, &opts, &img), Err(RtError::CacheStale(_))));
    // An unknown region name is a build error.
    let bad = BuildExtras { mesh: None, tags: vec![("nope".into(), "k".into(), 1)] };
    assert!(matches!(b.build_with(&input, &opts, &bad), Err(RtError::Build(v)) if v[0].code == "region.tag_unknown"));
    // Hostile tag section: wrong region count / truncated bytes are typed errors.
    for k in 0..img.sections.len() {
        if !matches!(img.sections[k].0, SectionTag::Private(4)) {
            continue;
        }
        for cut in [0usize, 8, 15] {
            let mut sections = img.sections.clone();
            sections[k].1.truncate(cut);
            let evil = CacheImage { key: img.key, sections };
            assert!(matches!(b.load_image(&evil), Err(RtError::CacheCorrupt(_))), "cut {cut}");
        }
        let mut sections = img.sections.clone();
        sections[k].1[0] ^= 0x7;
        assert!(b.load_image(&CacheImage { key: img.key, sections }).is_err());
    }
}

#[test]
fn sense_mismatch_and_retries_are_reported() {
    // The shell cell of two nested spheres, with a state at the centre (inside the cavity, so
    // outside the shell) heading out: the first crossing is an entry into the shell and there is
    // no exit.
    let f = nested_facets();
    let mut sc = Scratch::new();
    let fail = build(&f, &opts_fail()).unwrap();
    let mk = || {
        let mut st = TrackState::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        st.cell = CellId::new(1);
        st.region = RegionId::new(1);
        st
    };
    let mut st = mk();
    let s = fail.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    let StepEnd::Lost(l) = s.end else { panic!("{:?}", s.end) };
    assert_eq!(l.cause, LostCause::SenseMismatch);
    assert_eq!(l.retries, 0);
    assert_eq!(st, mk(), "a lost step leaves the state unchanged");
    // With nudging a state far outside the model is RecoveryFailed and `retries` counts the
    // nudges tried; a distance limit shorter than the first nudge stops the retries early.
    let mesh = hex_block([4; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let mut o = BuildOptions::default();
    o.recovery = RecoveryPolicy::Nudge { tries: 3 };
    let nudge = build_mesh_tracker(&mesh, &vacuum(), &o);
    let out = || {
        let mut st = TrackState::new(Point3::new(9.0, 0.5, 0.5), Vector3::new(0.0, 1.0, 0.0));
        st.cell = CellId::new(0);
        st.region = RegionId::new(0);
        st
    };
    let mut st = out();
    let StepEnd::Lost(l) = nudge.step(&mut st, f64::INFINITY, &mut sc).unwrap().end else { panic!() };
    assert_eq!((l.cause, l.retries), (LostCause::RecoveryFailed, 3));
    let mut st = out();
    let StepEnd::Lost(l) = nudge.step(&mut st, 1e-9, &mut sc).unwrap().end else { panic!() };
    assert_eq!(l.retries, 0, "no nudge fits in the limit");
}
