//! Acceptances 10 and 11: the `.smrt` cache image and determinism of builds and bundles.
mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_rt::{MeshTrackerBuilder, load_image, read_smrt, write_smrt, write_smrt_file};
use sm_rt_api::{
    BoundaryDef, BoundarySpec, BuildInput, BuildOptions, CacheImage, ComplementPolicy, ContentHash, ContentHasher, MeshInput, RegionRule, ResponseKind, RtError, SectionTag, Tracker,
    TrackerBuilder, TraceOptions,
};
use sm_geom_api::Ray;

fn rays(name: &str, n: usize, lo: [f64; 3], hi: [f64; 3]) -> Vec<Ray> {
    let mut r = rng(name);
    (0..n)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            let o = [lo[0] + (hi[0] - lo[0]) * (r.unit() + 1.0) / 2.0, lo[1] + (hi[1] - lo[1]) * (r.unit() + 1.0) / 2.0, lo[2] + (hi[2] - lo[2]) * (r.unit() + 1.0) / 2.0];
            Ray { origin: p3(o), dir: v3(d) }
        })
        .collect()
}

fn hash_bytes(b: &[u8]) -> ContentHash {
    let mut h = ContentHasher::new();
    h.write_bytes(b);
    h.finish()
}

#[test]
fn acc10_round_trip_through_smrt_is_bit_identical() {
    let mesh = hex_block([6; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec, &comp));
    let opts = BuildOptions::default();
    let b = MeshTrackerBuilder;
    let (built, _) = b.build(&input, &opts).unwrap();
    let img = built.export_cache().expect("cacheable");
    let mut bytes = Vec::new();
    write_smrt(&mut bytes, &img).unwrap();
    assert_eq!(bytes, img.encode(), "the file is the CacheImage bytes verbatim");
    assert_eq!(&bytes[..8], b"SMRTCACH");
    let path = std::env::temp_dir().join(format!("sm_rt_acc10_{}.smrt", std::process::id()));
    write_smrt_file(&path, &img).unwrap();
    let from_file = std::fs::read(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    assert_eq!(from_file, bytes);
    let back = read_smrt(&from_file).unwrap();
    assert_eq!(back, img);
    let (loaded, report) = b.load(&input, &opts, &back).unwrap();
    assert!(report.from_cache);
    assert_eq!(loaded.structure_hash(), built.structure_hash());
    assert_eq!(loaded.info(), built.info(), "TrackerInfo (everything but from_cache, which lives in the report)");
    assert_eq!(loaded.export_cache().unwrap().encode(), bytes, "re-export is canonical");
    // Fixed 1e5-ray bundle, bit-identical.
    let rs = rays("acc10_rays", 100_000, [-0.5; 3], [6.5; 3]);
    let topts = TraceOptions::default();
    let (x, y) = (built.trace_bundle(&rs, &topts).unwrap(), loaded.trace_bundle(&rs, &topts).unwrap());
    assert_eq!(x, y);
    assert_eq!(x.content_hash(), y.content_hash());
    // Without the mesh.
    let bare = load_image(&back).unwrap();
    assert_eq!(bare.structure_hash(), built.structure_hash());
    assert_eq!(bare.trace_bundle(&rs[..1000], &topts).unwrap(), built.trace_bundle(&rs[..1000], &topts).unwrap());
}

fn section_payloads(bytes: &[u8]) -> Vec<(usize, usize)> {
    let n = u32::from_le_bytes(bytes[20..24].try_into().unwrap()) as usize;
    (0..n)
        .map(|i| {
            let at = 128 + 32 * i;
            let off = u64::from_le_bytes(bytes[at + 8..at + 16].try_into().unwrap()) as usize;
            let len = u64::from_le_bytes(bytes[at + 16..at + 24].try_into().unwrap()) as usize;
            (off, len)
        })
        .collect()
}

#[test]
fn acc10_corruption_and_staleness_are_typed_errors() {
    let mesh = hex_block([3; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec, &comp));
    let opts = BuildOptions::default();
    let b = MeshTrackerBuilder;
    let (t, _) = b.build(&input, &opts).unwrap();
    let img = t.export_cache().unwrap();
    let bytes = img.encode();
    // One flipped byte in every section.
    for (i, (off, len)) in section_payloads(&bytes).into_iter().enumerate() {
        if len == 0 {
            continue;
        }
        let mut bad = bytes.clone();
        bad[off + len / 2] ^= 0x10;
        assert!(matches!(read_smrt(&bad), Err(RtError::CacheCorrupt(_))), "section {i}");
    }
    assert!(matches!(read_smrt(&bytes[..100]), Err(RtError::CacheCorrupt(_))));
    assert!(matches!(read_smrt(b"not a cache"), Err(RtError::CacheCorrupt(_))));
    // Hash-consistent tampering reaches the structural checks: typed error or an identical
    // tracker, never a panic. Flip every byte of the small sections, sampled in the big ones.
    let mut tampered = 0;
    for (si, (tag, data)) in img.sections.iter().enumerate() {
        let step = (data.len() / 200).max(1);
        for k in (0..data.len()).step_by(step) {
            let mut sections = img.sections.clone();
            sections[si].1[k] ^= 0x5a;
            let evil = CacheImage { key: img.key, sections };
            match load_image(&evil) {
                Err(RtError::CacheCorrupt(_) | RtError::CacheStale(_)) => {}
                Err(e) => panic!("section {tag:?} byte {k}: unexpected {e}"),
                Ok(t2) => assert_eq!(t2.structure_hash(), t.structure_hash(), "an accepted tamper must be a no-op ({tag:?} byte {k})"),
            }
            tampered += 1;
        }
    }
    assert!(tampered > 500);
    // Missing section.
    let mut sections = img.sections.clone();
    sections.remove(0);
    assert!(matches!(load_image(&CacheImage { key: img.key, sections }), Err(RtError::CacheCorrupt(_))));

    // Staleness: another coordinate, another boundary condition, another option.
    let other = hex_block([3; 3], [1.0, 1.0, 1.0000001], &mut SerialAlloc::new()).unwrap();
    let in2 = BuildInput::Mesh(MeshInput::new(&other, &rules, &spec, &comp));
    assert!(matches!(b.load(&in2, &opts, &img), Err(RtError::CacheStale("geometry changed"))));
    let spec2 = BoundarySpec::new(BoundaryDef::Respond(ResponseKind::White), vec![]);
    let in3 = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec2, &comp));
    assert!(matches!(b.load(&in3, &opts, &img), Err(RtError::CacheStale("boundary conditions changed"))));
    let mut o2 = opts;
    o2.bvh_leaf_size += 1;
    assert!(matches!(b.load(&input, &o2, &img), Err(RtError::CacheStale("options changed"))));
    // Keys: stable across repeated builds and thread counts, sensitive to every edit.
    let k0 = b.cache_key(&input, &opts).unwrap();
    for threads in [1, 3] {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        let (k, h) = pool.install(|| {
            let (t, _) = b.build(&input, &opts).unwrap();
            (b.cache_key(&input, &opts).unwrap(), t.structure_hash())
        });
        assert_eq!(k, k0);
        assert_eq!(h, t.structure_hash());
    }
    assert_ne!(b.cache_key(&in2, &opts).unwrap(), k0);
    assert_ne!(b.cache_key(&in3, &opts).unwrap(), k0);
    assert_ne!(b.cache_key(&input, &o2).unwrap(), k0);
}

#[test]
fn acc10_facet_trackers_round_trip_too() {
    let f = nested_facets();
    let spec = BoundarySpec::default();
    let input = BuildInput::Facets(&f, &spec);
    let opts = BuildOptions::default();
    let b = MeshTrackerBuilder;
    let (t, _) = b.build(&input, &opts).unwrap();
    let img = read_smrt(&t.export_cache().unwrap().encode()).unwrap();
    let (u, _) = b.load(&input, &opts, &img).unwrap();
    assert_eq!(u.info(), t.info());
    let rs = rays("acc10_facets", 20_000, [-2.5; 3], [2.5; 3]);
    assert_eq!(t.trace_bundle(&rs, &TraceOptions::default()).unwrap(), u.trace_bundle(&rs, &TraceOptions::default()).unwrap());
    let mut g = f.clone();
    g.vertices[3][0] += 1e-9;
    assert!(matches!(b.load(&BuildInput::Facets(&g, &spec), &opts, &img), Err(RtError::CacheStale("geometry changed"))));
    // Private sections stay private: all section tags are known to the contract decoder.
    assert!(img.sections.iter().all(|(t, _)| matches!(t, SectionTag::Private(_) | SectionTag::FacePlanes | SectionTag::CellInfo | SectionTag::Bvh | SectionTag::CellBvh | SectionTag::Tables | SectionTag::Boundaries)));
}

#[test]
fn acc11_build_cache_and_bundle_are_deterministic() {
    let mesh = hex_block([5; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let f = nested_facets();
    let rs = rays("acc11_rays", 100_000, [-0.5; 3], [5.5; 3]);
    let fr = rays("acc11_facet_rays", 100_000, [-2.2; 3], [2.2; 3]);
    let run = |threads: usize| -> (ContentHash, ContentHash, ContentHash, ContentHash) {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let t = build_mesh_tracker(&mesh, &vacuum(), &BuildOptions::default());
            let img = t.export_cache().unwrap().encode();
            let bundle = t.trace_bundle(&rs, &TraceOptions::default()).unwrap().content_hash();
            let spec = BoundarySpec::default();
            let (ft, _) = MeshTrackerBuilder.build(&BuildInput::Facets(&f, &spec), &BuildOptions::default()).unwrap();
            (hash_bytes(&img), bundle, hash_bytes(&ft.export_cache().unwrap().encode()), ft.trace_bundle(&fr, &TraceOptions::default()).unwrap().content_hash())
        })
    };
    sm_testkit::determinism::check(2, 4, run).unwrap();
}

#[test]
fn acc10_key_follows_the_mesh_identity_hashes() {
    use sm_testkit::meshes::{CellSpec, build_mesh};
    // The key is a pure function of the mesh's own identity hashes (`topology_hash`,
    // `geometry_hash`), so it is exactly as canonical as the mesh: a rebuild of the same cells
    // gives the same key and structure; the cell index is part of the mesh identity (tallies are
    // indexed by it), so swapping two cells changes the key.
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0]];
    let loops = vec![vec![0, 3, 2, 1], vec![4, 5, 6, 7], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]];
    let mut pts2 = pts.to_vec();
    pts2.extend(pts.iter().map(|p| [p[0] + 3.0, p[1], p[2]]));
    let shifted: Vec<Vec<usize>> = loops.iter().map(|l| l.iter().map(|v| v + 8).collect()).collect();
    let key_of = |cells: &[CellSpec]| {
        let mesh = build_mesh(&pts2, cells, &mut SerialAlloc::new()).unwrap();
        let (rules, comp, spec) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
        let input = BuildInput::Mesh(MeshInput::new(&mesh, &rules, &spec, &comp));
        let key = MeshTrackerBuilder.cache_key(&input, &BuildOptions::default()).unwrap();
        let (t, _) = MeshTrackerBuilder.build(&input, &BuildOptions::default()).unwrap();
        (key, t.structure_hash())
    };
    let ab = [CellSpec::Poly(loops.clone()), CellSpec::Poly(shifted.clone())];
    let ba = [CellSpec::Poly(shifted), CellSpec::Poly(loops)];
    assert_eq!(key_of(&ab), key_of(&ab));
    assert_ne!(key_of(&ab).0, key_of(&ba).0);
}
