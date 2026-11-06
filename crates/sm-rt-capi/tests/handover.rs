//! W3b-RT-CAPI acceptance on the real mesh tracker: the handover file written by sm-rt is opened
//! through `smrt_open`; a C program reads node sets and region names, assigns its own sigma per
//! name and flies 1e4 surface and delta histories whose tallies equal the Rust path bit for bit
//! (acceptance 1); contract violations return typed codes (2, 4); surface areas and normals
//! (3); determinism (6).
#![cfg(feature = "sm-rt")]

mod cutil;
#[path = "common/flight_ref.rs"]
mod flight_ref;

use cutil::*;
use sm_core::Point3;
use sm_mesh_api::{FaceIx, MeshRead};
use sm_rt::{MeshTracker, MeshTrackerBuilder, write_smrt_file};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RegionRule, Scratch, SurfaceId, TraceOptions, Tracker,
};
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;
use sm_testkit::meshes::hex_block_two_volumes;
use std::ffi::CString;
use std::path::{Path, PathBuf};

const EXT: [f64; 3] = [6.0, 3.0, 3.0];
/// The interface sits at x = 2, off the kd split at the median, so leaves straddle it and delta
/// flights see two regions (and reject).
const SPLIT: u32 = 2;

fn brick_tracker(partition: bool) -> MeshTracker {
    let fx = hex_block_two_volumes([6, 3, 3], [1.0; 3], SPLIT).expect("brick");
    let mut opts = BuildOptions::default();
    if partition {
        let mut p = PartitionOptions::default();
        p.enabled = true;
        p.leaf_cells = 20;
        opts = opts.with_partition(p);
    }
    let (rule, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(&fx.mesh, &rule, &bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &opts).expect("build").0
}

fn write_image(dir: &Path, t: &MeshTracker, name: &str) -> PathBuf {
    let path = dir.join(name);
    write_smrt_file(&path, &t.export_cache().expect("cache image")).expect("write .smrt");
    path
}

fn open(path: &Path) -> *mut SmrtTracker {
    let c = CString::new(path.to_str().expect("utf8")).expect("no NUL");
    let mut h = std::ptr::null_mut();
    // SAFETY: `c` is NUL-terminated and `h` is a valid out slot.
    assert_eq!(unsafe { smrt_open(c.as_ptr(), &mut h) }, 0, "{}", last_error());
    assert!(!h.is_null());
    h
}

/// The CASE lines of a C program's output.
fn case_lines(out: &str) -> Vec<String> {
    out.lines().filter(|l| l.starts_with("CASE ")).map(str::to_owned).collect()
}

#[test]
fn c_handover_flights_equal_the_rust_path_bit_for_bit() {
    let t = brick_tracker(true);
    let dir = scratch("handover-brick");
    let file = write_image(&dir, &t, "brick.smrt");
    let corrupt = file.with_file_name("corrupt.smrt");
    let mut bytes = std::fs::read(&file).expect("read");
    let n = bytes.len() / 2;
    bytes.truncate(n);
    std::fs::write(&corrupt, bytes).expect("write corrupt");

    let out = build_and_run_c_args(&crate_dir().join("tests/c/handover.c"), "handover", &[file.to_str().expect("utf8"), corrupt.to_str().expect("utf8")]);
    assert!(out.contains("handover: ok"), "{out}");
    // Region names reach the C code: the mesh has two named volumes plus the complement.
    assert!(out.lines().filter(|l| l.starts_with("REGION ")).count() >= 2, "{out}");

    let lines = case_lines(&out);
    assert_eq!(lines.len(), 3, "{out}");
    let want = [
        flight_ref::run_case("surface", &t, 0, 10_000, 0x1111),
        flight_ref::run_case("delta", &t, 1, 10_000, 0x2222),
        flight_ref::run_case("delta_block", &t, 2, 10_000, 0x3333),
    ];
    for (got, want) in lines.iter().zip(&want) {
        assert_eq!(got, want);
    }
    // The runs are not vacuous: histories flew, delta flights rejected and collided.
    assert!(want[0].contains("hist=10000") && !want[0].contains("seg=0 "), "{}", want[0]);
    assert!(!want[1].contains("coll=0 ") && !want[1].contains("rej=0 "), "{}", want[1]);
    assert!(!want[2].contains("coll=0 "), "{}", want[2]);
}

fn last_error() -> String {
    let mut buf = vec![0u8; 512];
    // SAFETY: `buf` is valid for 512 bytes.
    let n = unsafe { smrt_last_error_message(buf.as_mut_ptr().cast(), 512) };
    String::from_utf8_lossy(&buf[..(n.max(0) as usize).min(511)]).into_owned()
}

#[test]
fn opened_tracker_is_the_built_tracker_and_has_names_sets_and_nodes() {
    let t = brick_tracker(true);
    let dir = scratch("handover-names");
    let h = open(&write_image(&dir, &t, "names.smrt"));
    let mut info = SmrtInfo::default();
    // SAFETY: live handle and out slot.
    assert_eq!(unsafe { smrt_info(h, &mut info) }, 0);
    let want = t.info().structure_hash.0;
    assert_eq!(((u128::from(info.structure_hash_hi)) << 64) | u128::from(info.structure_hash_lo), want, "same structure hash as the built tracker");
    let names: Vec<String> = t.regions().0.iter().map(|r| r.name.clone()).collect();
    for (r, want) in names.iter().enumerate() {
        let mut buf = [0u8; 64];
        // SAFETY: live handle, `buf` valid for 64 bytes.
        let n = unsafe { smrt_region_name(h, r as i64, buf.as_mut_ptr().cast(), 64) };
        assert_eq!(&buf[..n as usize], want.as_bytes());
        assert_eq!(buf[n as usize], 0);
    }
    let (mut nn, mut ns) = (0i64, 0i64);
    // SAFETY: live handle and out slots.
    unsafe {
        assert_eq!(smrt_n_nodes(h, &mut nn), 0);
        assert_eq!(smrt_n_sets(h, &mut ns), 0);
    }
    let hv = t.majorant_view().expect("partition").hierarchy();
    assert_eq!((nn as usize, ns as usize), (hv.node_count(), hv.set_count()));
    for node in 0..nn {
        let mut set = -1i64;
        let mut ni = SmrtNodeInfo::default();
        // SAFETY: live handle and out slots.
        unsafe {
            assert_eq!(smrt_node_set(h, node, &mut set), 0);
            assert_eq!(smrt_node_info(h, node, &mut ni), 0);
        }
        let want = hv.node(sm_rt_api::NodeId::new(node as usize)).expect("node");
        assert_eq!(set, i64::from(want.set.0));
        assert_eq!(ni.set, set);
        assert_eq!(ni.cells, i64::from(want.cells));
        assert_eq!(ni.bounds_lo, [want.bounds.lo.x, want.bounds.lo.y, want.bounds.lo.z]);
    }
    for s in 0..ns {
        let (mut ids, mut n) = ([0i64; 8], 0i64);
        // SAFETY: live handle; `ids` holds 8 entries.
        assert_eq!(unsafe { smrt_set_regions(h, s, ids.as_mut_ptr(), 8, &mut n) }, 0);
        let want = hv.set_regions(sm_rt_api::MaterialSetId(s as u32)).expect("set");
        assert_eq!(n as usize, want.len());
        let raw: Vec<i64> = ids[..want.len()].iter().map(|&i| if i == SMRT_COMPLEMENT { i64::from(t.regions().implicit_complement().expect("complement").0) } else { i }).collect();
        assert_eq!(raw, want.iter().map(|r| i64::from(r.0)).collect::<Vec<_>>());
    }
    // SAFETY: freed once.
    assert_eq!(unsafe { smrt_tracker_free(h) }, 0);
}

#[test]
fn a_tracker_without_a_partition_has_no_nodes_and_no_flights() {
    let t = brick_tracker(false);
    let dir = scratch("handover-plain");
    let h = open(&write_image(&dir, &t, "plain.smrt"));
    let mut n = 0i64;
    let st = SmrtState::default();
    let mut f = std::ptr::null_mut();
    // SAFETY: live handle and valid arguments.
    unsafe {
        assert_eq!(smrt_n_nodes(h, &mut n), SMRT_ERR_UNSUPPORTED);
        assert_eq!(smrt_n_sets(h, &mut n), SMRT_ERR_UNSUPPORTED);
        // The state is checked first (a zero direction is no direction), then the view.
        assert_ne!(smrt_flight_new(h, &st, std::ptr::null(), &mut f), 0);
        assert!(f.is_null());
        let mut ctx = std::ptr::null_mut();
        assert_eq!(smrt_ctx_new(h, &mut ctx), 0);
        assert_eq!(smrt_init_state(ctx, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, -1), 0);
        let mut s = SmrtState::default();
        assert_eq!(smrt_get_state(ctx, &mut s), 0);
        assert_eq!(smrt_flight_new(h, &s, std::ptr::null(), &mut f), SMRT_ERR_UNSUPPORTED);
        assert!(f.is_null());
        assert_eq!(smrt_ctx_free(ctx), 0);
        assert_eq!(smrt_tracker_free(h), 0);
    }
}

fn surface_count(t: &MeshTracker) -> usize {
    t.surfaces().0.len()
}

#[test]
fn measure_area_and_get_angle_equal_the_analytic_face_values() {
    let fx = hex_block_two_volumes([6, 3, 3], [1.0; 3], SPLIT).expect("brick");
    let t = brick_tracker(false);
    let dir = scratch("handover-faces");
    let h = open(&write_image(&dir, &t, "faces.smrt"));
    let co = fx.mesh.coords();
    assert!(surface_count(&t) >= 5, "{} surfaces", surface_count(&t));
    let mut total_boundary = 0.0;
    for s in 0..surface_count(&t) {
        let sid = SurfaceId::new(s);
        // Faces of the surface through the ABI equal the contract's list.
        let want_faces = t.surface_faces(sid);
        let (mut buf, mut nf) = (vec![0i64; want_faces.len().max(1)], 0i64);
        // SAFETY: live handle; `buf` has room for the faces.
        assert_eq!(unsafe { smrt_dag_surface_faces(h, s as i64, buf.as_mut_ptr(), buf.len() as i64, &mut nf) }, 0);
        assert_eq!(nf as usize, want_faces.len());
        assert_eq!(&buf[..want_faces.len()], want_faces.iter().map(|f| i64::from(f.0)).collect::<Vec<_>>().as_slice());
        assert!(!want_faces.is_empty());

        // Analytic values from the mesh vertices (Newell's vector area of each planar polygon).
        let (mut analytic, mut nsum) = (0.0, [0.0f64; 3]);
        for f in want_faces {
            let vs = fx.mesh.face_vertices(FaceIx::new(f.get()));
            let pt = |i: usize| [co.x[vs[i].get()], co.y[vs[i].get()], co.z[vs[i].get()]];
            let mut a = [0.0; 3];
            for i in 0..vs.len() {
                let (p, q) = (pt(i), pt((i + 1) % vs.len()));
                a[0] += (p[1] - q[1]) * (p[2] + q[2]);
                a[1] += (p[2] - q[2]) * (p[0] + q[0]);
                a[2] += (p[0] - q[0]) * (p[1] + q[1]);
            }
            let m = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt() * 0.5;
            analytic += m;
            nsum = [a[0] * 0.5, a[1] * 0.5, a[2] * 0.5];
        }
        let norm = (nsum[0] * nsum[0] + nsum[1] * nsum[1] + nsum[2] * nsum[2]).sqrt();
        let unit = [nsum[0] / norm, nsum[1] / norm, nsum[2] / norm];

        let mut area = 0.0;
        let mut n = [0.0; 3];
        let p = [0.0, 0.0, 0.0];
        // SAFETY: live handle; arrays of the documented sizes.
        unsafe {
            assert_eq!(smrt_dag_measure_area(h, s as i64, &mut area), 0);
            assert_eq!(smrt_dag_get_angle_surface(h, s as i64, p.as_ptr(), n.as_mut_ptr()), 0);
        }
        assert!((area - analytic).abs() < 1e-12, "surface {s}: area {area} vs {analytic}");
        // The normal is the face normal up to orientation (checked in the next test).
        let dot = n[0] * unit[0] + n[1] * unit[1] + n[2] * unit[2];
        assert!((dot.abs() - 1.0).abs() < 1e-12, "surface {s}: normal {n:?} vs {unit:?}");
        // And it equals the contract's face normal of that face (bit for bit).
        let fid = want_faces[0];
        let want_n = t.face_normal(fid, Point3::new(p[0], p[1], p[2])).expect("face normal");
        assert_eq!(n, [want_n.x, want_n.y, want_n.z]);
        let mut n2 = [0.0; 3];
        // SAFETY: as above.
        assert_eq!(unsafe { smrt_dag_get_angle(h, i64::from(fid.0), p.as_ptr(), n2.as_mut_ptr()) }, 0);
        assert_eq!(n, n2);
        if t.surfaces().get(sid).expect("surface").back.is_none() {
            total_boundary += area;
        }
    }
    // The classified outer surfaces are part of the 90-unit skin (some skin faces are unclassified).
    assert!(total_boundary > 0.0 && total_boundary <= fx.area + 1e-9, "{total_boundary} of {}", fx.area);
    // Unknown surface; null output.
    let mut a = 0.0;
    // SAFETY: live handle.
    unsafe {
        assert_eq!(smrt_dag_measure_area(h, 1_000_000, &mut a), SMRT_ERR_UNKNOWN_ID);
        assert_eq!(smrt_dag_measure_area(h, 0, std::ptr::null_mut()), SMRT_ERR_NULL);
        assert_eq!(smrt_dag_get_angle_surface(h, 0, std::ptr::null(), std::ptr::null_mut()), SMRT_ERR_NULL);
        assert_eq!(smrt_tracker_free(h), 0);
    }
}

#[test]
fn surface_normals_point_out_of_the_front_region() {
    // Orientation through the ABI: stepping from a point on a face against the normal enters the
    // surface's front region; stepping along it enters the back region (or leaves the model).
    let fx = hex_block_two_volumes([6, 3, 3], [1.0; 3], SPLIT).expect("brick");
    let t = brick_tracker(false);
    let dir = scratch("handover-orient");
    let h = open(&write_image(&dir, &t, "orient.smrt"));
    let mesh = &fx.mesh;
    let co = mesh.coords();
    let mut sc = Scratch::new();
    for s in 0..surface_count(&t) {
        let info = t.surfaces().get(SurfaceId::new(s)).expect("surface").clone();
        let fid = t.surface_faces(SurfaceId::new(s))[0];
        let vs = mesh.face_vertices(FaceIx::new(fid.get()));
        let mut c = [0.0; 3];
        for v in vs {
            c[0] += co.x[v.get()] / vs.len() as f64;
            c[1] += co.y[v.get()] / vs.len() as f64;
            c[2] += co.z[v.get()] / vs.len() as f64;
        }
        let mut n = [0.0; 3];
        // SAFETY: live handle; arrays of the documented sizes.
        assert_eq!(unsafe { smrt_dag_get_angle(h, i64::from(fid.0), c.as_ptr(), n.as_mut_ptr()) }, 0);
        let at = |sgn: f64| Point3::new(c[0] + sgn * 1e-3 * n[0], c[1] + sgn * 1e-3 * n[1], c[2] + sgn * 1e-3 * n[2]);
        let behind = t.locate(at(-1.0), None, None, &mut sc).expect("locate").region;
        let ahead = t.locate(at(1.0), None, None, &mut sc).expect("locate").region;
        assert_eq!(behind, info.front, "surface {s}: against the normal is the front region");
        let comp = t.regions().implicit_complement();
        let ahead_ok = ahead == info.back || (info.back.is_none() && (ahead.is_none() || Some(ahead) == comp));
        assert!(ahead_ok, "surface {s}: along the normal is {ahead:?}, back is {:?}", info.back);
    }
    // SAFETY: freed once.
    assert_eq!(unsafe { smrt_tracker_free(h) }, 0);
}

fn words(h: *const SmrtTracker, threads: usize) -> Vec<u64> {
    let n = 2000usize;
    let mut cols: [Vec<f64>; 6] = Default::default();
    let mut rng = flight_ref::Sm(0xB0B);
    for _ in 0..n {
        let o = [rng.unit01() * EXT[0], rng.unit01() * EXT[1], rng.unit01() * EXT[2]];
        let d = [0.6, 0.0, 0.8];
        for k in 0..3 {
            cols[k].push(o[k]);
            cols[3 + k].push(d[k]);
        }
    }
    let hp = h as usize;
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    let cols = &cols;
    pool.install(move || {
        let r = SmrtRays { x: cols[0].as_ptr(), y: cols[1].as_ptr(), z: cols[2].as_ptr(), u: cols[3].as_ptr(), v: cols[4].as_ptr(), w: cols[5].as_ptr(), start_cell: std::ptr::null() };
        let mut len = 0u64;
        let h = hp as *const SmrtTracker;
        // SAFETY: arrays hold `n` entries; the null buffer with capacity 0 is the size query.
        let q = unsafe { smrt_trace_bundle(h, n as i64, &r, 64, std::ptr::null(), std::ptr::null_mut(), 0, &mut len) };
        assert_eq!(q, SMRT_ERR_BAD_ARG);
        let mut buf = vec![0u64; len as usize];
        let mut len2 = 0u64;
        // SAFETY: `buf` holds `len` words.
        assert_eq!(unsafe { smrt_trace_bundle(h, n as i64, &r, 64, std::ptr::null(), buf.as_mut_ptr(), len, &mut len2) }, 0);
        buf
    })
}

#[test]
fn determinism_runs_and_thread_counts_and_c_output() {
    let t = brick_tracker(true);
    let dir = scratch("handover-det");
    let file = write_image(&dir, &t, "det.smrt");
    let h = open(&file);
    let one = words(h, 1);
    let many = words(h, 4);
    assert_eq!(one, many, "bundle words, 1 vs N threads");
    assert_eq!(one, words(h, 1), "and across runs");
    // The same bundle from the in-process tracker.
    let mut rng = flight_ref::Sm(0xB0B);
    let rays: Vec<_> = (0..2000)
        .map(|_| {
            let o = [rng.unit01() * EXT[0], rng.unit01() * EXT[1], rng.unit01() * EXT[2]];
            sm_geom_api::Ray { origin: Point3::new(o[0], o[1], o[2]), dir: sm_core::Vector3::new(0.6, 0.0, 0.8) }
        })
        .collect();
    let b = t.trace_bundle(&rays, &TraceOptions::default()).expect("bundle");
    assert_eq!(one, sm_rt_capi::bundle::encode(&b), "opened tracker traces like the built one");
    // SAFETY: freed once.
    assert_eq!(unsafe { smrt_tracker_free(h) }, 0);

    let corrupt = file.with_file_name("corrupt_det.smrt");
    std::fs::write(&corrupt, b"not a tracking image").expect("write");
    let run = || case_lines(&build_and_run_c_args(&crate_dir().join("tests/c/handover.c"), "handover_det", &[file.to_str().expect("utf8"), corrupt.to_str().expect("utf8")]));
    assert_eq!(run(), run(), "the C program prints the same bits on every run");
}
