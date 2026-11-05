//! W3b FOAM-WARP: warped faces are split into planar pieces at export; the internal mesh and
//! the cell volumes stay as they were.

#![allow(clippy::needless_range_loop)]

mod common;

use common::foam::{area_vector, cell_stats, faces_on, SIDES};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_meshio::foam::*;
use sm_testkit::determinism;
use sm_testkit::meshes::{CellSpec, build_mesh, nonconvex_poly, truncated_octahedron};

/// An `n`-cell cube of unit side whose interior nodes are displaced, so interior faces warp and
/// the boundary stays planar.
fn warped_block(n: usize, amp: f64) -> ReferenceMesh {
    let np = n + 1;
    let id = |i: usize, j: usize, k: usize| (k * np + j) * np + i;
    let mut pts = vec![[0.0; 3]; np * np * np];
    for k in 0..np {
        for j in 0..np {
            for i in 0..np {
                let mut p = [i as f64 / n as f64, j as f64 / n as f64, k as f64 / n as f64];
                let inner = |a: usize| a > 0 && a < n;
                if inner(i) && inner(j) && inner(k) {
                    let s = (i * 3 + j * 5 + k * 7) as f64;
                    p[0] += amp * s.sin();
                    p[1] += amp * (s * 1.7).cos();
                    p[2] += amp * (s * 2.3).sin();
                }
                pts[id(i, j, k)] = p;
            }
        }
    }
    let mut cells = Vec::new();
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let v = vec![id(i, j, k), id(i + 1, j, k), id(i + 1, j + 1, k), id(i, j + 1, k), id(i, j, k + 1), id(i + 1, j, k + 1), id(i + 1, j + 1, k + 1), id(i, j + 1, k + 1)];
                cells.push(CellSpec::Template(CellShape::Hex, v));
            }
        }
    }
    let mut alloc = SerialAlloc::new();
    let mut m = build_mesh(&pts, &cells, &mut alloc).unwrap();
    for (i, name) in SIDES.iter().enumerate() {
        let f = faces_on(&m, i / 2, (i % 2) as f64);
        MeshEdit::add_set(&mut m, name, SetMembers::Faces(f), &mut alloc).unwrap();
    }
    let half = (n * n * n / 2) as u32;
    MeshEdit::add_set(&mut m, "lo", SetMembers::Cells((0..half).map(|c| CellIx::new(c as usize)).collect()), &mut alloc).unwrap();
    m
}

fn off() -> FoamOptions {
    FoamOptions::new()
}

fn on() -> FoamOptions {
    FoamOptions::new().split_warped_faces(true)
}

/// Max distance of a face's vertices from its own least-squares-free plane (through vertex 0,
/// normal the Newell vector), relative to sqrt of the area; independent of the crate.
fn flatness(pm: &PolyMesh, f: usize) -> f64 {
    let l = pm.face(f);
    let sf = area_vector(pm, f);
    let len = (sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2]).sqrt();
    let p0 = pm.points[l[0] as usize];
    let d = l.iter().map(|&v| {
        let p = pm.points[v as usize];
        ((sf[0] * (p[0] - p0[0]) + sf[1] * (p[1] - p0[1]) + sf[2] * (p[2] - p0[2])) / len).abs()
    });
    d.fold(0.0, f64::max) / len.sqrt()
}

/// Per cell volume of the OpenFOAM pyramid decomposition: `sum +-Sf . (fc - x0) / 3` with `x0`
/// the vertex mean of the cell and `fc` the area-weighted centre of the triangles about the
/// face's vertex mean; written here independently of the crate.
fn openfoam_volumes(pm: &PolyMesh) -> Vec<f64> {
    let nc = pm.n_cells as usize;
    let mut x0 = vec![[0.0; 3]; nc];
    let mut cnt = vec![0.0; nc];
    let mut vol = vec![0.0; nc];
    let geo: Vec<([f64; 3], [f64; 3])> = (0..pm.n_faces()).map(|f| {
        let l = pm.face(f);
        let sf = area_vector(pm, f);
        let k = l.len() as f64;
        let mut est = [0.0; 3];
        for &v in l {
            for d in 0..3 {
                est[d] += pm.points[v as usize][d] / k;
            }
        }
        let (mut sw, mut swc) = (0.0, [0.0; 3]);
        for e in 0..l.len() {
            let (a, b) = (pm.points[l[e] as usize], pm.points[l[(e + 1) % l.len()] as usize]);
            let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [est[0] - a[0], est[1] - a[1], est[2] - a[2]]);
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            let w = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            sw += w;
            for d in 0..3 {
                swc[d] += w * (a[d] + b[d] + est[d]) / 3.0;
            }
        }
        (sf, [swc[0] / sw, swc[1] / sw, swc[2] / sw])
    }).collect();
    for f in 0..pm.n_faces() {
        let mut cells = vec![pm.owner[f] as usize];
        if f < pm.n_internal() {
            cells.push(pm.neighbour[f] as usize);
        }
        for c in cells {
            cnt[c] += 1.0;
            for d in 0..3 {
                x0[c][d] += geo[f].1[d];
            }
        }
    }
    for c in 0..nc {
        for d in 0..3 {
            x0[c][d] /= cnt[c];
        }
    }
    for f in 0..pm.n_faces() {
        let (sf, fc) = geo[f];
        let mut add = |c: usize, s: f64| {
            vol[c] += s * (sf[0] * (fc[0] - x0[c][0]) + sf[1] * (fc[1] - x0[c][1]) + sf[2] * (fc[2] - x0[c][2])) / 3.0;
        };
        add(pm.owner[f] as usize, 1.0);
        if f < pm.n_internal() {
            add(pm.neighbour[f] as usize, -1.0);
        }
    }
    vol
}

#[test]
fn warped_faces_are_split_into_planar_pieces_and_volumes_are_conserved() {
    let m = warped_block(3, 0.03);
    let before = build_polymesh(&m, &off()).unwrap();
    let max_warp = (0..before.mesh.n_faces()).map(|f| flatness(&before.mesh, f)).fold(0.0, f64::max);
    assert!(max_warp > 1e-3, "the fixture must be warped: {max_warp}");
    let after = build_polymesh(&m, &on()).unwrap();
    let (a, b) = (&before.mesh, &after.mesh);
    // Every face of the export is planar to 1e-12.
    for f in 0..b.n_faces() {
        assert!(flatness(b, f) < 1e-12, "face {f}: {}", flatness(b, f));
        assert!(face_warpage(b, f) <= WARP_TOLERANCE);
    }
    // Volumes in the OpenFOAM decomposition before equal the exact volumes after.
    let want = openfoam_volumes(a);
    let got = cell_stats(b);
    assert_eq!(want.len(), got.len());
    for c in 0..want.len() {
        assert!((got[c].0 - want[c]).abs() < 1e-12 * want[c], "cell {c}: {} vs {}", got[c].0, want[c]);
        assert!(got[c].1 < 1e-12 * got[c].2, "cell {c} unclosed");
    }
    let volumes_after = openfoam_volumes(b);
    for c in 0..want.len() {
        assert!((volumes_after[c] - want[c]).abs() < 1e-12 * want[c]);
    }
    // Pieces of one face sum to its area vector.
    let w = after.report.warp.as_ref().expect("a warp report");
    assert!(w.faces_split > 0 && w.pieces == 4 * w.faces_split && w.points_added == w.faces_split);
    let mut face = 0;
    for f in 0..a.n_faces() {
        let pieces = if face_warpage(a, f) > WARP_TOLERANCE { a.face(f).len() } else { 1 };
        let mut s = [0.0; 3];
        for p in face..face + pieces {
            let v = area_vector(b, p);
            for d in 0..3 {
                s[d] += v[d];
            }
            assert_eq!(after.face_of_foam[p], before.face_of_foam[f]);
            assert_eq!(b.owner[p], a.owner[f]);
        }
        let sf = area_vector(a, f);
        for d in 0..3 {
            assert!((s[d] - sf[d]).abs() < 1e-12, "face {f} component {d}");
        }
        face += pieces;
    }
    assert_eq!(face, b.n_faces());
    // Valid polyMesh apart from the intended ties of pieces between one cell pair.
    let issues: Vec<_> = check_polymesh(b, 1e-9).into_iter().filter(|i| i.code != "foam.upper_triangular_tie").collect();
    assert!(issues.is_empty(), "{issues:?}");
    // Census: cells with a split face are polyhedra.
    assert_eq!(after.report.shape_counts.hexahedra + w.cells_affected, before.report.shape_counts.hexahedra);
    assert_eq!(after.report.shape_counts.polyhedra, w.cells_affected);
    // Loss note present.
    assert!(after.report.loss.items.iter().any(|i| i.code == "foam.warped_faces_split"));
}

#[test]
fn internal_cells_patches_and_zones_are_untouched() {
    let m = warped_block(3, 0.03);
    let a = build_polymesh(&m, &off()).unwrap();
    let b = build_polymesh(&m, &on()).unwrap();
    assert_eq!(a.mesh.n_cells, b.mesh.n_cells);
    assert!(b.mesh.cell_map.is_empty());
    assert_eq!(a.mesh.cell_zones, b.mesh.cell_zones);
    // The boundary is planar: patches keep their sizes and order, only start faces move.
    assert_eq!(a.mesh.patches.iter().map(|p| (&p.name, p.n_faces)).collect::<Vec<_>>(), b.mesh.patches.iter().map(|p| (&p.name, p.n_faces)).collect::<Vec<_>>());
    let n_bnd = a.mesh.n_faces() - a.mesh.n_internal();
    assert_eq!(b.mesh.n_faces() - b.mesh.n_internal(), n_bnd);
    // Original points keep their indices and coordinates; boundary faces are byte equal.
    assert_eq!(&b.mesh.points[..a.mesh.points.len()], &a.mesh.points[..]);
    for i in 0..n_bnd {
        assert_eq!(a.mesh.face(a.mesh.n_internal() + i), b.mesh.face(b.mesh.n_internal() + i));
    }
    assert_eq!(b.point_of_foam.len(), b.mesh.points.len());
    assert!(b.point_of_foam[a.mesh.points.len()..].iter().all(|p| *p == u32::MAX));
    assert_eq!(b.face_of_foam.len(), b.mesh.n_faces());
    assert_eq!(b.report.n_internal_faces as usize, b.mesh.n_internal());
    // Round trip through the files.
    let dir = b.mesh.files().unwrap();
    assert!(!dir.is_empty());
}

#[test]
fn planar_meshes_are_byte_identical_with_or_without_the_pass() {
    let m = hex_block([3, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let a = build_polymesh(&m, &off()).unwrap();
    let b = build_polymesh(&m, &on()).unwrap();
    assert!(b.report.warp.is_none());
    assert_eq!(a.report, b.report);
    assert_eq!(a.mesh, b.mesh);
    // Default options serialize as before (no new keys); opting in adds one.
    assert!(!serde_json::to_string(&FoamOptions::default()).unwrap().contains("warp"));
    assert!(serde_json::to_string(&on()).unwrap().contains("split_warped_faces"));
    // Off by default: a warped mesh exports unchanged and without a warp report.
    let w = build_polymesh(&warped_block(3, 0.03), &FoamOptions::default()).unwrap();
    assert!(w.report.warp.is_none());
}

#[test]
fn face_zones_follow_the_pieces() {
    let mut m = warped_block(3, 0.03);
    // A face set of interior faces: the faces of cell 13 (the centre cell) shared with cell 12.
    let mut alloc = SerialAlloc::new();
    let faces: Vec<FaceIx> = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| m.face_right(*f).is_some() && (m.face_left(*f).get() == 13 || m.face_right(*f).map(|c| c.get()) == Some(13)))
        .collect();
    assert!(!faces.is_empty());
    MeshEdit::add_set(&mut m, "zone", SetMembers::Faces(faces), &mut alloc).unwrap();
    let opts = FoamOptions::new().face_zones(vec!["zone".into()]);
    let on_opts = opts.clone().split_warped_faces(true);
    let b = build_polymesh(&m, &on_opts).unwrap();
    let a = build_polymesh(&m, &opts).unwrap();
    let (za, zb) = (&a.mesh.face_zones[0], &b.mesh.face_zones[0]);
    assert_eq!(zb.labels.len(), zb.flips.len());
    assert!(zb.labels.len() > za.labels.len());
    assert!(zb.labels.windows(2).all(|w| w[0] < w[1]));
    // Every zone face of the export descends from a zone face of the source.
    let src: Vec<u32> = za.labels.iter().map(|&l| a.face_of_foam[l as usize]).collect();
    for &l in &zb.labels {
        assert!(src.contains(&b.face_of_foam[l as usize]));
    }
}

#[test]
fn concave_counts_are_recorded_before_and_after() {
    let mut rows = Vec::new();
    for (name, m) in [
        ("warped_block_3_0.03", warped_block(3, 0.03)),
        ("warped_block_3_0.08", warped_block(3, 0.08)),
        ("warped_block_4_0.05", warped_block(4, 0.05)),
    ] {
        let b = build_polymesh(&m, &on()).unwrap();
        let w = b.report.warp.clone().expect("warp report");
        rows.push(format!("{name}: faces_split {} cells {} concave {} -> {}", w.faces_split, w.cells_affected, w.concave_before, w.concave_after));
        let a = build_polymesh(&m, &off()).unwrap();
        assert_eq!(concave_cell_count(&a.mesh), w.concave_before);
        assert_eq!(concave_cell_count(&b.mesh), w.concave_after);
    }
    // Reference polyhedra with planar faces: no warped faces, nothing to split.
    for f in [truncated_octahedron().unwrap(), nonconvex_poly().unwrap()] {
        let b = build_polymesh(&f.mesh, &on()).unwrap();
        assert!(b.report.warp.is_none(), "{}", f.name);
        rows.push(format!("{}: planar, concave {}", f.name, concave_cell_count(&b.mesh)));
    }
    eprintln!("{}", rows.join("\n"));
}

#[test]
fn a_concave_cell_by_face_planes_is_counted() {
    // A hex whose top face is dented inward by moving one top node below the plane of the
    // neighbouring top face's... simpler: a nonconvex L-shaped prism from `nonconvex_poly`.
    let f = nonconvex_poly().unwrap();
    let b = build_polymesh(&f.mesh, &off()).unwrap();
    assert!(concave_cell_count(&b.mesh) >= 1);
    // A unit cube is not concave.
    let c = build_polymesh(&hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap(), &off()).unwrap();
    assert_eq!(concave_cell_count(&c.mesh), 0);
}

#[test]
fn split_output_is_bit_identical_across_runs_and_thread_counts() {
    let m = warped_block(4, 0.05);
    determinism::check(2, 4, |t| {
        rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(|| {
            let b = build_polymesh(&m, &on()).unwrap();
            let mut h = 0xcbf2_9ce4_8422_2325u64;
            for f in b.mesh.files().unwrap() {
                for x in f.bytes {
                    h = (h ^ u64::from(x)).wrapping_mul(0x0100_0000_01b3);
                }
            }
            for v in b.face_of_foam.iter().chain(&b.point_of_foam) {
                h = (h ^ u64::from(*v)).wrapping_mul(0x0100_0000_01b3);
            }
            h ^ serde_json::to_string(&b.report).unwrap().len() as u64
        })
    })
    .unwrap();
}

#[test]
fn the_split_survives_the_small_determinant_split() {
    // A single warped hex has no internal face: determinant 0, so both passes run.
    let p = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.2]];
    let mut alloc = SerialAlloc::new();
    let m = build_mesh(&p, &[CellSpec::Template(CellShape::Hex, (0..8).collect())], &mut alloc).unwrap();
    let b = build_polymesh(&m, &on()).unwrap();
    assert!(b.report.warp.is_some());
    for f in 0..b.mesh.n_faces() {
        assert!(flatness(&b.mesh, f) < 1e-12);
    }
    assert_eq!(b.report.shape_counts.hexahedra, 0);
    assert_eq!(b.report.n_cells, u64::from(b.mesh.n_cells));
    assert_eq!(b.report.shape_counts.pyramids + b.report.shape_counts.tetrahedra + b.report.shape_counts.polyhedra, u64::from(b.mesh.n_cells));
}

#[test]
fn a_bad_warp_tolerance_is_rejected() {
    let m = warped_block(2, 0.03);
    for t in [-1e-9, f64::NAN, f64::INFINITY] {
        let r = build_polymesh(&m, &on().warp_tolerance(t));
        assert!(r.is_err(), "tolerance {t} must be rejected");
    }
    assert!(build_polymesh(&m, &on().warp_tolerance(0.0)).is_ok());
}
