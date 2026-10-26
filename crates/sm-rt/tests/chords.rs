//! Acceptances 1-5: chord sums, non-convex cells, shared-face consistency, Cauchy.
mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, MeshRead, SerialAlloc};
use sm_rt_api::{CellId, FaceId, Scratch, StepEnd, Tracker};
use sm_testkit::meshes::{MeshFixture, dual_patch, hexagonal_prism, mixed_transition, nonconvex_poly, truncated_octahedron};





fn iur_fixture(fx: &MeshFixture) {
    let t = build_mesh_tracker(&fx.mesh, &vacuum(), &opts_fail());
    let polys: Vec<Convex> = (0..fx.mesh.cell_count()).map(|c| cell_polytope(&fx.mesh, c)).collect();
    let (lo, hi) = bbox(&fx.mesh);
    let center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
    let rho = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt() / 2.0 * 1.0001;
    let runs = |o: [f64; 3], d: [f64; 3]| merge_runs(polys.iter().filter_map(|p| p.clip(o, d)).collect());
    let st = run_iur(&t, fx.name, n_lines(), center, rho, &runs);
    assert_eq!(st.lost, 0, "{}: lost particles", fx.name);
    assert!(st.max_err <= 1e-12, "{}: chord-sum error {}", fx.name, st.max_err);
    assert_cell_means(&st, &fx.cell_volumes, rho, 4.0);
}

#[test]
fn acc01_hex_block_iur_chords() {
    let mesh = hex_block([10; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
    let hull = Convex::boxed([0.0; 3], [10.0; 3]);
    let rho = (3.0f64).sqrt() * 5.0;
    let runs = |o: [f64; 3], d: [f64; 3]| hull.clip(o, d).into_iter().filter(|(a, b)| b > a).collect::<Vec<_>>();
    let st = run_iur(&t, "hex_block_10", n_lines(), [5.0; 3], rho, &runs);
    assert_eq!(st.lost, 0, "lost particles");
    assert!(st.max_err <= 1e-12, "max chord-sum error {}", st.max_err);
    assert_cell_means(&st, &vec![1.0; mesh.cell_count()], rho, 4.0);
}

#[test]
fn acc02_truncated_octahedron_chords() {
    iur_fixture(&truncated_octahedron().unwrap());
}
#[test]
fn acc02_hexagonal_prism_chords() {
    iur_fixture(&hexagonal_prism(1.0, 2.0).unwrap());
}
#[test]
fn acc02_mixed_transition_chords() {
    iur_fixture(&mixed_transition().unwrap());
}
#[test]
fn acc02_dual_patch_chords() {
    iur_fixture(&dual_patch().unwrap());
}

/// The L cell of `nonconvex_poly` as the union of two boxes.
fn l_cell_intervals(o: [f64; 3], d: [f64; 3]) -> Vec<(f64, f64)> {
    let a = Convex::boxed([0.0, 0.0, 0.0], [2.0, 1.0, 1.0]);
    let b = Convex::boxed([0.0, 1.0, 0.0], [1.0, 2.0, 1.0]);
    merge_runs([a.clip(o, d), b.clip(o, d)].into_iter().flatten().collect())
}

#[test]
fn acc03_nonconvex_two_chords_through_the_notch() {
    let fx = nonconvex_poly().unwrap();
    let t = build_mesh_tracker(&fx.mesh, &vacuum(), &opts_fail());
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let d = [s, -s, 0.0];
    let o = [0.5, 2.0, 0.5];
    // Analytic: two L chords of length sqrt(2)/2 and a brick chord of the same length.
    let brick = Convex::boxed([2.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
    let l = l_cell_intervals(o, d);
    assert_eq!(l.len(), 2);
    let mut sc = Scratch::new();
    let mut l_chords = Vec::new();
    let mut all = l.clone();
    all.extend(brick.clip(o, d));
    for (a, b) in merge_runs(all) {
        let p0 = [o[0] + a * d[0], o[1] + a * d[1], o[2]];
        let tr = trace_to_end(&t, p0, d, &mut sc);
        assert!(!tr.lost);
        let total: f64 = tr.chords.iter().map(|c| c.1).sum();
        assert!((total - (b - a)).abs() < 1e-12);
        l_chords.extend(tr.chords.iter().filter(|c| c.0 == 0).map(|c| c.1));
    }
    assert_eq!(l_chords.len(), 2, "exactly two disjoint chords in the L cell");
    for c in l_chords {
        assert!((c - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12, "{c}");
    }
}

#[test]
fn acc03_nonconvex_iur_expected_chord() {
    let fx = nonconvex_poly().unwrap();
    let t = build_mesh_tracker(&fx.mesh, &vacuum(), &opts_fail());
    let brick = Convex::boxed([2.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
    let center = [1.5, 1.0, 0.5];
    let rho = ((3.0f64).powi(2) + 2.0f64.powi(2) + 1.0).sqrt() / 2.0 * 1.0001;
    let runs = |o: [f64; 3], d: [f64; 3]| {
        let mut iv = l_cell_intervals(o, d);
        iv.extend(brick.clip(o, d));
        merge_runs(iv)
    };
    let st = run_iur(&t, "nonconvex_poly", n_lines(), center, rho, &runs);
    assert_eq!(st.lost, 0);
    assert!(st.max_err <= 1e-12, "{}", st.max_err);
    assert_cell_means(&st, &fx.cell_volumes, rho, 4.0);
}

/// Forward and reversed traces of one run cross the same faces.
fn faces_of(t: &dyn Tracker, p0: [f64; 3], dir: [f64; 3], sc: &mut Scratch) -> Option<Vec<FaceId>> {
    let mut st = t.init_state(p3(p0), v3(dir), None, sc).ok()?;
    if st.cell.is_none() {
        return Some(vec![]);
    }
    let mut out = Vec::new();
    for _ in 0..100_000 {
        let s = t.step(&mut st, f64::INFINITY, sc).ok()?;
        match s.end {
            StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } => out.push(face),
            StepEnd::Boundary { face, .. } => {
                out.push(face);
                return Some(out);
            }
            _ => return None,
        }
    }
    None
}

#[test]
fn acc04_shared_face_consistency_forward_and_reverse() {
    let hb = hex_block([4; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let cases: Vec<(String, sm_mesh_api::reference::ReferenceMesh)> = vec![
        ("hex_block_4".into(), hb),
        ("mixed_transition".into(), mixed_transition().unwrap().mesh),
        ("nonconvex_poly".into(), nonconvex_poly().unwrap().mesh),
        ("dual_patch".into(), dual_patch().unwrap().mesh),
    ];
    let per_case = if cfg!(debug_assertions) { 5_000 } else { 25_000 };
    for (name, mesh) in cases {
        let t = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
        let polys: Vec<Convex> = (0..mesh.cell_count()).map(|c| cell_polytope(&mesh, c)).collect();
        let (lo, hi) = bbox(&mesh);
        let center = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
        let rho = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt() / 2.0 * 1.0001;
        let mut r = rng(&format!("acc04_{name}"));
        let mut sc = Scratch::new();
        let mut checked = 0;
        let mut crossings = 0usize;
        // The non-convex L cell is the union of two boxes for the analytic hull.
        for _ in 0..per_case {
            let (o, d) = iur_line(&mut r, center, rho);
            let mut iv: Vec<(f64, f64)> = polys.iter().filter_map(|p| p.clip(o, d)).collect();
            if name == "nonconvex_poly" {
                iv = l_cell_intervals(o, d);
                iv.extend(Convex::boxed([2.0, 0.0, 0.0], [3.0, 1.0, 1.0]).clip(o, d));
            }
            for (a, b) in merge_runs(iv) {
                let (p0, p1) = ([o[0] + a * d[0], o[1] + a * d[1], o[2] + a * d[2]], [o[0] + b * d[0], o[1] + b * d[1], o[2] + b * d[2]]);
                let nd = [-d[0], -d[1], -d[2]];
                let (Some(fw), Some(bw)) = (faces_of(&t, p0, d, &mut sc), faces_of(&t, p1, nd, &mut sc)) else {
                    panic!("{name}: lost or malformed trace")
                };
                // Forward exits f1..fk (fk on the boundary); reverse exits fk-1..f1, f0.
                assert_eq!(fw.len(), bw.len(), "{name}: crossing count differs");
                let k = fw.len();
                for i in 0..k - 1 {
                    assert_eq!(fw[i], bw[k - 2 - i], "{name}: shared face differs at crossing {i}");
                }
                crossings += k - 1;
                checked += 1;
            }
        }
        assert!(checked > 100, "{name}: too few lines hit ({checked})");
        eprintln!("{name}: {checked} runs, {crossings} shared-face crossings");
    }
}



#[test]
fn acc04_edge_exits_pick_the_lowest_face_and_never_get_lost() {
    let mesh = hex_block([4; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
    let mut r = rng("acc04_edges");
    let mut sc = Scratch::new();
    let mut tested = 0;
    for c in 0..mesh.cell_count() {
                for (f1, f2, p, q) in shared_edge(&mesh, c) {
            let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0];
            // The cell centre is on the inner side of both faces; aim from it through the edge.
            let cell_center = {
                let co = mesh.coords();
                let mut s = [0.0; 3];
                let mut n = 0.0;
                for sf in mesh.cell_faces(CellIx::new(c)) {
                    for v in mesh.face_vertices(sf.face()) {
                        s = [s[0] + co.x[v.get()], s[1] + co.y[v.get()], s[2] + co.z[v.get()]];
                        n += 1.0;
                    }
                }
                [s[0] / n, s[1] / n, s[2] / n]
            };
            for _ in 0..4 {
                // Directions through the edge midpoint from a random start inside the cell.
                let jitter = [0.2 * r.unit(), 0.2 * r.unit(), 0.2 * r.unit()];
                let start = [cell_center[0] + jitter[0] * 0.5, cell_center[1] + jitter[1] * 0.5, cell_center[2] + jitter[2] * 0.5];
                let dd = sub(mid, start);
                let l = dot(dd, dd).sqrt();
                let d = [dd[0] / l, dd[1] / l, dd[2] / l];
                let mut st = t.init_state(p3(start), v3(d), Some(CellId::new(c)), &mut sc).unwrap();
                assert_eq!(st.cell, CellId::new(c));
                let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
                let face = match s.end {
                    StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } | StepEnd::Boundary { face, .. } => face,
                    other => panic!("cell {c}: {other:?}"),
                };
                let lowest = FaceId::new(f1.get().min(f2.get()));
                // The exit is within rounding of the edge, so either face of the edge may be the
                // true minimum; ties (within the tie window) resolve to the lowest id.
                assert!(face == FaceId::new(f1.get()) || face == FaceId::new(f2.get()), "exit through neither edge face");
                let exit_pt = [start[0] + s.dist * d[0], start[1] + s.dist * d[1], start[2] + s.dist * d[2]];
                assert!(sub(exit_pt, mid).iter().all(|x| x.abs() < 1e-12));
                assert_eq!(face, lowest, "tied exit must pick the lowest FaceId");
                tested += 1;
            }
        }
    }
    assert!(tested > 500);
    // Lines within 1e-9 of an edge or vertex: never Lost, from the whole model.
    let mut lost = 0;
    for c in 0..mesh.cell_count() {
        for (_, _, p, q) in shared_edge(&mesh, c) {
            for k in 0..3 {
                let w = (k as f64 + 1.0) / 4.0;
                let on = [p[0] + (q[0] - p[0]) * w, p[1] + (q[1] - p[1]) * w, p[2] + (q[2] - p[2]) * w];
                let eps = [1e-9 * r.unit(), 1e-9 * r.unit(), 1e-9 * r.unit()];
                let (d, _, _) = random_frame(&mut r);
                let o = [on[0] + eps[0] - 3.0 * d[0], on[1] + eps[1] - 3.0 * d[1], on[2] + eps[2] - 3.0 * d[2]];
                let hull = Convex::boxed([0.0; 3], [4.0; 3]);
                if let Some((a, b)) = hull.clip(o, d).filter(|(a, b)| b > a) {
                    let tr = trace_to_end(&t, [o[0] + a * d[0], o[1] + a * d[1], o[2] + a * d[2]], d, &mut sc);
                    let total: f64 = tr.chords.iter().map(|c| c.1).sum();
                    // Chords shorter than the model tolerance may be dropped (the start is within
                    // tolerance of the hull and the direction leaves); longer ones must be exact.
                    if tr.lost || (b - a > 1e-3 && (total - (b - a)).abs() > 1e-12) || total > b - a + 1e-12 {
                        lost += 1;
                        if lost < 6 {
                            eprintln!("cell {c}: lost={} total={total} want={} o={o:?} d={d:?} chords={}", tr.lost, b - a, tr.chords.len());
                        }
                    }
                }
            }
        }
    }
    assert_eq!(lost, 0);
}



#[test]
fn acc04_vertex_exits_pick_the_lowest_face_and_never_get_lost() {
    let mesh = hex_block([3; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
    let mut r = rng("acc04_vertices");
    let mut sc = Scratch::new();
    let mut tested = 0;
    for c in 0..mesh.cell_count() {
        let center = cell_vertices(&mesh, c).iter().fold([0.0; 3], |a, (_, p, _)| [a[0] + p[0] / 8.0, a[1] + p[1] / 8.0, a[2] + p[2] / 8.0]);
        for (_, vtx, faces) in cell_vertices(&mesh, c) {
            for _ in 0..3 {
                let start = [center[0] + 0.1 * r.unit(), center[1] + 0.1 * r.unit(), center[2] + 0.1 * r.unit()];
                let dd = sub(vtx, start);
                let l = dot(dd, dd).sqrt();
                let d = [dd[0] / l, dd[1] / l, dd[2] / l];
                let mut st = t.init_state(p3(start), v3(d), Some(CellId::new(c)), &mut sc).unwrap();
                let s = t.step(&mut st, f64::INFINITY, &mut sc).unwrap();
                let face = match s.end {
                    StepEnd::CellFace { face, .. } | StepEnd::RegionBoundary { face, .. } | StepEnd::Boundary { face, .. } => face,
                    other => panic!("cell {c}: {other:?}"),
                };
                let lowest = faces.iter().map(|f| f.get()).min().unwrap();
                assert_eq!(face, FaceId::new(lowest), "vertex exit must pick the lowest of the {} tied faces", faces.len());
                tested += 1;
            }
        }
    }
    assert!(tested > 600);
    // Lines within 1e-9 of a vertex never end lost and keep the chord sum.
    let hull = Convex::boxed([0.0; 3], [3.0; 3]);
    for c in 0..mesh.cell_count() {
        for (_, vtx, _) in cell_vertices(&mesh, c) {
            for _ in 0..5 {
                let (d, _, _) = random_frame(&mut r);
                let o = [vtx[0] + 1e-9 * r.unit() - 5.0 * d[0], vtx[1] + 1e-9 * r.unit() - 5.0 * d[1], vtx[2] + 1e-9 * r.unit() - 5.0 * d[2]];
                if let Some((a, b)) = hull.clip(o, d).filter(|(a, b)| b - a > 1e-3) {
                    let tr = trace_to_end(&t, [o[0] + a * d[0], o[1] + a * d[1], o[2] + a * d[2]], d, &mut sc);
                    let total: f64 = tr.chords.iter().map(|c| c.1).sum();
                    assert!(!tr.lost && (total - (b - a)).abs() < 1e-12, "cell {c} vertex {vtx:?}: lost {} err {}", tr.lost, (total - (b - a)).abs());
                }
            }
        }
    }
}
