//! Acceptance 8: BVH locate against brute force, near-feature points, hashed direction.
mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, MeshRead, SerialAlloc};
use sm_rt_api::{CellId, Scratch, Tracker};
use sm_testkit::meshes::mixed_transition;

fn brute_cell(polys: &[Convex], p: [f64; 3]) -> Option<usize> {
    let mut found = None;
    for (c, pl) in polys.iter().enumerate() {
        let min_slack = pl.planes.iter().map(|(n, d)| d - dot(*n, p)).fold(f64::INFINITY, f64::min);
        if min_slack.abs() < 1e-9 {
            return None; // too close to a face for an unambiguous answer
        }
        if min_slack > 0.0 {
            found = Some(c);
        }
    }
    Some(found.unwrap_or(usize::MAX)) // usize::MAX: outside every cell
}

fn random_points_agree(mesh: &dyn MeshRead, name: &str) {
    let t = build_mesh_tracker(mesh, &vacuum(), &opts_fail());
    let polys: Vec<Convex> = (0..mesh.cell_count()).map(|c| cell_polytope(mesh, c)).collect();
    let (lo, hi) = bbox(mesh);
    let mut r = rng(name);
    let mut sc = Scratch::new();
    let (mut inside, mut outside) = (0, 0);
    for i in 0..100_000 {
        let u = |k: usize, r: &mut SplitMix64| lo[k] - 0.1 + (hi[k] - lo[k] + 0.2) * (r.unit() + 1.0) / 2.0;
        let q = [u(0, &mut r), u(1, &mut r), u(2, &mut r)];
        let Some(want) = brute_cell(&polys, q) else { continue };
        // Alternate between no hint, a far hint and the right hint.
        let hint = match i % 3 {
            0 => None,
            1 => Some(CellId::new((i / 3) % mesh.cell_count())),
            _ => (want != usize::MAX).then(|| CellId::new(want)),
        };
        let loc = t.locate(p3(q), None, hint, &mut sc).unwrap();
        if loc.on_face.is_some() {
            // Within the on-face tolerance the direction (hashed here) decides; flagged, never lost.
            assert!(!loc.cell.is_none());
            continue;
        }
        if want == usize::MAX {
            assert!(loc.cell.is_none(), "{name}: {q:?} -> {:?}", loc.cell);
            outside += 1;
        } else {
            assert_eq!(loc.cell, CellId::new(want), "{name}: {q:?} hint {hint:?}");
            inside += 1;
        }
    }
    assert!(inside > 1000 && outside > 100, "{inside} {outside}");
}

#[test]
fn acc08_random_points_match_brute_force() {
    let hb = hex_block([10; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    random_points_agree(&hb, "locate_hex_block_10");
    random_points_agree(&mixed_transition().unwrap().mesh, "locate_mixed_transition");
}

/// Feature points of cell `c`: face centres, edge midpoints, vertices.
fn features(m: &dyn MeshRead, c: usize) -> Vec<[f64; 3]> {
    let co = m.coords();
    let mut out = Vec::new();
    for sf in m.cell_faces(CellIx::new(c)) {
        let l = m.face_vertices(sf.face());
        let mean = l.iter().fold([0.0; 3], |a, v| [a[0] + co.x[v.get()], a[1] + co.y[v.get()], a[2] + co.z[v.get()]]);
        out.push([mean[0] / l.len() as f64, mean[1] / l.len() as f64, mean[2] / l.len() as f64]);
    }
    for (_, _, p, q) in shared_edge(m, c) {
        out.push([(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0]);
    }
    for (_, v, _) in cell_vertices(m, c) {
        out.push(v);
    }
    out
}

fn cell_centroid(m: &dyn MeshRead, c: usize) -> [f64; 3] {
    let vs = cell_vertices(m, c);
    let s = vs.iter().fold([0.0; 3], |a, (_, p, _)| [a[0] + p[0], a[1] + p[1], a[2] + p[2]]);
    [s[0] / vs.len() as f64, s[1] / vs.len() as f64, s[2] / vs.len() as f64]
}

fn near_features(mesh: &dyn MeshRead, name: &str) {
    let t = build_mesh_tracker(mesh, &vacuum(), &opts_fail());
    let mut sc = Scratch::new();
    let mut n = 0;
    for c in 0..mesh.cell_count() {
        let ctr = cell_centroid(mesh, c);
        for f in features(mesh, c) {
            let dir = sub(ctr, f);
            let l = dot(dir, dir).sqrt();
            let u = [dir[0] / l, dir[1] / l, dir[2] / l];
            for delta in [1e-3, 1e-9, 0.0] {
                let p = [f[0] + delta * u[0], f[1] + delta * u[1], f[2] + delta * u[2]];
                // No direction: the hashed default decides on a face; the result must be a cell
                // that touches the feature, flagged on_face when within tolerance.
                let loc = t.locate(p3(p), None, None, &mut sc).unwrap_or_else(|e| panic!("{name}: {e}"));
                // On a model-boundary feature the hashed direction may point out of the model:
                // then the answer is "outside", flagged on_face.
                assert!(!loc.cell.is_none() || loc.on_face.is_some(), "{name}: cell {c} feature {f:?} delta {delta}");
                if delta == 1e-3 {
                    assert_eq!(loc.cell, CellId::new(c), "{name}: strictly inside is exact");
                    assert!(loc.on_face.is_none());
                } else if loc.on_face.is_none() {
                    assert_eq!(loc.cell, CellId::new(c), "{name}: off-face points are classified exactly");
                } else {
                    // Flagged: the direction picks the cell; aiming into the cell must give c.
                    let d = v3(u);
                    let loc2 = t.locate(p3(p), Some(d), None, &mut sc).unwrap();
                    assert_eq!(loc2.cell, CellId::new(c), "{name}: direction into the cell picks it");
                }
                n += 1;
            }
        }
    }
    assert!(n > 100);
}

#[test]
fn acc08_points_near_faces_edges_and_vertices_never_lost() {
    near_features(&hex_block([3; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap(), "hex_block_3");
    near_features(&mixed_transition().unwrap().mesh, "mixed_transition");
}

#[test]
fn acc08_directionless_locate_on_a_face_is_deterministic() {
    let mesh = hex_block([4; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_mesh_tracker(&mesh, &vacuum(), &opts_fail());
    let mut pts = Vec::new();
    for c in 0..mesh.cell_count() {
        pts.extend(features(&mesh, c));
    }
    let run = |threads: usize| -> u64 {
        let outs = sm_testkit::determinism::par_map(threads, pts.len(), |i| {
            let mut sc = Scratch::new();
            let a = t.locate(p3(pts[i]), None, None, &mut sc).unwrap();
            let b = t.locate(p3(pts[i]), None, Some(CellId::new(i % 64)), &mut sc).unwrap();
            assert_eq!((a.cell, a.on_face), (b.cell, b.on_face), "hint must not change a directionless answer");
            (a.cell.0, a.on_face.map_or(u32::MAX, |f| f.0))
        });
        outs.iter().fold(0u64, |h, o| h.rotate_left(7) ^ (u64::from(o.0) << 32 | u64::from(o.1)))
    };
    sm_testkit::determinism::check(3, 4, run).unwrap();
}
