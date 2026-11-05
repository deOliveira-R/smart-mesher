//! W3b-RT-DELTA acceptance 9: the region jump gives the walk's answers.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, SerialAlloc};
use sm_rt::MeshTracker;
use sm_rt_api::{FireResult, RegionRule, Scratch, StepEnd, TrackState, Tracker, TrackerCaps};
use sm_testkit::meshes::{MeshFixture, SetSpec, with_sets};

fn same_end(a: &StepEnd, b: &StepEnd, strict: bool) -> bool {
    match (a, b) {
        (StepEnd::Limit, StepEnd::Limit) => true,
        (StepEnd::RegionBoundary { surface: s0, face: f0, to_region: r0, to_cell: c0 }, StepEnd::RegionBoundary { surface: s1, face: f1, to_region: r1, to_cell: c1 }) => {
            s0 == s1 && r0 == r1 && (!strict || (f0 == f1 && c0 == c1))
        }
        (StepEnd::Boundary { surface: s0, face: f0, action: a0 }, StepEnd::Boundary { surface: s1, face: f1, action: a1 }) => s0 == s1 && a0 == a1 && (!strict || f0 == f1),
        (StepEnd::Lost(_), StepEnd::Lost(_)) => true,
        _ => false,
    }
}

/// A block with region `a` an L-shape (`i < n/2 || j < n/2`, non-convex) and region `b` the
/// rest, cells numbered x fastest.
fn l_shape(n: u32) -> MeshFixture {
    let base = MeshFixture {
        name: "l_shape",
        mesh: hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap(),
        cell_volumes: vec![1.0; (n * n * n) as usize],
        volume: f64::from(n * n * n),
        area: 6.0 * f64::from(n * n),
        boundary_faces: (6 * n * n) as usize,
    };
    let (mut a, mut b) = (Vec::new(), Vec::new());
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let c = CellIx::new((i + n * (j + n * k)) as usize);
                if i < n / 2 || j < n / 2 { a.push(c) } else { b.push(c) }
            }
        }
    }
    with_sets(base, &[SetSpec::new("a", sm_mesh_api::SetMembers::Cells(a)), SetSpec::new("b", sm_mesh_api::SetMembers::Cells(b))]).unwrap()
}

/// Exit distance of a ray from the box `[0, hi]^3`, by the slab formula (well conditioned: the
/// numerator is a difference of nearby doubles, so it is exact to one rounding).
fn box_exit(o: &[f64; 3], d: &[f64; 3], hi: f64) -> f64 {
    (0..3).filter(|&i| d[i] != 0.0).map(|i| (if d[i] > 0.0 { hi } else { 0.0 } - o[i]) / d[i]).fold(f64::INFINITY, f64::min)
}

fn compare(jump: &MeshTracker, walk: &MeshTracker, rays: &[([f64; 3], [f64; 3], f64)], name: &str, strict: bool) -> (u64, u64, u64) {
    compare_box(jump, walk, rays, name, strict, None)
}

/// With `exact_box = Some(hi)` (a one-region `[0, hi]^3` block) a `Boundary` end is also checked
/// against the exact slab exit. The walk accumulates a rounding error per crossed cell, amplified
/// by `1/|d.n|` for grazing rays, so the walk is compared within a conditioning-scaled tolerance
/// and the jump (one plane test) is the reference.
fn compare_box(jump: &MeshTracker, walk: &MeshTracker, rays: &[([f64; 3], [f64; 3], f64)], name: &str, strict: bool, exact_box: Option<f64>) -> (u64, u64, u64) {
    let (mut sj, mut sw) = (Scratch::new(), Scratch::new());
    let mut checked = 0;
    for (i, (o, d, lim)) in rays.iter().enumerate() {
        let a = walk.init_state(p3(*o), v3(*d), None, &mut sw).unwrap();
        if a.cell.is_none() {
            continue;
        }
        let b = jump.init_state(p3(*o), v3(*d), None, &mut sj).unwrap();
        assert_eq!(a, b);
        let (mut sa, mut sb): (TrackState, TrackState) = (a, b);
        let ra: FireResult = walk.fire(&mut sa, *lim, 1_000_000, &mut sw).unwrap();
        let rb: FireResult = jump.fire(&mut sb, *lim, 1_000_000, &mut sj).unwrap();
        assert!(same_end(&ra.end, &rb.end, strict), "{name} ray {i} from {o:?} dir {d:?} limit {lim}: walk {ra:?} vs jump {rb:?}");
        // Walk drift ~ n_cells * eps * L / |d.n|; |d.n| is bounded below by the smallest component.
        let dmin = d.iter().map(|x| x.abs()).fold(f64::INFINITY, f64::min).max(1e-300);
        let tol = 1e-12 * (1.0 + ra.dist) + 256.0 * f64::EPSILON * (1.0 + o.iter().fold(0.0f64, |m, x| m.max(x.abs()))) / dmin.min(1.0);
        assert!((ra.dist - rb.dist).abs() <= tol, "{name} ray {i}: walk {} vs jump {} (tol {tol})", ra.dist, rb.dist);
        if let (Some(hi), StepEnd::Boundary { .. }) = (exact_box, &rb.end) {
            let t = box_exit(o, d, hi);
            assert!((rb.dist - t).abs() <= 1e-14 * (1.0 + t), "{name} ray {i}: jump {} vs exact slab exit {t}", rb.dist);
        }
        if strict && !matches!(ra.end, StepEnd::Lost(_)) {
            assert_eq!(sa.cell, sb.cell, "{name} ray {i}: final cell");
            assert_eq!(sa.region, sb.region);
        }
        checked += 1;
    }
    assert!(checked > rays.len() as u64 / 2, "{name}: {checked} of {} rays started inside", rays.len());
    (sw.counters.face_tests, sj.counters.face_tests + sj.counters.node_visits, sj.counters.jumps)
}

fn random_rays(name: &str, n: usize, lo: f64, hi: f64) -> Vec<([f64; 3], [f64; 3], f64)> {
    let mut r = rng(name);
    (0..n)
        .map(|i| {
            let o = [lo + (hi - lo) * unit01(&mut r), lo + (hi - lo) * unit01(&mut r), lo + (hi - lo) * unit01(&mut r)];
            let (d, _, _) = random_frame(&mut r);
            let lim = if i % 3 == 0 { (hi - lo) * 2.0 * unit01(&mut r) } else { f64::INFINITY };
            (o, d, lim)
        })
        .collect()
}

#[test]
fn acc09_jump_equals_walk_on_one_region_and_a_non_convex_region() {
    let n = if cfg!(debug_assertions) { 30_000 } else { 1_000_000 };
    let mesh = hex_block([20; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let rule = RegionRule::ByVolumeClassification;
    let jump = build_part(&mesh, &rule, &vacuum(), popts(8, true));
    assert!(jump.caps().has(TrackerCaps::REGION_JUMP));
    let walk = build_plain(&mesh, &rule, &vacuum());
    let (w, j, jumps) = compare_box(&jump, &walk, &random_rays("acc09_hex20", n, 0.0, 20.0), "hex_block(20)", true, Some(20.0));
    assert!(jumps > 0);
    eprintln!("hex_block(20): walk face_tests {w}, jump face_tests+node_visits {j}, jumps {jumps}");
    // Two regions, one of them non-convex.
    let fx = l_shape(8);
    let rule = RegionRule::ByCellSets(vec!["a".into(), "b".into()]);
    let jump = build_part(&fx.mesh, &rule, &vacuum(), popts(4, true));
    let walk = build_plain(&fx.mesh, &rule, &vacuum());
    compare(&jump, &walk, &random_rays("acc09_l", n, 0.0, 8.0), "l_shape(8)", true);
    // Mirror decks: the deck event is the walk's.
    let jump = build_part(&fx.mesh, &rule, &mirror_all(), popts(4, true));
    let walk = build_plain(&fx.mesh, &rule, &mirror_all());
    compare(&jump, &walk, &random_rays("acc09_l_mirror", n / 4, 0.0, 8.0), "l_shape(8) mirror", true);
}

#[test]
fn acc09_ties_on_axes_and_diagonals_end_on_the_same_surface() {
    // Rays from cell centres along axes, face diagonals and the main diagonal pass through
    // edges and vertices of the grid. The crossing there is decided by the exact predicates
    // (simulation of simplicity), so the jump leaves through one of the faces of the edge and
    // may differ from the walk's face and `to_cell` (documented in DESIGN.md); the end kind,
    // surface, region and distance still agree.
    let fx = l_shape(8);
    let rule = RegionRule::ByCellSets(vec!["a".into(), "b".into()]);
    let jump = build_part(&fx.mesh, &rule, &vacuum(), popts(4, true));
    let walk = build_plain(&fx.mesh, &rule, &vacuum());
    let mut rays = Vec::new();
    let s2 = std::f64::consts::FRAC_1_SQRT_2;
    let s3 = 1.0 / 3f64.sqrt();
    let dirs = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [s2, s2, 0.0], [s2, 0.0, s2], [0.0, -s2, s2], [s3, s3, s3], [-s3, s3, -s3]];
    for k in 0..8 {
        for j in 0..8 {
            for i in 0..8 {
                for d in dirs {
                    rays.push(([f64::from(i) + 0.5, f64::from(j) + 0.5, f64::from(k) + 0.5], d, f64::INFINITY));
                }
            }
        }
    }
    compare(&jump, &walk, &rays, "ties", false);
}

#[test]
fn acc09_jump_does_less_work_than_the_walk_on_a_large_block() {
    let n = if cfg!(debug_assertions) { 2_000 } else { 100_000 };
    let mesh = hex_block([40; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let rule = RegionRule::ByVolumeClassification;
    let jump = build_part(&mesh, &rule, &vacuum(), popts(8, true));
    let walk = build_plain(&mesh, &rule, &vacuum());
    let (w, j, jumps) = compare(&jump, &walk, &random_rays("acc09_hex40", n, 0.0, 40.0), "hex_block(40)", true);
    eprintln!("hex_block(40), {n} rays: walk face_tests {w}, jump face_tests+node_visits {j} ({jumps} jumps)");
    assert!(j < w, "jump {j} must do fewer tests than the walk {w}");
    // `step` and `trace` never jump.
    let mut sc = Scratch::new();
    let mut st = jump.init_state(p3([1.5, 1.5, 1.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    sc.clear_counters();
    let s = jump.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::CellFace { .. }));
    assert_eq!(sc.counters.jumps, 0);
}
