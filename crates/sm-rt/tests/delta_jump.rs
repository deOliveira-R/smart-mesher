//! W3b-RT-DELTA acceptance 9: the region jump gives the walk's answers.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, MeshRead, SerialAlloc};
use sm_rt::MeshTracker;
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{
    BoundaryDef, BoundarySpec, BuildInput, BuildOptions, CellId, ComplementPolicy, FaceId, FireResult, MeshInput, RegionRule, Scratch, StepEnd, TrackState, Tracker, TrackerCaps,
};
use sm_testkit::meshes::{MeshFixture, SetSpec, nonconvex_poly, with_sets};

/// The face a fire ended on, if the end is a face event.
fn end_face(e: &StepEnd) -> Option<FaceId> {
    match e {
        StepEnd::RegionBoundary { face, .. } | StepEnd::Boundary { face, .. } => Some(*face),
        _ => None,
    }
}

/// Kind, surface, region and (for boundaries) action of an end; faces and cells are compared
/// separately because of the tie rule.
fn same_kind(a: &StepEnd, b: &StepEnd) -> bool {
    match (a, b) {
        (StepEnd::Limit, StepEnd::Limit) => true,
        (StepEnd::RegionBoundary { surface: s0, to_region: r0, .. }, StepEnd::RegionBoundary { surface: s1, to_region: r1, .. }) => s0 == s1 && r0 == r1,
        (StepEnd::Boundary { surface: s0, action: a0, .. }, StepEnd::Boundary { surface: s1, action: a1, .. }) => s0 == s1 && a0 == a1,
        (StepEnd::Lost(_), StepEnd::Lost(_)) => true,
        _ => false,
    }
}

/// Exact distance from `o` along unit `d` to the plane of face `f` (planar faces), with the
/// face normal. `None` for faces that are not mesh faces.
fn exact_exit(t: &MeshTracker, mesh: &dyn MeshRead, f: FaceId, o: [f64; 3], d: [f64; 3]) -> Option<(f64, [f64; 3])> {
    if f.get() >= mesh.face_count() {
        // A virtual face `n_faces + k` of the complement: side `k` (-x, +x, -y, +y, -z, +z) of
        // the box of the vertices.
        let k = f.get() - mesh.face_count();
        if k >= 6 {
            return None;
        }
        let (ax, hi) = (k / 2, k % 2 == 1);
        let b = t.info().bounds;
        let plane = if hi { [b.hi.x, b.hi.y, b.hi.z][ax] } else { [b.lo.x, b.lo.y, b.lo.z][ax] };
        let mut n = [0.0; 3];
        n[ax] = if hi { 1.0 } else { -1.0 };
        return Some(((plane - o[ax]) / d[ax], n));
    }
    let v = mesh.face_vertices(FaceIx::new(f.get()))[0];
    let p0 = mesh.point(v.get());
    let n = t.face_normal(f, p0).ok()?;
    let n = [n.x, n.y, n.z];
    let nd = dot(n, d);
    let num = n[0] * (p0.x - o[0]) + n[1] * (p0.y - o[1]) + n[2] * (p0.z - o[2]);
    Some((num / nd, n))
}

/// Distance from `p` to the triangle `abc` (closest point by barycentric regions).
fn dist_to_triangle(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let sub = |x: [f64; 3], y: [f64; 3]| [x[0] - y[0], x[1] - y[1], x[2] - y[2]];
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let at = |q: [f64; 3]| dot(sub(p, q), sub(p, q)).sqrt();
    if d1 <= 0.0 && d2 <= 0.0 {
        return at(a);
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return at(b);
    }
    let vc = d1 * d4 - d3 * d2;
    let on = |o: [f64; 3], e: [f64; 3], t: f64| at([o[0] + e[0] * t, o[1] + e[1] * t, o[2] + e[2] * t]);
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return on(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return at(c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return on(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let bc = sub(c, b);
        return on(b, bc, (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    let q = [a[0] + ab[0] * vb * denom + ac[0] * vc * denom, a[1] + ab[1] * vb * denom + ac[1] * vc * denom, a[2] + ab[2] * vb * denom + ac[2] * vc * denom];
    at(q)
}

/// Distance from `p` to the closed polygon of mesh face `f` (its fan triangles).
fn dist_to_face(mesh: &dyn MeshRead, f: FaceId, p: [f64; 3]) -> f64 {
    let vs = mesh.face_vertices(FaceIx::new(f.get()));
    let pt = |i: usize| {
        let q = mesh.point(vs[i].get());
        [q.x, q.y, q.z]
    };
    (1..vs.len() - 1).map(|i| dist_to_triangle(p, pt(0), pt(i), pt(i + 1))).fold(f64::INFINITY, f64::min)
}

/// Do two mesh faces share a vertex (hence an edge or a vertex of the grid)?
fn faces_share_vertex(mesh: &dyn MeshRead, a: FaceId, b: FaceId) -> bool {
    let (va, vb) = (mesh.face_vertices(FaceIx::new(a.get())), mesh.face_vertices(FaceIx::new(b.get())));
    va.iter().any(|x| vb.contains(x))
}

/// What a comparison saw beyond the work counts.
#[derive(Default, Debug)]
struct Seen {
    walk_tests: u64,
    jump_work: u64,
    jumps: u64,
    /// Rays that ended on a complement face (a virtual box side).
    complement_exits: u64,
    /// Rays that started inside the complement.
    complement_starts: u64,
    /// Rays whose jump face differs from the walk's (edge or vertex ties).
    ties: u64,
}

/// Ruling R1: the jump matches the exact exit distance within `1e-12 (1 + d)` and the walk
/// within `1e-12 (1 + d) / max(|d.n|, 1e-6)`, with the same end kind, surface, region and
/// to_region. Face and to_cell are the walk's, except on an exact edge or vertex tie, where the
/// jump's face must also contain the exit point.
fn compare(jump: &MeshTracker, walk: &MeshTracker, mesh: &dyn MeshRead, rays: &[([f64; 3], [f64; 3], f64)], name: &str) -> Seen {
    let (mut sj, mut sw) = (Scratch::new(), Scratch::new());
    let mut seen = Seen::default();
    let mut checked = 0;
    for (i, (o, d, lim)) in rays.iter().enumerate() {
        let a = walk.init_state(p3(*o), v3(*d), None, &mut sw).unwrap();
        if a.cell.is_none() {
            continue;
        }
        let b = jump.init_state(p3(*o), v3(*d), None, &mut sj).unwrap();
        assert_eq!(a, b);
        seen.complement_starts += u64::from(a.cell == CellId::COMPLEMENT);
        let (mut sa, mut sb): (TrackState, TrackState) = (a, b);
        let ra: FireResult = walk.fire(&mut sa, *lim, 1_000_000, &mut sw).unwrap();
        let rb: FireResult = jump.fire(&mut sb, *lim, 1_000_000, &mut sj).unwrap();
        let ctx = format!("{name} ray {i} from {o:?} dir {d:?} limit {lim}: walk {ra:?} vs jump {rb:?}");
        assert!(same_kind(&ra.end, &rb.end), "{ctx}");
        let mut tol_walk = 1e-12 * (1.0 + ra.dist);
        let mut same_face = true;
        if let Some(f) = end_face(&ra.end) {
            if let Some((ex, n)) = exact_exit(walk, mesh, f, *o, *d) {
                assert!((rb.dist - ex).abs() <= 1e-12 * (1.0 + ex), "{ctx}: exact {ex}");
                tol_walk /= dot(n, *d).abs().max(1e-6);
            }
            seen.complement_exits += u64::from(f.get() >= mesh.face_count());
            let fb = end_face(&rb.end).unwrap();
            same_face = f == fb;
            if !same_face {
                // Tie: the jump's face must contain the exit point (its closed polygon, not only
                // its plane), or share an edge or vertex with the walk's face.
                seen.ties += 1;
                assert!(fb.get() < mesh.face_count() && f.get() < mesh.face_count(), "{ctx}: a tie on a complement face");
                let exit = [o[0] + d[0] * rb.dist, o[1] + d[1] * rb.dist, o[2] + d[2] * rb.dist];
                let (on_face, shared) = (dist_to_face(mesh, fb, exit) <= 1e-9, faces_share_vertex(mesh, f, fb));
                assert!(on_face || shared, "{ctx}: the exit point is {} from the jump face and the faces share no vertex", dist_to_face(mesh, fb, exit));
                let (ex2, _) = exact_exit(jump, mesh, fb, *o, *d).unwrap_or((rb.dist, [0.0; 3]));
                assert!((ex2 - rb.dist).abs() <= 1e-12 * (1.0 + rb.dist), "{ctx}: tie face distance {ex2}");
            }
            if same_face && let (StepEnd::RegionBoundary { to_cell: c0, .. }, StepEnd::RegionBoundary { to_cell: c1, .. }) = (&ra.end, &rb.end) {
                assert_eq!(c0, c1, "{ctx}");
            }
        }
        assert!((ra.dist - rb.dist).abs() <= tol_walk, "{ctx}: tol {tol_walk}");
        if !matches!(ra.end, StepEnd::Lost(_)) {
            assert_eq!(sa.region, sb.region, "{ctx}");
            if same_face {
                assert_eq!(sa.cell, sb.cell, "{ctx}: final cell");
            }
        }
        checked += 1;
    }
    assert!(checked > rays.len() as u64 / 2, "{name}: {checked} of {} rays started inside", rays.len());
    seen.walk_tests = sw.counters.face_tests;
    seen.jump_work = sj.counters.face_tests + sj.counters.node_visits;
    seen.jumps = sj.counters.jumps;
    seen
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
    let seen = compare(&jump, &walk, &mesh, &random_rays("acc09_hex20", n, 0.0, 20.0), "hex_block(20)");
    assert!(seen.jumps > 0);
    eprintln!("hex_block(20): {seen:?}");
    // Two regions, one of them non-convex.
    let fx = l_shape(8);
    let rule = RegionRule::ByCellSets(vec!["a".into(), "b".into()]);
    let jump = build_part(&fx.mesh, &rule, &vacuum(), popts(4, true));
    let walk = build_plain(&fx.mesh, &rule, &vacuum());
    compare(&jump, &walk, &fx.mesh, &random_rays("acc09_l", n, 0.0, 8.0), "l_shape(8)");
    // Mirror decks: the deck event is the walk's.
    let jump = build_part(&fx.mesh, &rule, &mirror_all(), popts(4, true));
    let walk = build_plain(&fx.mesh, &rule, &mirror_all());
    compare(&jump, &walk, &fx.mesh, &random_rays("acc09_l_mirror", n / 4, 0.0, 8.0), "l_shape(8) mirror");
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
    let seen = compare(&jump, &walk, &fx.mesh, &rays, "ties");
    eprintln!("ties: {seen:?}");
}

/// A complement region ("void" fills the notch of the non-convex prism and the box around it) and
/// a reflecting default, so a ray leaves the complement through a virtual side of the box. The
/// jump must agree with the walk on those exits too, and they are counted, not skipped.
#[test]
fn acc09_complement_region_exits_are_counted_and_match_the_walk() {
    let fx = nonconvex_poly().unwrap();
    let rule = RegionRule::ByVolumeClassification;
    let comp = ComplementPolicy::Region("void".into());
    let mut spec = BoundarySpec::default();
    spec.default = BoundaryDef::Reflect;
    let build = |part: bool| {
        let input = BuildInput::Mesh(MeshInput::new(&fx.mesh, &rule, &spec, &comp));
        let opts = if part { BuildOptions::default().with_partition(popts(2, true)) } else { BuildOptions::default() };
        MeshTrackerBuilder.build_concrete(&input, &opts).unwrap().0
    };
    let (jump, walk) = (build(true), build(false));
    assert!(jump.caps().has(TrackerCaps::REGION_JUMP));
    let n = if cfg!(debug_assertions) { 30_000 } else { 400_000 };
    let mut r = rng("acc09_complement");
    let rays: Vec<([f64; 3], [f64; 3], f64)> = (0..n)
        .map(|i| {
            let o = [3.0 * unit01(&mut r), 2.0 * unit01(&mut r), unit01(&mut r)];
            let (d, _, _) = random_frame(&mut r);
            (o, d, if i % 4 == 0 { 6.0 * unit01(&mut r) } else { f64::INFINITY })
        })
        .collect();
    let seen = compare(&jump, &walk, &fx.mesh, &rays, "nonconvex_poly + complement");
    eprintln!("complement: {seen:?}");
    assert!(seen.complement_exits > 100, "rays ending on a virtual box side: {}", seen.complement_exits);
    assert!(seen.complement_starts > 100, "rays starting in the complement: {}", seen.complement_starts);
    assert!(seen.jumps > 0);
}

#[test]
fn acc09_jump_does_less_work_than_the_walk_on_a_large_block() {
    let n = if cfg!(debug_assertions) { 2_000 } else { 100_000 };
    let mesh = hex_block([40; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let rule = RegionRule::ByVolumeClassification;
    let jump = build_part(&mesh, &rule, &vacuum(), popts(8, true));
    let walk = build_plain(&mesh, &rule, &vacuum());
    let seen = compare(&jump, &walk, &mesh, &random_rays("acc09_hex40", n, 0.0, 40.0), "hex_block(40)");
    eprintln!("hex_block(40), {n} rays: {seen:?}");
    assert!(seen.jump_work < seen.walk_tests, "jump {} must do fewer tests than the walk {}", seen.jump_work, seen.walk_tests);
    // `step` and `trace` never jump.
    let mut sc = Scratch::new();
    let mut st = jump.init_state(p3([1.5, 1.5, 1.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    sc.clear_counters();
    let s = jump.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    assert!(matches!(s.end, StepEnd::CellFace { .. }));
    assert_eq!(sc.counters.jumps, 0);
}
