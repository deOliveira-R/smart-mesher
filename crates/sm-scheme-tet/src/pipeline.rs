//! The surface-to-volume pipeline: validate, Delaunay, recover, classify, refine, optimize,
//! canonical output.

use crate::canon::canonical_tets;
use crate::delaunay::{Dt, splitmix};
use crate::error::TetError;
use crate::geom::{V3, coord_ok, ori, sanitize};
use crate::guard::Guard;
use crate::quality::mesh_stats;
use crate::recover::{Constraints, Recover};
use crate::tds::{NONE, same_cycle, sorted3};
use crate::{SurfaceInput, TetMesh, TetOptions, TetStats, validate};
use std::time::Instant;

/// How often the build restarts with extra interior points after a recovery failure.
const MAX_RESTARTS: usize = 2;

/// Interior hints closer than this (relative to the triangle size) to an input triangle are
/// dropped: they would sit on the surface.
const MIN_MARGIN: f64 = 1e-6;

/// Number of auxiliary far points added around the input.
pub(crate) const N_FAR: usize = 8;

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

fn far_points(pts: &[V3]) -> Result<[V3; N_FAR], TetError> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let span = (0..3).map(|k| hi[k] - lo[k]).fold(0.0f64, f64::max);
    let l = if span > 0.0 { span } else { 1.0 };
    let d = 20.0 * l;
    let mut out = [[0.0; 3]; N_FAR];
    for (i, o) in out.iter_mut().enumerate() {
        for k in 0..3 {
            let sgn = if (i >> k) & 1 == 1 { 1.0 } else { -1.0 };
            let j = (splitmix(0xFA2 + 3 * i as u64 + k as u64) >> 11) as f64 / (1u64 << 53) as f64 - 0.5;
            let c = 0.5 * (lo[k] + hi[k]);
            o[k] = c + d * (sgn + 0.3 * j);
            if !coord_ok(o[k]) {
                return Err(TetError::OutOfRange(sm_math::PredicateError::OutOfRange));
            }
        }
    }
    Ok(out)
}

/// Per-point target size: the mean incident boundary edge length at input points (others 0,
/// filled when the points are created).
fn boundary_sizes(points: &[V3], tris: &[[u32; 3]], total: usize) -> Vec<f64> {
    let mut sum = vec![0.0f64; total];
    let mut cnt = vec![0u32; total];
    let mut edges: Vec<[u32; 2]> = Vec::with_capacity(tris.len() * 3 / 2);
    for t in tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.push([a.min(b), a.max(b)]);
        }
    }
    edges.sort_unstable();
    edges.dedup();
    for e in edges {
        let l = crate::geom::dist(points[e[0] as usize], points[e[1] as usize]);
        for x in e {
            sum[x as usize] += l;
            cnt[x as usize] += 1;
        }
    }
    (0..total).map(|i| if cnt[i] > 0 { sum[i] / f64::from(cnt[i]) } else { f64::INFINITY }).collect()
}

/// Vertex centroid of the connected surface component holding the failing triangle, if that
/// component is tiny (a few dozen vertices). Such a body is one pocket: a single point that sees
/// all of it (Schönhardt-like prisms and their affine images) is what recovery needs, while on
/// larger surfaces a far-away centroid would only get in the way.
fn pocket_point(points: &[V3], tris: &[[u32; 3]], tri: u32, edge: Option<[u32; 2]>) -> Option<V3> {
    const MAX_VERTS: usize = 64;
    let t0 = tris.get(tri as usize)?;
    let mut by_vert: std::collections::BTreeMap<u32, Vec<u32>> = std::collections::BTreeMap::new();
    for (i, t) in tris.iter().enumerate() {
        for &v in t {
            by_vert.entry(v).or_default().push(i as u32);
        }
    }
    let mut seen: std::collections::BTreeSet<u32> = t0.iter().copied().chain(edge.into_iter().flatten()).collect();
    let mut stack: Vec<u32> = seen.iter().copied().collect();
    while let Some(v) = stack.pop() {
        for &t in by_vert.get(&v).map_or(&[][..], |x| x) {
            for &u in &tris[t as usize] {
                if seen.insert(u) {
                    if seen.len() > MAX_VERTS {
                        return None;
                    }
                    stack.push(u);
                }
            }
        }
    }
    let mut c = [0.0; 3];
    for &v in &seen {
        c = crate::geom::add(c, points[v as usize]);
    }
    sanitize(crate::geom::scale(c, 1.0 / seen.len() as f64))
}

/// A Steiner candidate inside the solid next to triangle `tri` (inward normal offset), tried
/// when recovery fails around it and no other candidate was accepted.
fn steiner_near(points: &[V3], tris: &[[u32; 3]], tri: u32, edge: Option<[u32; 2]>, attempt: usize, guard: &Guard<'_>) -> Option<V3> {
    use crate::geom::{add, cross, dist, scale, sub};
    let t = tris.get(tri as usize)?;
    let [a, b, c] = t.map(|i| points[i as usize]);
    let n = cross(sub(b, a), sub(c, a));
    let nl = crate::geom::dot(n, n).sqrt();
    if nl == 0.0 {
        return None;
    }
    let n = scale(n, 1.0 / nl);
    let h = (dist(a, b) + dist(b, c) + dist(c, a)) / 3.0;
    let mut base = scale(add(add(a, b), c), 1.0 / 3.0);
    if let Some(e) = edge {
        // Aim at the failing edge: blend the centroid with the edge midpoint.
        let mid = scale(add(points[e[0] as usize], points[e[1] as usize]), 0.5);
        base = add(scale(base, 0.5), scale(mid, 0.5));
    }
    // Offsets from just compliant with S4 upwards, rotated by the restart number; the best
    // score wins (first on ties).
    let offs = [0.4, 0.55, 0.75, 1.0, 0.35, 0.65];
    let mut best: Option<(f64, V3)> = None;
    for j in 0..offs.len() {
        let off = offs[(attempt + j) % offs.len()];
        if let Some(p) = sanitize(sub(base, scale(n, off * h))) {
            let sc = guard.score(p);
            if best.is_none_or(|b| sc > b.0) {
                best = Some((sc, p));
            }
        }
    }
    best.map(|b| b.1)
}

fn check_options(o: &TetOptions<'_>) -> Result<(), TetError> {
    if o.optimize > 2 {
        return Err(TetError::BadOption(format!("optimize = {} is not supported (0..=2)", o.optimize)));
    }
    if !(o.radius_edge.is_finite() && o.radius_edge >= 1.0) {
        return Err(TetError::BadOption(format!("radius_edge = {} must be finite and >= 1", o.radius_edge)));
    }
    if !(o.gradation.is_finite() && o.gradation >= 1.0) {
        return Err(TetError::BadOption(format!("gradation = {} must be finite and >= 1", o.gradation)));
    }
    if let crate::SizeSpec::Constant(h) = o.size
        && !(h.is_finite() && h > 0.0) {
            return Err(TetError::BadOption(format!("size = {h} must be finite and positive")));
        }
    Ok(())
}

/// Classify tets as inside/outside by flooding from the ghost tets across non-constraint faces,
/// toggling at constraint triangles; returns the outside tets.
fn classify(dt: &Dt, con: &Constraints, tris: &[[u32; 3]]) -> Result<Vec<u32>, TetError> {
    let slots = dt.tds.slots();
    let mut state = vec![0u8; slots]; // 0 unknown, 1 outside, 2 inside
    let mut witness = vec![0u32; slots];
    let mut queue: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
    for t in 0..slots as u32 {
        if dt.tds.alive[t as usize] && dt.tds.ghost_slot(t).is_some() {
            state[t as usize] = 1;
            queue.push_back(t);
        }
    }
    while let Some(t) = queue.pop_front() {
        let s = state[t as usize];
        for i in 0..4 {
            let n = dt.tds.nbr[t as usize][i];
            if n == NONE {
                continue;
            }
            let f = dt.tds.face(t, i);
            let (ns, w) = match con.face_owner(sorted3(f[0], f[1], f[2])) {
                Some(tri) => {
                    let expected = if same_cycle(f, tris[tri as usize]) { 2 } else { 1 };
                    if s != expected {
                        return Err(TetError::InvertedShell { tri });
                    }
                    (3 - s, tri)
                }
                None => (s, witness[t as usize]),
            };
            let cur = state[n as usize];
            if cur == 0 {
                state[n as usize] = ns;
                witness[n as usize] = w;
                queue.push_back(n);
            } else if cur != ns {
                return Err(TetError::InvertedShell { tri: w });
            }
        }
    }
    Ok((0..slots as u32).filter(|&t| dt.tds.alive[t as usize] && state[t as usize] != 2).collect())
}

/// Tetrahedralize the volume enclosed by `input`; see the crate docs.
pub fn tetrahedralize(input: &SurfaceInput, opts: &TetOptions<'_>) -> Result<TetMesh, TetError> {
    tetrahedralize_with(input, opts, false)
}

/// [`tetrahedralize`]; with `complete_only` boundary recovery skips the interior-point recovery
/// and runs complete recovery ([`crate::steiner`]) right after the Delaunay stage (tests).
pub(crate) fn tetrahedralize_with(input: &SurfaceInput, opts: &TetOptions<'_>, complete_only: bool) -> Result<TetMesh, TetError> {
    let mut stage_ms = [0.0f64; 5];
    let t0 = Instant::now();
    check_options(opts)?;
    validate::check_surface(&input.points, &input.tris)?;
    // Canonical triangle order (rotated to the smallest vertex first, sorted), so the result does
    // not depend on the order the triangles are listed in.
    let mut order: Vec<u32> = (0..input.tris.len() as u32).collect();
    let canon: Vec<[u32; 3]> = input.tris.iter().map(|&t| crate::tds::canon_tri(t)).collect();
    order.sort_unstable_by_key(|&i| canon[i as usize]);
    let canonical = SurfaceInput { points: input.points.clone(), tris: order.iter().map(|&i| canon[i as usize]).collect() };
    stage_ms[0] = ms(t0);
    let back = |i: u32| order.get(i as usize).copied().unwrap_or(i);
    run(&canonical, opts, stage_ms, complete_only).map_err(|e| match e {
        TetError::InvertedShell { tri } => TetError::InvertedShell { tri: back(tri) },
        TetError::RecoveryFailed { tri, edge, reason } => TetError::RecoveryFailed { tri: back(tri), edge, reason },
        TetError::SelfIntersection { tri_a, tri_b } => {
            let (a, b) = (back(tri_a), back(tri_b));
            TetError::SelfIntersection { tri_a: a.min(b), tri_b: a.max(b) }
        }
        other => other,
    })
}

fn run(input: &SurfaceInput, opts: &TetOptions<'_>, mut stage_ms: [f64; 5], complete_only: bool) -> Result<TetMesh, TetError> {
    let n = input.points.len();
    let max_steiner = if opts.max_steiner == 0 { 8 * n + 100_000 } else { opts.max_steiner };

    // Stages 2 and 3: Delaunay of the input plus auxiliary far points, then boundary recovery.
    // A recovery failure that interior points can cure restarts the build with an extra point
    // placed inside the solid next to the failing triangle (at most [`MAX_RESTARTS`] times).
    let far = far_points(&input.points)?;
    let con = Constraints::new(&input.tris);
    let guard = Guard::new(&input.points, &input.tris);
    let mut extra: Vec<V3> = Vec::new();
    let (mut dt, mut flips, mut t_delaunay, mut t_recover);
    let mut attempt = 0;
    let mut bnd_steiner: usize;
    loop {
        let t1 = Instant::now();
        let mut pts = input.points.clone();
        pts.extend_from_slice(&far);
        pts.extend_from_slice(&extra);
        let mut ids: Vec<u32> = (0..n as u32).collect();
        ids.extend((n + N_FAR) as u32..pts.len() as u32);
        let order = crate::delaunay::insertion_order(&pts, &ids);
        let mut d = Dt::new(pts);
        let f0 = n as u32;
        // Corners 0, 3, 5, 6 of the far box form a tetrahedron.
        d.init([f0, f0 + 3, f0 + 5, f0 + 6])?;
        for k in [1usize, 2, 4, 7] {
            d.insert(f0 + k as u32)?;
        }
        for &p in &order {
            d.insert(p)?;
        }
        // Zero-volume tets of the perturbed Delaunay would make crossing tests degenerate.
        let _ = d.remove_flats(false, &guard)?;
        t_delaunay = ms(t1);
        let t2 = Instant::now();
        d.droppable_from = (n + N_FAR) as u32;
        let r = {
            let mut rec = Recover::new(&mut d, &con, &guard, max_steiner, n as u32, attempt);
            let mut r = if complete_only { Err(TetError::RecoveryFailed { tri: 0, edge: None, reason: String::new() }) } else { rec.run() };
            // A tiny body (a Schönhardt-like pocket) restarts with its centroid; anything else
            // that is stuck goes to complete recovery with temporary boundary Steiner points.
            if let Err(TetError::RecoveryFailed { tri, edge, .. }) = &r {
                let tiny = !complete_only && pocket_point(&input.points, &input.tris, *tri, *edge).is_some();
                if !(tiny && attempt < MAX_RESTARTS) {
                    r = rec.complete();
                }
            }
            bnd_steiner = rec.n_boundary_steiner;
            (r, rec.flips, rec.hints.clone())
        };
        t_recover = ms(t2);
        d.droppable_from = u32::MAX;
        d.search_limit = u64::MAX;
        match r.0 {
            Ok(()) => {
                dt = d;
                flips = r.1;
                break;
            }
            Err(TetError::RecoveryFailed { tri, edge, reason }) if attempt < MAX_RESTARTS && !complete_only => {
                // Add interior points and rebuild: the pocket centroid of a tiny body (the point
                // a non-star-shaped pocket needs), then the points the failed recoveries
                // suggested, best S4 score first (compliant points before the rest). A point on
                // an input triangle is useless and dropped.
                let before = extra.len();
                let cap = before + 32usize.max(n / 8);
                let pocket = pocket_point(&input.points, &input.tris, tri, edge);
                let mut scored: Vec<(f64, usize, V3)> = pocket
                    .iter()
                    .chain(r.2.iter())
                    .copied()
                    .enumerate()
                    .filter(|(_, p)| !input.points.contains(p) && guard.margin(*p) >= MIN_MARGIN)
                    .map(|(i, p)| (if i == 0 && pocket.is_some() { f64::INFINITY } else { guard.score(p) }, i, p))
                    .collect();
                scored.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
                for (_, _, h) in scored {
                    if extra.len() < cap && !extra.contains(&h) {
                        extra.push(h);
                    }
                }
                if extra.len() == before {
                    match steiner_near(&input.points, &input.tris, tri, edge, attempt, &guard) {
                        Some(p) if !extra.contains(&p) => extra.push(p),
                        _ => return Err(TetError::RecoveryFailed { tri, edge, reason }),
                    }
                }
                attempt += 1;
            }
            Err(e) => return Err(e),
        }
    }
    stage_ms[1] = t_delaunay;
    let t2 = Instant::now();
    let outside = classify(&dt, &con, &input.tris)?;
    dt.tds.remove(&outside);
    flips += dt.remove_flats(true, &guard)?;
    // Recovery points the final mesh can do without (a refill of their star without them).
    flips += dt.remove_points(n + N_FAR, dt.pts.len())?;
    flips += dt.delaunay_flips()?;
    flips += dt.flip_optimize(1e-3, 6)?;
    stage_ms[2] = ms(t2) + t_recover;

    // Stage 5: refinement. Points from here on are refinement points.
    let first_refine = dt.pts.len();
    let t3 = Instant::now();
    let mut smooth_passes = 0usize;
    let unrefinable;
    {
        let h = boundary_sizes(&input.points, &input.tris, dt.pts.len());
        let mut rf = crate::refine::Refiner::new(&mut dt, h, opts, &guard, max_steiner);
        unrefinable = rf.run()?;
        flips += rf.flips;
    }
    stage_ms[3] = ms(t3);
    // Stage 6: optimization.
    let t4 = Instant::now();
    if opts.optimize >= 1 {
        flips += dt.flip_optimize(0.7, 8)?;
    }
    if opts.optimize >= 2 {
        let first_movable = n + N_FAR;
        smooth_passes += dt.smooth(first_movable, 12, &guard);
        flips += dt.flip_optimize(0.7, 4)?;
        smooth_passes += dt.smooth(first_movable, 8, &guard);
        flips += dt.flip_optimize(0.7, 4)?;
    }
    stage_ms[4] = ms(t4);

    // Stage 7: canonical output.
    let mut tets_raw: Vec<[u32; 4]> = Vec::with_capacity(dt.tds.len());
    for t in 0..dt.tds.slots() as u32 {
        if dt.tds.alive[t as usize] {
            let v = dt.tds.verts[t as usize];
            if v.iter().any(|&x| (n as u32..n as u32 + N_FAR as u32).contains(&x)) {
                return Err(TetError::Internal("an interior tet uses an auxiliary point".into()));
            }
            tets_raw.push(v);
        }
    }
    for v in &tets_raw {
        let p = |i: usize| dt.pts[v[i] as usize];
        if ori(p(0), p(1), p(2), p(3)) <= 0 {
            return Err(TetError::Internal("flat or inverted tet in the final mesh".into()));
        }
    }
    let mut used = vec![false; dt.pts.len()];
    for v in &tets_raw {
        for &x in v {
            used[x as usize] = true;
        }
    }
    let mut remap = vec![u32::MAX; dt.pts.len()];
    let mut points: Vec<V3> = Vec::new();
    for i in 0..dt.pts.len() {
        let keep = i < n || (i >= n + N_FAR && used[i]);
        if keep {
            remap[i] = points.len() as u32;
            points.push(dt.pts[i]);
        }
    }
    let mut tets: Vec<[u32; 4]> = tets_raw.iter().map(|v| v.map(|x| remap[x as usize])).collect();
    canonical_tets(&mut tets);
    let (min_d, max_d, max_re, min_mr, mean_mr) = mesh_stats(&points, &tets);
    let n_steiner_recovery = (n + N_FAR..first_refine.min(dt.pts.len())).filter(|&i| used[i]).count();
    let stats = TetStats {
        n_steiner: points.len() - n,
        n_steiner_recovery,
        n_steiner_refine: points.len() - n - n_steiner_recovery,
        n_boundary_steiner: bnd_steiner,
        flips,
        smooth_passes,
        unrefinable,
        min_dihedral_deg: min_d,
        max_dihedral_deg: max_d,
        max_radius_edge: max_re,
        min_mean_ratio: min_mr,
        mean_mean_ratio: mean_mr,
        stage_ms,
    };
    Ok(TetMesh { points, n_input: n, tets, stats })
}
