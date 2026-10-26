//! The surface-to-volume pipeline: validate, Delaunay, recover, classify, refine, optimize,
//! canonical output.

use crate::canon::canonical_tets;
use crate::delaunay::{Dt, splitmix};
use crate::error::TetError;
use crate::geom::{V3, coord_ok, ori};
use crate::quality::mesh_stats;
use crate::recover::{Constraints, Recover};
use crate::tds::{NONE, same_cycle, sorted3};
use crate::{SurfaceInput, TetMesh, TetOptions, TetStats, validate};
use std::time::Instant;

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
    if let crate::SizeSpec::Constant(h) = o.size {
        if !(h.is_finite() && h > 0.0) {
            return Err(TetError::BadOption(format!("size = {h} must be finite and positive")));
        }
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
    let mut stage_ms = [0.0f64; 5];
    let t0 = Instant::now();
    check_options(opts)?;
    validate::check_surface(&input.points, &input.tris)?;
    stage_ms[0] = ms(t0);
    let n = input.points.len();
    let max_steiner = if opts.max_steiner == 0 { 8 * n + 100_000 } else { opts.max_steiner };

    // Stage 2: Delaunay of the input plus auxiliary far points.
    let t1 = Instant::now();
    let far = far_points(&input.points)?;
    let mut pts = input.points.clone();
    pts.extend_from_slice(&far);
    let order = crate::delaunay::insertion_order(&pts, &(0..n as u32).collect::<Vec<_>>());
    let mut dt = Dt::new(pts);
    let f0 = n as u32;
    // Corners 0, 3, 5, 6 of the far box form a tetrahedron.
    dt.init([f0, f0 + 3, f0 + 5, f0 + 6])?;
    for k in [1usize, 2, 4, 7] {
        dt.insert(f0 + k as u32)?;
    }
    for &p in &order {
        dt.insert(p)?;
    }
    stage_ms[1] = ms(t1);

    let dbg = std::env::var("TET_DEBUG").is_ok();
    let count_flat = |dt: &Dt| -> usize {
        (0..dt.tds.slots() as u32)
            .filter(|&t| dt.tds.alive[t as usize] && dt.tds.ghost_slot(t).is_none())
            .filter(|&t| {
                let v = dt.tds.verts[t as usize];
                ori(dt.pts[v[0] as usize], dt.pts[v[1] as usize], dt.pts[v[2] as usize], dt.pts[v[3] as usize]) <= 0
            })
            .count()
    };
    if dbg {
        eprintln!("flat tets after Delaunay: {} of {}", count_flat(&dt), dt.tds.len());
    }
    // Stage 3 + 4: recovery and classification.
    let t2 = Instant::now();
    let con = Constraints::new(&input.tris);
    let (mut flips, mut n_steiner);
    {
        let mut rec = Recover::new(&mut dt, &con, max_steiner, n as u32);
        rec.run()?;
        flips = rec.flips;
        n_steiner = rec.n_steiner;
    }
    if dbg {
        eprintln!("flat tets after recovery: {} of {}", count_flat(&dt), dt.tds.len());
    }
    let outside = classify(&dt, &con, &input.tris)?;
    dt.tds.remove(&outside);
    if dbg {
        eprintln!("flat tets inside: {} of {}", count_flat(&dt), dt.tds.len());
    }
    flips += dt.remove_flats()?;
    flips += dt.delaunay_flips()?;
    flips += dt.flip_optimize(1e-3, 6)?;
    stage_ms[2] = ms(t2);

    // Stage 5 + 6 (filled in by the refine / optimize modules).
    let t3 = Instant::now();
    let mut unrefinable = 0usize;
    let mut smooth_passes = 0usize;
    {
        let h = boundary_sizes(&input.points, &input.tris, dt.pts.len());
        let mut rf = crate::refine::Refiner::new(&mut dt, h, opts, max_steiner, n);
        unrefinable = rf.run()?;
        if dbg {
            eprintln!("refine: {} inserted, {} unrefinable, failure reasons {:?}", rf.n_steiner, unrefinable, rf.reasons);
        }
        n_steiner += rf.n_steiner;
        flips += rf.flips;
    }
    let _ = (&mut flips, &mut smooth_passes);
    stage_ms[3] = ms(t3);
    let t4 = Instant::now();
    if opts.optimize >= 1 {
        flips += dt.flip_optimize(0.7, 8)?;
    }
    if opts.optimize >= 2 {
        let first_movable = n + N_FAR;
        smooth_passes += dt.smooth(first_movable, 12);
        flips += dt.flip_optimize(0.7, 4)?;
        smooth_passes += dt.smooth(first_movable, 8);
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
    let stats = TetStats {
        n_steiner: points.len() - n,
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
