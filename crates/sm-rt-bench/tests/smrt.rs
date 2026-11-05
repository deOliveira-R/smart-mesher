//! RTBENCH-B acceptance on the sm-rt rows: analytic truth (2), the twin bound (3, second half),
//! work counters (5), W-flight unbiasedness (6), determinism (7), fairness of the new rows and
//! the partial reporting of the competitors (9, 10). Work counters, hashes and analytic values
//! only; no timing is asserted.

use sm_core::{Point3, Vector3};
use sm_rt_api::{FlightInput, FlightOptions, ModePolicy, NodeId, Scratch};
use sm_rt_bench::adapter::{Adapter, FlightMode, Output, WorkInput, Workload};
use sm_rt_bench::bound::{compare_bounded, ray_bound};
use sm_rt_bench::cases::{Case, CaseData, PIN_H, PITCH, R_CLAD, R_FUEL, Scale, build};
use sm_rt_bench::formats::{Rays, RegionSpec};
use sm_rt_bench::result::{Env, ResultFile};
use sm_rt_bench::rng::SplitMix64;
use sm_rt_bench::runner::{RunConfig, run_case};
use sm_rt_bench::smrt::{analytic_from_scene, analytic_p2_bridge, flight_adapter, hierarchy_moderated, kuhn_box, smrt_rows};
use sm_rt_bench::truth::{Scene, Shape, Solid, Track};

fn input<'a>(rays: &'a Rays, regions: &'a [RegionSpec], threads: usize, histories: usize) -> WorkInput<'a> {
    WorkInput { rays, regions, threads, histories, seed: 17 }
}

fn spec(id: i32, name: &str, sigma_t: f64, sigma_a: f64) -> RegionSpec {
    RegionSpec { id, name: name.into(), sigma_t, sigma_a }
}

fn sum_region(t: &Track, r: i32) -> f64 {
    t.regions.iter().zip(&t.chords).filter(|(x, _)| **x == r).map(|(_, c)| c).sum()
}

/// A random horizontal-ish direction with horizontal speed `rho` and the angle `phi`.
fn dir3(phi: f64, dz: f64) -> [f64; 3] {
    let rho = (1.0 - dz * dz).sqrt();
    [rho * phi.cos(), rho * phi.sin(), dz]
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

// ---- acceptance 2: analytic truth ------------------------------------------------------------

/// Expected chord pieces of a ray through a pin of axis `c`: line at horizontal offset `b`, start
/// at horizontal arclength `s` from the foot of the perpendicular, horizontal speed `rho`.
fn pin_expectation(b: f64, s: f64, rho: f64) -> (f64, f64) {
    let hc = (R_CLAD * R_CLAD - b * b).sqrt();
    let hf = if b.abs() < R_FUEL { (R_FUEL * R_FUEL - b * b).sqrt() } else { 0.0 };
    let fuel = (hf - s.max(-hf)).max(0.0);
    let clad_and_fuel = hc - s;
    ((clad_and_fuel - fuel) / rho, fuel / rho)
}

#[test]
fn p1_analytic_chords_equal_the_closed_form_on_1e4_rays_and_sum_to_the_box_path() {
    let scale = Scale { rays: 200, ..Scale::reduced() };
    let d = build(Case::P1, &scale).unwrap();
    let mut a = analytic_from_scene(d.scene.as_ref().unwrap(), &d.manifest.regions).unwrap();
    let mut rng = SplitMix64::from_name("p1-analytic-truth");
    let mut rays = Rays::default();
    let mut want: Vec<(f64, f64, f64)> = Vec::new(); // fuel, clad, path to the box wall
    let half = 0.5 * PITCH;
    for _ in 0..10_000 {
        let theta = std::f64::consts::TAU * rng.unit();
        let o = [0.6 * theta.cos(), 0.6 * theta.sin(), 5.0 + 90.0 * rng.unit()];
        let b = (2.0 * rng.unit() - 1.0) * R_CLAD * 0.999;
        let alpha = (b / 0.6).asin();
        let dz = 0.4 * rng.unit() - 0.2;
        let u = dir3(theta + std::f64::consts::PI + alpha, dz);
        let rho = u[0].hypot(u[1]);
        let bb = (o[0] * u[1] - o[1] * u[0]).abs() / rho;
        // The start is outside the cylinder, on the incoming side: the foot is `ahead` in front.
        let ahead = -(o[0] * u[0] + o[1] * u[1]) / rho;
        // Path of the ray to the box wall.
        let mut t_exit = f64::INFINITY;
        for ax in 0..2 {
            if u[ax] > 0.0 {
                t_exit = t_exit.min((half - o[ax]) / u[ax]);
            } else if u[ax] < 0.0 {
                t_exit = t_exit.min((-half - o[ax]) / u[ax]);
            }
        }
        // Closed form of the chords: start at horizontal distance `ahead` before the foot.
        let hc = (R_CLAD * R_CLAD - bb * bb).sqrt();
        let hf = if bb < R_FUEL { (R_FUEL * R_FUEL - bb * bb).sqrt() } else { 0.0 };
        let fuel = 2.0 * hf / rho;
        let clad = 2.0 * (hc - hf) / rho;
        assert!(ahead > hc, "the start is outside the cladding");
        want.push((fuel, clad, t_exit));
        rays.push(o, u, 2);
    }
    let out = a.run(Workload::Track, &input(&rays, &d.manifest.regions, 1, 0)).unwrap();
    assert_eq!(out.counters.lost, 0);
    let (mut worst_chord, mut worst_sum) = (0.0f64, 0.0f64);
    for (i, t) in out.tracks.iter().enumerate() {
        let (fuel, clad, t_exit) = want[i];
        worst_chord = worst_chord.max(rel(sum_region(t, 0), fuel)).max(rel(sum_region(t, 1), clad));
        worst_sum = worst_sum.max(rel(t.chords.iter().sum::<f64>(), t_exit));
    }
    assert!(worst_chord < 1e-11, "chord vs 2 sqrt(r^2 - b^2) / rho: {worst_chord:e}");
    assert!(worst_sum < 1e-10, "chord sum vs the path to the box wall: {worst_sum:e}");
}

#[test]
fn p2_bridge_chords_equal_the_closed_form_on_the_17x17_assembly() {
    // 3 x 3 lattices of 17 x 17 pins (2601 pins): the bridged, flattened analytic tracker.
    let regions = [spec(0, "fuel", 1.0, 0.3), spec(1, "clad", 1.0, 0.3)];
    let scale = Scale { lattice_n: 17, ..Scale::tiny() };
    let mut a = analytic_p2_bridge(&scale, &regions).unwrap();
    let mut rng = SplitMix64::from_name("p2-bridge-truth");
    let mut rays = Rays::default();
    let mut want = Vec::new();
    while rays.len() < 10_000 {
        let (i, j) = ((rng.unit() * 51.0).floor().min(50.0), (rng.unit() * 51.0).floor().min(50.0));
        let b = (2.0 * rng.unit() - 1.0) * R_CLAD * 0.999;
        let phi = std::f64::consts::TAU * rng.unit();
        let dz = 0.4 * rng.unit() - 0.2;
        let u = dir3(phi, dz);
        let rho = u[0].hypot(u[1]);
        let (nx, ny) = (-u[1] / rho, u[0] / rho);
        let hc = (R_CLAD * R_CLAD - b * b).sqrt();
        let s = (2.0 * rng.unit() - 1.0) * 0.9 * hc;
        let (ux, uy) = (u[0] / rho, u[1] / rho);
        let o = [i * PITCH + b * nx + s * ux, j * PITCH + b * ny + s * uy, 30.0 + 40.0 * rng.unit()];
        let rho0 = (b * b + s * s).sqrt();
        if (rho0 - R_FUEL).abs() < 1e-5 {
            continue; // a start this close to the interface is inside the locate tolerance
        }
        let start = if rho0 < R_FUEL { 0 } else { 1 };
        let (clad, fuel) = pin_expectation(b, s, rho);
        want.push((fuel, clad, (hc - s) / rho));
        rays.push(o, u, start);
    }
    let out = a.run(Workload::Track, &input(&rays, &regions, 1, 0)).unwrap();
    assert_eq!(out.counters.lost, 0);
    let (mut worst_chord, mut worst_sum) = (0.0f64, 0.0f64);
    for (k, t) in out.tracks.iter().enumerate() {
        let (fuel, clad, path) = want[k];
        worst_chord = worst_chord.max(rel(sum_region(t, 0), fuel)).max(rel(sum_region(t, 1), clad));
        worst_sum = worst_sum.max(rel(t.chords.iter().sum::<f64>(), path));
    }
    assert!(worst_chord < 1e-11, "chord vs closed form: {worst_chord:e}");
    assert!(worst_sum < 1e-10, "chord sum vs the path out of the cladding: {worst_sum:e}");
}

// ---- acceptance 3, second half: geom_error of the exact run against the facet-twin run --------

fn twin_bound_check(case: Case) {
    let scale = Scale { rays: 2000, ..Scale::reduced() };
    let d = build(case, &scale).unwrap();
    let mut rows = smrt_rows(&d, &scale, 12);
    let mut tracks = Vec::new();
    for a in rows.geometric.iter_mut().filter(|a| matches!(a.mode().as_deref(), Some("analytic") | Some("facets"))) {
        tracks.push((a.mode().unwrap(), a.run(Workload::Track, &input(&d.rays, &d.manifest.regions, 1, 0)).unwrap().tracks));
    }
    assert_eq!(tracks.len(), 2, "{:?}", rows.notes);
    let scene = d.scene.as_ref().unwrap();
    let bounds: Vec<_> = (0..d.rays.len()).map(|i| ray_bound(scene, d.rays.origin[i], d.rays.dir[i], d.manifest.sag)).collect();
    let (exact, twin) = if tracks[0].0 == "analytic" { (&tracks[0].1, &tracks[1].1) } else { (&tracks[1].1, &tracks[0].1) };
    let rep = compare_bounded(exact, twin, &bounds, d.manifest.diag);
    assert!(rep.checked > 1500, "{rep:?}");
    assert_eq!(rep.failed, 0, "{}: {rep:?} (first failing ray {:?})", d.case, rep.first_failed);
    assert!(rep.max_ratio <= 1.0, "{rep:?}");
    println!("{}: {} judged, {} grazing, max error / bound {:.3}", d.case, rep.checked, rep.grazing, rep.max_ratio);
}

#[test]
fn p1_exact_run_agrees_with_the_facet_twin_run_within_the_per_ray_bound() {
    twin_bound_check(Case::P1);
}

#[test]
fn p2_exact_run_agrees_with_the_facet_twin_run_within_the_per_ray_bound() {
    twin_bound_check(Case::P2);
}

// ---- acceptance 5: work counters ---------------------------------------------------------------

/// Elements of the `n x n` grid of pitch `PITCH` (lower corner at `-PITCH/2`) that the ray's
/// horizontal projection crosses before it leaves the grid or the z range: an Amanatides-Woo walk
/// over cells, independent of any tracker.
fn dda_cells(o: [f64; 3], d: [f64; 3], n: i64) -> u64 {
    let lo = -0.5 * PITCH;
    let hi = lo + n as f64 * PITCH;
    // Parameter interval inside the grid box and the z slab.
    let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
    for ax in 0..3 {
        let (l, h) = if ax == 2 { (0.0, PIN_H) } else { (lo, hi) };
        if d[ax].abs() < 1e-300 {
            if o[ax] < l || o[ax] > h {
                return 0;
            }
        } else {
            let (a, b) = ((l - o[ax]) / d[ax], (h - o[ax]) / d[ax]);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
        }
    }
    if t0 >= t1 {
        return 0;
    }
    let p = [o[0] + t0 * d[0], o[1] + t0 * d[1]];
    let mut cell = [((p[0] - lo) / PITCH).floor().clamp(0.0, n as f64 - 1.0) as i64, ((p[1] - lo) / PITCH).floor().clamp(0.0, n as f64 - 1.0) as i64];
    let mut t = t0;
    let mut count = 1u64;
    let step = [if d[0] > 0.0 { 1 } else { -1 }, if d[1] > 0.0 { 1 } else { -1 }];
    let next = |ax: usize, c: i64| -> f64 {
        if d[ax].abs() < 1e-300 {
            return f64::INFINITY;
        }
        let edge = lo + (c + i64::from(step[ax] > 0)) as f64 * PITCH;
        (edge - o[ax]) / d[ax]
    };
    loop {
        let (tx, ty) = (next(0, cell[0]), next(1, cell[1]));
        let (tn, ax) = if tx < ty { (tx, 0) } else { (ty, 1) };
        if tn >= t1 || !tn.is_finite() {
            break;
        }
        t = tn;
        cell[ax] += step[ax];
        if cell[ax] < 0 || cell[ax] >= n {
            break;
        }
        count += 1;
    }
    let _ = t;
    count
}

#[test]
fn track_counters_on_p2_segments_and_elements_touched_per_track() {
    let scale = Scale { rays: 400, ..Scale::reduced() };
    let d = build(Case::P2, &scale).unwrap();
    let mut rows = smrt_rows(&d, &scale, 12);
    // Segments per track in surface mode: the facet walker and the analytic tracker.
    for a in rows.geometric.iter_mut() {
        let out = a.run(Workload::Track, &input(&d.rays, &d.manifest.regions, 1, 0)).unwrap();
        assert!(out.counters.segments as usize >= out.tracks.iter().map(|t| t.regions.len()).sum::<usize>(), "{:?}", a.mode());
        assert!(out.counters.segments as usize >= d.rays.len(), "{:?}: at least one segment per ray", a.mode());
    }
    // Elements touched in HIERARCHY mode against the independent grid walk. P2's own twin has no
    // moderator (every ray ends in its first pin), so the moderated pin cell fills the same lattice.
    let n = 3 * scale.lattice_n as i64;
    let mut h = hierarchy_moderated(&scale).unwrap();
    let mut rng = SplitMix64::from_name("p2-touched");
    let mut rays = Rays::default();
    let span = n as f64 * PITCH;
    for _ in 0..400 {
        let o = [-0.5 * PITCH + 0.999 * span * rng.unit() + 0.0005 * span, -0.5 * PITCH + 0.999 * span * rng.unit() + 0.0005 * span, 5.0 + 90.0 * rng.unit()];
        let dir = SplitMix64::new(rng.next_u64()).direction();
        rays.push(o, dir, 2);
    }
    let out = h.run(Workload::Track, &input(&rays, &d.manifest.regions, 1, 0)).unwrap();
    let geometric: u64 = (0..rays.len()).map(|i| dda_cells(rays.origin[i], rays.dir[i], n)).sum();
    assert!(out.counters.pins_touched >= rays.len() as u64 - out.counters.lost, "each ray touches its start element");
    assert!(out.counters.pins_touched > rays.len() as u64, "rays cross elements: {}", out.counters.pins_touched);
    assert!(out.counters.pins_touched <= geometric, "touched {} against the grid walk {geometric}", out.counters.pins_touched);
    println!("elements touched {} of {geometric} grid cells on {} rays", out.counters.pins_touched, rays.len());
    // The same chords as the flat facet twin of the moderated lattice would give: the world run
    // ends where the assembly ends, so chords sum to the path to the lattice box.
    for (i, t) in out.tracks.iter().enumerate() {
        let (o, dd) = (rays.origin[i], rays.dir[i]);
        let (lo, hi) = (-0.5 * PITCH, -0.5 * PITCH + span);
        let mut t_exit = f64::INFINITY;
        for ax in 0..3 {
            let (l, hh) = if ax == 2 { (0.0, PIN_H) } else { (lo, hi) };
            if dd[ax] > 0.0 {
                t_exit = t_exit.min((hh - o[ax]) / dd[ax]);
            } else if dd[ax] < 0.0 {
                t_exit = t_exit.min((l - o[ax]) / dd[ax]);
            }
        }
        assert!((t.chords.iter().sum::<f64>() - t_exit).abs() < 1e-8 * t_exit.max(1.0), "ray {i}");
    }
}

fn h1_delta(scale: &Scale) -> (CaseData, sm_rt_bench::adapter::TrackerAdapter) {
    let d = build(Case::H1, scale).unwrap();
    let a = flight_adapter(d.tets.as_ref().unwrap(), FlightMode::Delta, 8).unwrap();
    (d, a)
}

#[test]
fn delta_flights_report_rejections_and_a_homogeneous_interior_needs_no_surface_work() {
    let scale = Scale { rays: 300, ..Scale::reduced() };
    let (d, mut a) = h1_delta(&scale);
    let out = a.run(Workload::Flight, &input(&d.rays, &d.manifest.regions, 1, 600)).unwrap();
    let c = out.counters;
    assert_eq!(c.histories, 600);
    assert!(c.delta_segments > 0 && c.steps >= c.delta_segments, "{c:?}");
    assert!(c.steps as f64 / c.histories as f64 >= 1.0, "fires (segments) per history");
    // Rejected (virtual) collisions need a node that holds regions of different sigma: P1's
    // fuel, clad and water meet inside leaves.
    let p1 = build(Case::P1, &Scale { rays: 300, ..Scale::reduced() }).unwrap();
    let tets = sm_rt_bench::smrt::tet_twin(&p1, 12).unwrap().unwrap();
    let mut pd = flight_adapter(&tets, FlightMode::Delta, 8).unwrap();
    let pc = pd.run(Workload::Flight, &input(&p1.rays, &p1.manifest.regions, 1, 2000)).unwrap().counters;
    assert!(pc.rejected > 0 && pc.tentative >= pc.rejected, "{pc:?}");
    assert!(pc.collisions > 0 && pc.delta_segments > 0, "{pc:?}");
    // A flight that starts and ends inside the water, no inclusion on the path: the delta flight
    // does no face test and no BVH descent beyond the one that located its start.
    let t = sm_rt_bench::smrt::tracker_of(&a);
    let view = t.majorant_view().expect("partition");
    let water = d.manifest.regions.iter().find(|r| r.name == "water").map(|r| r.id).expect("water region");
    let sigma: Vec<f64> = t.regions().0.iter().map(|r| d.manifest.regions.iter().find(|x| x.name == r.name).map_or(0.0, |x| x.sigma_t)).collect();
    let _ = water;
    let h = view.hierarchy();
    let maj: Vec<f64> = (0..h.node_count())
        .map(|n| {
            let info = h.node(NodeId::new(n)).unwrap();
            h.set_regions(info.set).unwrap().iter().map(|r| sigma[r.get()]).fold(0.0, f64::max)
        })
        .collect();
    // Face tests of a flight of `dist` from the cube's centre along the diagonal, in `policy`.
    let flight = |policy: ModePolicy, maj: &[f64], dist: f64| -> (u64, u64, f64, usize) {
        let mut sc = Scratch::new();
        let mut st = t.init_state(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0) * (1.0 / 3f64.sqrt()), None, &mut sc).unwrap();
        let before = sc.counters;
        let opts = FlightOptions::default().with_policy(policy).with_max_dist(dist);
        let input = FlightInput::new(maj);
        let mut cur = view.begin(&st, &opts).unwrap();
        let (mut flown, mut segs) = (0.0, 0usize);
        while let Some(seg) = cur.next_segment(view, input, &mut st, &mut sc).unwrap() {
            flown += seg.length();
            segs += 1;
        }
        let after = sc.counters;
        (after.face_tests - before.face_tests, after.bvh_descents - before.bvh_descents, flown, segs)
    };
    // Nothing but water within 17 cm of the centre along the diagonal (the inclusions start at 20).
    let delta: Vec<_> = [5.0, 15.0, 30.0].iter().map(|&d| flight(ModePolicy::Delta, &maj, d)).collect();
    for (&d, f) in [5.0, 15.0, 30.0].iter().zip(&delta) {
        assert!((f.2 - d).abs() < 1e-9, "the flight ends at max_dist: {f:?}");
        assert_eq!(f.1, 0, "no BVH descent after the start: {f:?}");
    }
    // Delta flights do the same few face tests whatever their length (the containment test of
    // the end point's tet: at most its four faces), not one per cell crossed.
    assert!(delta.iter().all(|f| f.0 == delta[0].0 && f.0 <= 4), "{delta:?}");
}

// ---- acceptance 6: W-flight unbiasedness ---------------------------------------------------------

const SIGMA: f64 = 0.3;
const R_SPHERE: f64 = 5.0;

/// `n` starts uniform in the ball of radius `R_SPHERE`, isotropic directions.
fn ball_rays(n: usize) -> Rays {
    let mut rng = SplitMix64::from_name("absorber-sphere");
    let mut rays = Rays::default();
    while rays.len() < n {
        let p = [(2.0 * rng.unit() - 1.0) * R_SPHERE, (2.0 * rng.unit() - 1.0) * R_SPHERE, (2.0 * rng.unit() - 1.0) * R_SPHERE];
        if p.iter().map(|x| x * x).sum::<f64>() < R_SPHERE * R_SPHERE {
            rays.push(p, rng.direction(), 0);
        }
    }
    rays
}

fn assert_transmitted(what: &str, out: &Output, p: &[f64], nreg: usize) {
    let n = p.len() as f64;
    let mean: f64 = p.iter().sum();
    let var: f64 = p.iter().map(|q| q * (1.0 - q)).sum();
    let leaked = out.tally[2 * nreg];
    let absorbed = out.tally[2 * nreg + 1];
    assert_eq!(leaked + absorbed, n, "{what}: every history ends");
    assert_eq!(out.counters.lost, 0, "{what}");
    assert!((leaked - mean).abs() <= 4.0 * var.sqrt(), "{what}: transmitted {leaked} vs {mean} +- {} (4 sigma = {})", var.sqrt(), 4.0 * var.sqrt());
}

fn unbiasedness(n: usize) {
    let rays = ball_rays(n);
    let regions = [spec(0, "absorber", SIGMA, SIGMA), spec(1, "air", 0.0, 0.0)];
    // Exact geometry: the surface flight of the analytic tracker against exp(-Sigma d) averaged
    // over the starts (d: distance to the sphere, analytic).
    let scene = Scene::new(vec![Solid { shape: Shape::Sphere { c: [0.0; 3], r: R_SPHERE }, region: 0, depth: 0 }]);
    let exact: Vec<f64> = (0..n)
        .map(|i| {
            let (o, d) = (rays.origin[i], rays.dir[i]);
            let od: f64 = (0..3).map(|a| o[a] * d[a]).sum();
            let oo: f64 = (0..3).map(|a| o[a] * o[a]).sum();
            let dist = -od + (od * od + R_SPHERE * R_SPHERE - oo).sqrt();
            (-SIGMA * dist).exp()
        })
        .collect();
    let mut analytic = analytic_from_scene(&scene, &regions[..1]).unwrap();
    assert_transmitted("analytic surface", &analytic.run(Workload::Flight, &input(&rays, &regions[..1], 2, n)).unwrap(), &exact, 1);
    // Tet twin (stair-stepped sphere): surface walk, pure delta and the blocked mix against
    // exp(-Sigma L) with L the chord the same mesh gives each start.
    let tets = kuhn_box([-8.0; 3], [8.0; 3], [16; 3], &|c| i32::from(c.iter().map(|x| x * x).sum::<f64>() >= R_SPHERE * R_SPHERE)).unwrap();
    let mut walk = flight_adapter(&tets, FlightMode::Surface, 8).unwrap();
    let tracks = walk.run(Workload::Track, &input(&rays, &regions, 2, 0)).unwrap().tracks;
    let mesh_expect: Vec<f64> = tracks.iter().map(|t| (-SIGMA * sum_region(t, 0)).exp()).collect();
    for mode in [FlightMode::Surface, FlightMode::Delta, FlightMode::Block] {
        let mut a = flight_adapter(&tets, mode, 8).unwrap();
        let out = a.run(Workload::Flight, &input(&rays, &regions, 2, n)).unwrap();
        assert_transmitted(mode.name(), &out, &mesh_expect, 2);
    }
}

#[test]
fn absorber_sphere_transmission_is_unbiased_in_surface_delta_and_block_modes() {
    unbiasedness(200_000);
}

#[test]
#[ignore = "1e6 histories (the acceptance size); run with --ignored --release"]
fn absorber_sphere_transmission_is_unbiased_at_1e6_histories() {
    unbiasedness(1_000_000);
}

#[test]
fn p1_collision_tallies_of_walk_delta_and_block_agree_within_4_sigma() {
    let scale = Scale { rays: 500, ..Scale::reduced() };
    let d = build(Case::P1, &scale).unwrap();
    let mut rows = smrt_rows(&d, &scale, 12);
    assert_eq!(rows.mesh_flight.len(), 3, "{:?}", rows.notes);
    let cfg = RunConfig { reps: 1, warmup: 0, threads: vec![1, 3], workloads: vec![Workload::Flight], histories: 40_000, seed: 5 };
    let (f, _) = run_case(&d, &mut rows.mesh_flight, "surface", &cfg, Env::detect()).unwrap();
    let modes: Vec<&str> = f.rows.iter().map(|r| r.mode.as_str()).collect();
    assert!(modes.contains(&"walk") && modes.contains(&"delta") && modes.contains(&"block"), "{modes:?}");
    for r in f.rows.iter().filter(|r| r.mode != "walk") {
        assert!(r.agreement.checked >= 3, "{}: {:?}", r.mode, r.agreement);
        assert_eq!(r.agreement.failed, 0, "{} at {} threads: {:?}", r.mode, r.threads, r.agreement);
        assert_eq!(r.lost, 0);
    }
    assert!(f.rows.iter().all(|r| r.determinism.stable == Some(true)), "1 and 3 threads hash equal");
}

// ---- acceptance 7: determinism ---------------------------------------------------------------------

#[test]
fn smrt_track_and_flight_hashes_are_equal_for_one_and_n_threads_and_two_runs() {
    for case in [Case::P1, Case::P2, Case::H1] {
        let scale = Scale { rays: 300, ..Scale::reduced() };
        let d = build(case, &scale).unwrap();
        let h = sm_testkit::determinism::check(2, 4, |t| {
            let mut rows = smrt_rows(&d, &scale, 10);
            let mut hashes: Vec<String> = Vec::new();
            for a in rows.geometric.iter_mut() {
                hashes.push(a.run(Workload::Track, &input(&d.rays, &d.manifest.regions, t, 0)).unwrap().content_hash());
            }
            for a in rows.exact_flight.iter_mut().chain(rows.mesh_flight.iter_mut()) {
                hashes.push(a.run(Workload::Flight, &input(&d.rays, &d.manifest.regions, t, 1500)).unwrap().content_hash());
            }
            hashes.join(" ")
        })
        .unwrap();
        assert!(h.len() >= 64, "{}: {h}", d.case);
    }
}

fn args(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

fn read_results(dir: &std::path::Path) -> Vec<ResultFile> {
    let mut paths: Vec<_> = std::fs::read_dir(dir).unwrap().map(|e| e.unwrap().path()).filter(|p| p.extension().is_some_and(|x| x == "json") && p.file_name().is_some_and(|n| n != "bench-env.json")).collect();
    paths.sort();
    paths.iter().map(|p| ResultFile::from_json(&std::fs::read_to_string(p).unwrap()).unwrap()).collect()
}

#[test]
fn two_smrt_runs_give_one_result_hash_and_date_and_sequence_stay_out_of_it() {
    let tmp = std::env::temp_dir().join(format!("smrt-bench-hash-{}", std::process::id()));
    let mut hashes = Vec::new();
    for k in 0..2 {
        let out = tmp.join(format!("run{k}"));
        let code = sm_rt_bench::cli::main_with(&args(&["run", "--cases", "P1,H1", "--trackers", "smrt", "--scale", "reduced", "--rays", "300", "--reps", "1", "--warmup", "0", "--histories", "1500", "--out", out.to_str().unwrap()])).unwrap();
        assert_eq!(code, 0);
        let files = read_results(&out);
        assert_eq!(files.len(), 2);
        assert!(files.iter().all(|f| f.rows.iter().all(|r| r.tracker == "smrt")));
        assert!(files.iter().all(|f| f.rows.iter().all(|r| r.determinism.stable == Some(true))));
        hashes.push(files.iter().map(|f| f.result_hash.clone()).collect::<Vec<_>>());
        for f in &files {
            assert_eq!(f.compute_hash().unwrap(), f.result_hash);
        }
    }
    assert_eq!(hashes[0], hashes[1], "`run --trackers smrt --reps 1` twice: equal result_hash");
    // The date and the write counter are not in any hashed output: another date, same hash.
    let mut f = read_results(&tmp.join("run0")).remove(0);
    let want = f.result_hash.clone();
    f.env.date_utc = "2031-02-03T04:05:06Z".into();
    f.git_sha = "other".into();
    assert_eq!(f.compute_hash().unwrap(), want);
    // Two writes in one second get different names (counter), the same hash.
    let (p1, p2) = (sm_rt_bench::result::write_result(&tmp.join("names"), &f).unwrap(), sm_rt_bench::result::write_result(&tmp.join("names"), &f).unwrap());
    assert_ne!(p1, p2);
    let (g1, g2) = (ResultFile::from_json(&std::fs::read_to_string(&p1).unwrap()).unwrap(), ResultFile::from_json(&std::fs::read_to_string(&p2).unwrap()).unwrap());
    assert_eq!(g1.result_hash, g2.result_hash);
    // Row outputs of the adapters do not mention a date or a counter either.
    let text = std::fs::read_to_string(&p1).unwrap();
    let stem = p1.file_stem().unwrap().to_string_lossy().to_string();
    assert!(!text.contains(&stem) && !text.contains(&format!("-{}-", std::process::id())), "the file name (date, pid, counter) is not inside the file");
    std::fs::remove_dir_all(&tmp).unwrap();
}

// ---- fairness of the new rows and the competitors (9, 10) ---------------------------------------------

#[test]
fn the_report_labels_macos_rows_smoke_lists_partial_rows_and_claims_no_ratio() {
    let tmp = std::env::temp_dir().join(format!("smrt-bench-report-{}", std::process::id()));
    let code = sm_rt_bench::cli::main_with(&args(&["run", "--cases", "P1", "--trackers", "smrt,dagmc-moab,dagmc-dd,xdg", "--scale", "reduced", "--rays", "200", "--reps", "1", "--warmup", "0", "--histories", "800", "--out", tmp.to_str().unwrap()])).unwrap();
    let files = read_results(&tmp);
    assert_eq!(files.len(), 1);
    let f = &files[0];
    // Without a competitor image every competitor is reported partial with its reason, and no
    // row of it exists; where an image runs, its rows are in the file instead.
    for t in ["dagmc-moab", "dagmc-dd", "xdg"] {
        let has_rows = f.rows.iter().any(|r| r.tracker == t);
        let partial = f.partial.iter().find(|p| p.starts_with(&format!("{t}:")));
        assert!(has_rows != partial.is_some(), "{t}: rows xor partial; partial = {:?}", f.partial);
        if let Some(p) = partial {
            assert!(p.contains("docker") || p.contains("`"), "the failing command is named: {p}");
        }
    }
    assert_eq!(code, if f.partial.is_empty() { 0 } else { 2 });
    let md = sm_rt_bench::report::render(&files).unwrap();
    assert_eq!(md.contains("SMOKE RESULT"), Env::detect().smoke());
    if Env::detect().smoke() {
        assert!(md.contains("smrt (smoke)"), "rows are labelled smoke");
    }
    if !f.partial.is_empty() {
        assert!(md.contains("Partial (not run, nothing invented)"));
    }
    assert!(!md.to_lowercase().contains("speedup"));
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn run_competitor_without_an_image_is_partial_with_the_command() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let bad = sm_rt_bench::docker::run_competitor(root, "no-such-tracker", root, "P1", &[1]);
    assert!(matches!(bad, sm_rt_bench::docker::Competitor::Partial(_)));
    match sm_rt_bench::docker::run_competitor(root, "xdg", root, "P1", &[1]) {
        sm_rt_bench::docker::Competitor::Partial(why) => assert!(why.contains('`') || why.contains("pins.env"), "{why}"),
        sm_rt_bench::docker::Competitor::Rows(r) => assert!(!r.is_empty()),
    }
}
