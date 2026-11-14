//! W4-RT-GRAZE: the grazing-trap bound is shared by every walk loop, and a legitimate run of tiny
//! steps (a ray along the axis of a many-sector pin) is carried through.
//!
//! Part 1 drives a hex lattice of mesh prisms (so the majorant partition exists) with rays that
//! graze a hex face plane and the z-wall between two layers, through `trace`, `fire` and the
//! surface and delta flights; each must end in a bounded number of steps, carried through or in
//! the typed error. Part 2 is a CSG pin of 16 sectors in a hex lattice.
#[path = "common/mod.rs"]
mod common;
#[path = "common/csg.rs"]
mod csg;
#[path = "common/hex.rs"]
mod hex;

use csg::*;
use sm_core::{Aabb, Point3, Transform, Vector3};
use sm_geom_api::HalfSpace;
use sm_rt::{MeshTrackerBuilder, WorldTracker};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, ComponentId, ComponentKind, CsgLeafInput, FlightInput, FlightOptions, Flow, HierInput, InstancePolicy, Lattice,
    LatticeKind, LeafInput, LostCause, MeshInput, ModePolicy, PartitionOptions, Placement, RegionRule, Scratch, Segment, StepEnd, TraceEnd, TraceOptions, Tracker, WorldSpec,
};
use sm_testkit::meshes::hexagonal_prism;

const P: f64 = 1.5;
const H: f64 = 2.0;

// ---------------------------------------------------------------- part 1: mesh prisms

/// An `n x n x nz` lattice (flat-top, `HexYRows`) of mesh hexagonal prisms (vertices at 0 degrees,
/// flat to flat `P`, height `H`), element `(0, 0, 0)` centred at the origin, with a partition.
fn mesh_world(n: u32, nz: u32) -> WorldTracker {
    let prism = hexagonal_prism(P / 3f64.sqrt(), H).expect("prism");
    let (rules, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let leaves = [LeafInput::Mesh(MeshInput::new(&prism.mesh, &rules, &bnd, &comp))];
    let lat = Lattice::new(LatticeKind::HexYRows { pitch: P, z_pitch: H }, p3(0.0, 0.0, 0.0), [n, n, nz], vec![ComponentId(1); (n * n * nz) as usize], None);
    let spec = WorldSpec::new(
        vec![
            ComponentKind::Leaf { input: 0 },
            ComponentKind::Union { placements: vec![Placement::new(ComponentId(0), Transform::translation(Vector3::new(0.0, 0.0, -0.5 * H)))] },
            ComponentKind::Lattice(lat),
        ],
        ComponentId(2),
        InstancePolicy::Hierarchical,
    );
    let mut part = PartitionOptions::default();
    part.enabled = true;
    part.leaf_cells = 4;
    let opts = BuildOptions::default().with_partition(part);
    MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &opts).expect("mesh hex world builds").0
}

/// How a walk ended and the steps it took.
#[derive(Debug)]
struct Walk {
    end: String,
    length: f64,
    steps: u64,
}

fn end_name(e: &TraceEnd) -> String {
    match e {
        TraceEnd::Lost(l) => format!("lost {:?}", l.cause),
        other => format!("{other:?}").split([' ', '{']).next().unwrap_or("").to_owned(),
    }
}

fn walk_trace(w: &dyn Tracker, o: Point3, d: Vector3) -> Walk {
    let mut sc = Scratch::new();
    let mut st = w.init_state(o, d, None, &mut sc).expect("init");
    assert!(!st.cell.is_none(), "start inside the model");
    let mut length = 0.0;
    let end = w
        .trace(
            &mut st,
            &TraceOptions::default(),
            &mut |s: &Segment| {
                length += s.length;
                Flow::Continue
            },
            &mut sc,
        )
        .expect("trace");
    Walk { end: end_name(&end), length, steps: sc.counters.steps }
}

fn walk_fire(w: &dyn Tracker, o: Point3, d: Vector3) -> Walk {
    let mut sc = Scratch::new();
    let mut st = w.init_state(o, d, None, &mut sc).expect("init");
    let r = w.fire(&mut st, f64::INFINITY, 100_000, &mut sc).expect("fire");
    let end = match r.end {
        StepEnd::Lost(l) => format!("lost {:?}", l.cause),
        StepEnd::Boundary { .. } => "Boundary".to_owned(),
        e => format!("{e:?}"),
    };
    Walk { end, length: r.dist, steps: sc.counters.steps }
}

/// A flight with zero majorants (segments never collide) in `mode`.
fn walk_flight(w: &dyn Tracker, mode: ModePolicy, o: Point3, d: Vector3) -> Walk {
    let mut sc = Scratch::new();
    let view = w.majorant_view().expect("partition");
    let maj = vec![0.0; view.hierarchy().node_count()];
    let mut st = w.init_state(o, d, None, &mut sc).expect("init");
    assert!(!st.cell.is_none(), "start inside the model");
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode)).expect("begin");
    let mut length = 0.0;
    let mut n = 0;
    while let Some(seg) = cur.next_segment(view, FlightInput::new(&maj), &mut st, &mut sc).expect("segment") {
        length = seg.t1;
        n += 1;
        assert!(n < 10_000, "a flight that does not end");
    }
    let end = cur.end().map_or("none".to_owned(), end_name);
    Walk { end, length, steps: sc.counters.steps }
}

/// The grazing rays of a `3 x 3 x 2` world: `(name, origin, direction)`. Element `(0, 0, 0)` spans
/// `z` in `[-1, 1]`, the second layer `[1, 3]`; flats have normals at `30 + 60 k` degrees.
fn rays() -> Vec<(String, Point3, Vector3)> {
    let mut v = Vec::new();
    // In the z-wall between the layers, in several directions.
    for (k, a) in [0.3f64, 1.0, 2.2, 4.0].into_iter().enumerate() {
        v.push((format!("zwall {k}"), p3(0.2, 0.1, 1.0), v3(a.cos(), a.sin(), 0.0)));
    }
    // In the flat of normal 30 degrees of element (0,0,0) (it faces element (1,0,0)): the flat is
    // the line `x cos30 + y sin30 = P/2`, direction 120 degrees, and a hair off it.
    let n = 30f64.to_radians();
    let a = 120f64.to_radians();
    let c = 0.5 * P;
    for (k, eps) in [0.0, 1e-13, -1e-10].into_iter().enumerate() {
        let o = p3(c * n.cos() - 0.1 * a.cos(), c * n.sin() - 0.1 * a.sin(), 0.2);
        v.push((format!("flat {k}"), o, v3((a + eps).cos(), (a + eps).sin(), 0.0)));
    }
    // In the flat and in the z-wall: both planes at once.
    for (k, dz) in [0.0, 1e-12].into_iter().enumerate() {
        let o = p3(c * n.cos() - 0.1 * a.cos(), c * n.sin() - 0.1 * a.sin(), 1.0);
        v.push((format!("flat+wall {k}"), o, v3(a.cos(), a.sin(), dz)));
    }
    v
}

#[test]
fn grazing_rays_end_in_bounded_steps_in_every_loop() {
    let w = mesh_world(3, 2);
    let mut worst = 0u64;
    for (name, o, d) in rays() {
        let runs: Vec<(&str, Walk)> = vec![
            ("trace", walk_trace(&w, o, d)),
            ("fire", walk_fire(&w, o, d)),
            ("surface flight", walk_flight(&w, ModePolicy::Surface, o, d)),
            ("delta flight", walk_flight(&w, ModePolicy::Delta, o, d)),
        ];
        let reference = walk_trace(&w, o, d);
        for (what, r) in runs {
            eprintln!("{name} {what}: {r:?}");
            let typed = r.end == format!("lost {:?}", LostCause::RecoveryFailed);
            let carried = r.end == "Boundary" || r.end == "LeftModel";
            assert!(carried || typed, "{name} {what}: ended {:?}", r.end);
            // At most 6 elements crossed, a few steps each; a trap that ran to max_steps is 10^5.
            assert!(r.steps <= 400, "{name} {what}: {} steps", r.steps);
            worst = worst.max(r.steps);
            if carried && reference.end == r.end {
                assert!((r.length - reference.length).abs() < 1e-6, "{name} {what}: {} vs {}", r.length, reference.length);
            }
        }
    }
    eprintln!("worst steps {worst}");
}

/// The same on a grid: from points of every flat of every element of the first layer, along the six
/// flat directions (exactly and a hair off), in the z-wall and inside, in all four loops.
#[test]
fn a_grid_of_grazing_rays_ends_in_bounded_steps_in_every_loop() {
    let w = mesh_world(3, 2);
    let (mut n, mut typed, mut worst) = (0usize, 0usize, 0u64);
    let mut sc = Scratch::new();
    for q in 0..3 {
        for r in 0..3 {
            let c = [P * 3f64.sqrt() / 2.0 * f64::from(q), P * (f64::from(r) + 0.5 * f64::from(q))];
            for k in 0..6 {
                let nrm = (30.0 + 60.0 * f64::from(k)).to_radians();
                let dir = nrm + 90f64.to_radians();
                for s in [-0.3, 0.0, 0.25] {
                    for (z, dz) in [(0.2, 0.0), (1.0, 0.0), (1.0, 1e-13), (0.2, 1e-9)] {
                        for eps in [0.0, 1e-13] {
                            let a = dir + eps;
                            let o = p3(c[0] + 0.5 * P * nrm.cos() + s * dir.cos(), c[1] + 0.5 * P * nrm.sin() + s * dir.sin(), z);
                            let d = v3(a.cos(), a.sin(), dz);
                            if w.locate(o, Some(d), None, &mut sc).map_or(true, |l| l.cell.is_none()) {
                                continue;
                            }
                            for (what, run) in [
                                ("trace", walk_trace(&w, o, d)),
                                ("fire", walk_fire(&w, o, d)),
                                ("surface flight", walk_flight(&w, ModePolicy::Surface, o, d)),
                                ("delta flight", walk_flight(&w, ModePolicy::Delta, o, d)),
                            ] {
                                let t = run.end == format!("lost {:?}", LostCause::RecoveryFailed);
                                assert!(t || run.end == "Boundary" || run.end == "LeftModel", "{what} from {o:?} {d:?}: {run:?}");
                                assert!(run.steps <= 400, "{what} from {o:?} {d:?}: {run:?}");
                                typed += usize::from(t);
                                worst = worst.max(run.steps);
                                n += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    eprintln!("{n} grazing walks, {typed} typed errors, worst {worst} steps");
    assert!(n > 1000);
}

// ---------------------------------------------------------------- part 2: many-sector pin

const RF: f64 = 0.4;
const ZH: f64 = 2.0;
const SECTORS: usize = 16;

/// A pin in a flat-top hex prism (flat to flat `P`): `SECTORS` fuel sectors of the cylinder `RF`
/// (cells `0..SECTORS`, wedges bounded by 8 planes through the axis) and the moderator (cell
/// `SECTORS`), z in `[-ZH, ZH]`.
fn sector_pin() -> Fx {
    let mut fx = Fx::new(10.0);
    fx.complement = ComplementPolicy::Outside;
    let hb = [P / 3f64.sqrt(), 0.5 * P];
    fx.bounds = Some(Aabb { lo: p3(-hb[0], -hb[1], -ZH), hi: p3(hb[0], hb[1], ZH) });
    let cyl = fx.cylinder(sm_geom_api::Frame::at(p3(0.0, 0.0, 0.0)), RF);
    let mut sides: Vec<HalfSpace> = hex::flats(false)
        .iter()
        .map(|n| {
            let s = fx.plane(p3(0.5 * P * n[0], 0.5 * P * n[1], 0.0), v3(n[0], n[1], 0.0));
            hs(s, true)
        })
        .collect();
    let zlo = fx.plane(p3(0.0, 0.0, -ZH), v3(0.0, 0.0, -1.0));
    let zhi = fx.plane(p3(0.0, 0.0, ZH), v3(0.0, 0.0, 1.0));
    sides.push(hs(zlo, true));
    sides.push(hs(zhi, true));
    // Plane k through the axis at angle `k * 22.5` degrees, normal to the left of it.
    let half = SECTORS / 2;
    let planes: Vec<_> = (0..half)
        .map(|k| {
            let t = (k as f64 * 180.0 / half as f64).to_radians();
            fx.plane(p3(0.0, 0.0, 0.0), v3(-t.sin(), t.cos(), 0.0))
        })
        .collect();
    for k in 0..SECTORS {
        // Sector k lies between the rays at `k` and `k + 1` times 22.5 degrees: left of ray k and
        // right of ray k + 1. Rays `half..` are the planes `0..half` reversed (`fwd` false).
        let (a, b) = (k % half, (k + 1) % half);
        let (fa, fb) = (k < half, (k + 1) % SECTORS < half);
        let mut h = sides.clone();
        h.push(hs(cyl, true));
        h.push(hs(planes[a], !fa));
        h.push(hs(planes[b], fb));
        fx.cell1(&format!("sector{k}"), &h);
    }
    let mut h = sides;
    h.push(hs(cyl, false));
    fx.cell1("mod", &h);
    fx
}

fn sector_world(n: u32, nz: u32) -> WorldTracker {
    let fx = sector_pin();
    let bnd = BoundarySpec::default();
    let leaf = LeafInput::Csg(CsgLeafInput::new(&fx.store, &fx.cells, fx.bounds, &fx.complement));
    let fills = vec![ComponentId(0); (n * n * nz) as usize];
    let spec = WorldSpec::new(
        vec![
            ComponentKind::Leaf { input: 0 },
            ComponentKind::Lattice(Lattice::new(LatticeKind::HexYRows { pitch: P, z_pitch: 2.0 * ZH }, p3(0.0, 0.0, 0.0), [n, n, nz], fills, None)),
        ],
        ComponentId(1),
        InstancePolicy::Auto,
    );
    let (w, issues) = MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &[leaf], &bnd)), &BuildOptions::default()).expect("sector world builds");
    for i in &issues.issues {
        eprintln!("issue: {i:?}");
    }
    w
}

/// `(cell, length)` of every segment of a trace, and the end.
fn segments(w: &dyn Tracker, o: Point3, d: Vector3) -> (Vec<(u32, f64)>, TraceEnd) {
    let mut sc = Scratch::new();
    let mut st = w.init_state(o, d, None, &mut sc).expect("init");
    assert!(!st.cell.is_none(), "start inside");
    let mut v = Vec::new();
    let end = w
        .trace(
            &mut st,
            &TraceOptions::default(),
            &mut |s: &Segment| {
                v.push((s.cell.0, s.length));
                Flow::Continue
            },
            &mut sc,
        )
        .expect("trace");
    (v, end)
}

/// Rays through the apex of the sectors (the point where all of them meet) in every direction of a
/// sweep that includes the sector planes (multiples of 22.5 degrees) and a tilt out of the plane:
/// none is lost, and the chords are the analytic ones (total to the prism, fuel `0.2 + RF / h`).
#[test]
fn rays_through_the_apex_of_a_sector_pin_have_the_analytic_chords() {
    let w = sector_world(1, 1);
    let flats = hex::flats(false);
    let (mut worst, mut max_run, mut n) = (0.0f64, 0usize, 0usize);
    for k in 0..240 {
        for alpha in [0.0f64, 0.3, -0.5] {
            let phi = (k as f64 * 1.5).to_radians();
            let d = v3(phi.cos() * alpha.cos(), phi.sin() * alpha.cos(), alpha.sin());
            let o = p3(-0.2 * d.x, -0.2 * d.y, -0.2 * d.z);
            let (segs, end) = segments(&w, o, d);
            assert!(matches!(end, TraceEnd::Boundary { .. } | TraceEnd::LeftModel), "phi {k} alpha {alpha}: {end:?}");
            // Analytic exit of the line from the apex: the nearest of the six flats and the z walls.
            let mut t_exit = f64::INFINITY;
            for f in &flats {
                let dn = d.x * f[0] + d.y * f[1];
                if dn > 1e-12 {
                    t_exit = t_exit.min(0.5 * P / dn);
                }
            }
            if d.z.abs() > 1e-12 {
                t_exit = t_exit.min(ZH / d.z.abs());
            }
            let total: f64 = segs.iter().map(|s| s.1).sum();
            let fuel: f64 = segs.iter().filter(|s| s.0 as usize % (SECTORS + 1) < SECTORS).map(|s| s.1).sum();
            let h = alpha.cos();
            let (want_total, want_fuel) = (0.2 + t_exit, 0.2 + RF / h);
            worst = worst.max((total - want_total).abs()).max((fuel - want_fuel).abs());
            let mut run = 0;
            for s in &segs {
                run = if s.1 <= 1e-9 { run + 1 } else { 0 };
                max_run = max_run.max(run);
            }
            n += 1;
        }
    }
    eprintln!("{n} rays through the apex: worst chord error {worst:.2e}, longest run of tiny segments {max_run}");
    assert!(worst < 1e-9, "chord error {worst}");
}

#[test]
fn a_ray_along_the_axis_of_a_sector_pin_is_carried_through() {
    let w = sector_world(3, 2);
    // Up the axis of element (0,0,0) and on through the second layer: all sectors meet on it.
    let (segs, end) = segments(&w, p3(0.0, 0.0, -ZH + 0.3), v3(0.0, 0.0, 1.0));
    eprintln!("axis: {} segments, {end:?}", segs.len());
    assert!(matches!(end, TraceEnd::Boundary { .. } | TraceEnd::LeftModel), "{end:?}");
    let total: f64 = segs.iter().map(|s| s.1).sum();
    // From z = -ZH + 0.3 to the top of the second layer (z = 3 ZH).
    assert!((total - (4.0 * ZH - 0.3)).abs() < 1e-9, "axis chord {total}");
    // The axis is inside the cylinder: every cell on it is a fuel sector.
    assert!(segs.iter().all(|s| (s.0 as usize % (SECTORS + 1)) < SECTORS || s.1 <= 1e-9), "{segs:?}");
}

#[test]
fn a_ray_in_a_sector_plane_has_the_analytic_chords() {
    let w = sector_world(3, 1);
    // Along x = 0 (the sector plane at 90 degrees, a flat normal): up the column (0, r), r = 0..2.
    let y0 = -0.3;
    let (segs, end) = segments(&w, p3(0.0, y0, 0.2), v3(0.0, 1.0, 0.0));
    assert!(matches!(end, TraceEnd::Boundary { .. } | TraceEnd::LeftModel), "{end:?}");
    let total: f64 = segs.iter().map(|s| s.1).sum();
    let fuel: f64 = segs.iter().filter(|s| (s.0 as usize % (SECTORS + 1)) < SECTORS).map(|s| s.1).sum();
    let want_total = 2.0 * P + 0.5 * P - y0;
    let want_fuel = (RF - y0) + 2.0 * (2.0 * RF);
    assert!((total - want_total).abs() < 1e-9, "total {total} vs {want_total}");
    assert!((fuel - want_fuel).abs() < 1e-9, "fuel {fuel} vs {want_fuel}");
    // Along the 22.5 degree sector plane through the axis, started on the axis side of the pin.
    let a = 22.5f64.to_radians();
    let (segs, end) = segments(&w, p3(-0.2 * a.cos(), -0.2 * a.sin(), 0.2), v3(a.cos(), a.sin(), 0.0));
    assert!(matches!(end, TraceEnd::Boundary { .. } | TraceEnd::LeftModel), "{end:?}");
    let fuel: f64 = segs.iter().filter(|s| (s.0 as usize % (SECTORS + 1)) < SECTORS).map(|s| s.1).sum();
    // The chord through the cylinder from 0.2 before the axis is 0.2 + RF; other elements add 2 RF each.
    assert!(fuel >= 0.2 + RF - 1e-9, "fuel {fuel}");
}

// ---------------------------------------------------------------- part 3: sector-fan wedge meshes

/// `ns` wedges around the z axis (unit radius), three layers of height 1: a ray along the axis edge
/// crosses `ns - 1` consecutive zero-length cells.
fn fan(ns: usize) -> sm_mesh_api::reference::ReferenceMesh {
    use sm_mesh_api::{CellShape, SerialAlloc};
    use sm_testkit::meshes::{CellSpec, build_mesh};
    let nz = 3usize;
    let mut pts = vec![];
    for k in 0..=nz {
        pts.push([0.0, 0.0, k as f64]);
        for i in 0..ns {
            let a = std::f64::consts::TAU * i as f64 / ns as f64;
            pts.push([a.cos(), a.sin(), k as f64]);
        }
    }
    let ix = |k: usize, i: usize| k * (ns + 1) + 1 + (i % ns);
    let ax = |k: usize| k * (ns + 1);
    let mut cells = vec![];
    for k in 0..nz {
        for i in 0..ns {
            cells.push(CellSpec::Template(CellShape::Wedge, vec![ax(k), ix(k, i), ix(k, i + 1), ax(k + 1), ix(k + 1, i), ix(k + 1, i + 1)]));
        }
    }
    build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("fan mesh")
}

#[test]
fn a_ray_through_the_axis_edge_of_a_fan_mesh_is_carried_through_and_limits_are_kept() {
    for ns in [16usize, 24, 48] {
        let mesh = fan(ns);
        let t = common::build_mesh_tracker(&mesh, &common::vacuum(), &common::opts_fail());
        let o = p3(-0.5, 0.0, 1.5);
        // Through the axis edge (a run of ns - 1 zero-length steps): chord = 0.5 + 1 (to the rim).
        let mut run = 0u32;
        let mut maxrun = 0u32;
        let mut len = 0.0;
        let mut sc = Scratch::new();
        let mut st = t.init_state(o, v3(1.0, 0.0, 0.0), None, &mut sc).expect("init");
        let end = t
            .trace(
                &mut st,
                &TraceOptions::default(),
                &mut |s: &Segment| {
                    len += s.length;
                    run = if s.length <= 1e-9 { run + 1 } else { 0 };
                    maxrun = maxrun.max(run);
                    Flow::Continue
                },
                &mut sc,
            )
            .expect("trace");
        assert!(maxrun > 8 || ns == 16, "ns {ns}: a run longer than the bound (got {maxrun})");
        assert!(matches!(end, TraceEnd::Boundary { .. } | TraceEnd::LeftModel), "ns {ns}: {end:?}");
        assert!((len - 1.5).abs() < 1e-8, "ns {ns}: trace length {len}");
        let mut st = t.init_state(o, v3(1.0, 0.0, 0.0), None, &mut sc).expect("init");
        let r = t.fire(&mut st, f64::INFINITY, 100_000, &mut sc).expect("fire");
        assert!(matches!(r.end, StepEnd::Boundary { .. }), "ns {ns}: {:?}", r.end);
        assert!((r.dist - 1.5).abs() < 1e-8, "ns {ns}: fire {}", r.dist);
        // A limit on the axis edge (or within a tolerance of it) is a Limit at that distance.
        for eps in [0.0, 1e-12, 1e-10, 1e-9, 5e-9, 1e-7, 1e-5] {
            let lim = 0.5 + eps;
            let mut st = t.init_state(o, v3(1.0, 0.0, 0.0), None, &mut sc).expect("init");
            let r = t.fire(&mut st, lim, 100_000, &mut sc).expect("fire");
            assert!(matches!(r.end, StepEnd::Limit), "ns {ns} lim {lim}: {:?}", r.end);
            assert!((r.dist - lim).abs() < 1e-8, "ns {ns} lim {lim}: dist {}", r.dist);
        }
    }
}
