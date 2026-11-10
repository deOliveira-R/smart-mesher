//! W4-RT-WORLD: CSG prototypes hosted by the world walker under `Union` and `Lattice`.
//!
//! The oracle is the flattened analytic tracker (`CsgWorld::analytic_tracker`, or a flat leaf built
//! by hand). An event is compared by cell id, region name and chord length (`1e-9`); surface ids
//! differ by construction (the world numbers surfaces per prototype).
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use sm_core::{EntityId, Point3, Transform};
use sm_rt::{CsgWorldOptions, MAX_FLAT_CELLS, MeshTrackerBuilder, WorldTracker, csg_world};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, CellId, ComponentId, ComponentKind, CsgLeafInput, HierInput, InstancePolicy, Lattice, LatticeKind, LeafInput, LostCause, PartitionOptions, Placement, RtError, Scratch,
    StepEnd, TraceEnd, TraceOptions, Tracker, TrackerBuilder, TrackerCaps, WorldSpec,
};
use sm_testkit::topo::pin_assembly;

const PITCH: f64 = 1.26;
const RF: f64 = 0.4096;
const RC: f64 = 0.475;
const H: f64 = 100.0;

/// What one trace is compared on: `(cell, region name, chord length)` per segment and the end kind.
type Events = (Vec<(u32, String, f64)>, &'static str);

fn events(t: &dyn Tracker, o: Point3, d: sm_core::Vector3) -> Option<Events> {
    let mut sc = Scratch::new();
    let mut st = t.init_state(o, d, None, &mut sc).ok()?;
    if st.cell.is_none() {
        return None;
    }
    let mut segs = Vec::new();
    let end = t
        .trace(
            &mut st,
            &TraceOptions::default(),
            &mut |s: &sm_rt_api::Segment| {
                segs.push((s.cell.0, t.regions().get(s.region).map(|r| r.name.clone()).unwrap_or_default(), s.length));
                sm_rt_api::Flow::Continue
            },
            &mut sc,
        )
        .expect("trace");
    let kind = match end {
        TraceEnd::Boundary { .. } => "boundary",
        TraceEnd::LeftModel => "left",
        TraceEnd::Limit => "limit",
        TraceEnd::Lost(_) => "lost",
        _ => "other",
    };
    Some((segs, kind))
}

fn assert_same(a: &Events, b: &Events, what: &str) {
    assert_eq!(a.1, b.1, "{what}: end kind");
    assert_eq!(a.0.len(), b.0.len(), "{what}: segment count\n{:?}\n{:?}", a.0, b.0);
    for (x, y) in a.0.iter().zip(&b.0) {
        assert_eq!((x.0, &x.1), (y.0, &y.1), "{what}: cell or region");
        assert!((x.2 - y.2).abs() < 1e-9, "{what}: chord {} vs {}", x.2, y.2);
    }
}

fn u01(r: &mut SplitMix64) -> f64 {
    0.5 * (r.unit() + 1.0)
}

// ---- P2: the bridged pin assembly ---------------------------------------------------------------

fn p2(n: usize) -> sm_rt::CsgWorld {
    let fix = pin_assembly(n, PITCH, RF, RC, H).unwrap();
    csg_world(&fix.model, &fix.complex, &CsgWorldOptions::default()).unwrap()
}

#[test]
fn p2_hierarchy_has_the_flattened_events_on_seeded_rays() {
    let w = p2(3);
    let (hier, _) = w.world_tracker(&BoundarySpec::default(), &BuildOptions::default()).unwrap();
    let flat = w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    assert_eq!(hier.info().cells, flat.info().cells);
    assert_eq!(hier.info().prototypes, 1);
    assert_eq!(hier.info().copies, 81, "9 n^2 pins");
    assert!(hier.caps().has(TrackerCaps::HIERARCHY) && hier.caps().has(TrackerCaps::ANALYTIC) && !hier.caps().has(TrackerCaps::EXACT_PLANAR));
    // Pins stand on a 9 x 9 grid of the pitch (3 clusters of 3 per side).
    let mut r = rng("csg-world-p2");
    let (mut n, mut chords) = (0, 0usize);
    for k in 0..400 {
        let (i, j) = ((u01(&mut r) * 9.0).floor().min(8.0), (u01(&mut r) * 9.0).floor().min(8.0));
        let b = (2.0 * u01(&mut r) - 1.0) * 0.999 * RC;
        let th = std::f64::consts::TAU * u01(&mut r);
        let raw = v3(th.cos(), std::hint::black_box(th).sin(), 0.6 * u01(&mut r) - 0.3);
        let d = raw * (1.0 / raw.norm());
        let rho = d.x.hypot(d.y);
        let (nx, ny) = (-d.y / rho, d.x / rho);
        let o = p3(i * PITCH + b * nx, j * PITCH + b * ny, 10.0 + 80.0 * u01(&mut r));
        let (Some(a), Some(c)) = (events(&hier, o, d), events(&flat, o, d)) else { continue };
        assert_same(&a, &c, &format!("ray {k}"));
        chords += a.0.len();
        n += 1;
    }
    assert!(n > 300 && chords > 300, "{n} rays, {chords} chords compared");
}

#[test]
fn p2_pins_touched_and_structure_bytes_follow_the_closed_forms() {
    // `structure_bytes` is `memory_bytes`: independent of the number of copies up to the lattice
    // fill array (4 bytes per element; the 9 cluster placements and the one lattice are fixed).
    let (w5, w17) = (p2(5), p2(17));
    let b = |w: &sm_rt::CsgWorld| w.world_tracker(&BoundarySpec::default(), &BuildOptions::default()).unwrap().0;
    let (t5, t17) = (b(&w5), b(&w17));
    assert_eq!((t5.info().copies, t17.info().copies), (9 * 25, 9 * 289));
    assert_eq!((t5.info().prototypes, t17.info().prototypes), (1, 1));
    assert_eq!(t17.info().memory_bytes - t5.info().memory_bytes, 4 * (289 - 25));
    // The flattened tracker grows with the copies.
    let f = |w: &sm_rt::CsgWorld| w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0.info().memory_bytes;
    assert!(f(&w17) > 10 * f(&w5) / 2);
    assert!(t17.info().memory_bytes < f(&w17) / 100);
}

#[test]
fn p2_determinism_one_thread_against_many() {
    let w = p2(2);
    let (t, _) = w.world_tracker(&BoundarySpec::default(), &BuildOptions::default()).unwrap();
    let mut r = rng("csg-world-det");
    let rays: Vec<sm_geom_api::Ray> = (0..600)
        .map(|_| {
            let (i, j) = ((u01(&mut r) * 6.0).floor().min(5.0), (u01(&mut r) * 6.0).floor().min(5.0));
            let o = p3(i * PITCH + 0.2 * r.unit(), j * PITCH + 0.2 * r.unit(), 10.0 + 80.0 * u01(&mut r));
            sm_geom_api::Ray { origin: o, dir: random_dir(&mut r) }
        })
        .collect();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let b = t.trace_bundle(&rays, &TraceOptions::default()).unwrap();
            let mut bits: Vec<u64> = b.length.iter().map(|x| x.to_bits()).collect();
            bits.extend(b.cell.iter().map(|&c| u64::from(c)));
            bits.extend(b.region.iter().map(|&c| u64::from(c)));
            bits.extend(b.end_surface.iter().map(|&c| u64::from(c)));
            bits.extend(b.end.iter().map(|e| format!("{e:?}").len() as u64));
            bits
        })
    };
    let base = sm_testkit::determinism::check(2, 4, run).unwrap();
    assert!(base.len() > 1000);
}

// ---- a moderated lattice (OpenMC style: pin universe with a box cell) ------------------------------

struct Pins {
    n: usize,
}

const HALF: f64 = 0.5 * PITCH;
const ZH: f64 = 5.0;

/// The pin universe: fuel, clad and moderator (the box minus the clad cylinder), z in `[-5, 5]`.
fn pin_leaf() -> Fx {
    let mut fx = Fx::new(10.0);
    fx.complement = sm_rt_api::ComplementPolicy::Outside;
    fx.bounds = None;
    let axis = Frame::at(p3(0.0, 0.0, 0.0));
    let cf = fx.cylinder(axis, RF);
    let cc = fx.cylinder(axis, RC);
    let bx = fx.box_planes(p3(-HALF, -HALF, -ZH), p3(HALF, HALF, ZH));
    let sides: Vec<_> = bx.iter().map(|&s| hs(s, true)).collect();
    let with = |extra: HalfSpace| {
        let mut v = sides.clone();
        v.push(extra);
        v
    };
    fx.cell1("fuel", &with(hs(cf, true)));
    let mut clad = with(hs(cc, true));
    clad.push(hs(cf, false));
    fx.cell1("clad", &clad);
    fx.cell1("mod", &with(hs(cc, false)));
    fx
}

use sm_geom_api::{Frame, HalfSpace};

/// The same lattice flattened by hand: `n x n` pins of the leaf above, shared wall planes.
fn flat_pins(n: usize) -> Fx {
    let mut fx = Fx::new(10.0);
    fx.complement = sm_rt_api::ComplementPolicy::Outside;
    fx.bounds = None;
    let xs: Vec<EntityId> = (0..=n).map(|i| fx.plane(p3((i as f64 - 0.5) * PITCH, 0.0, 0.0), v3(1.0, 0.0, 0.0))).collect();
    let ys: Vec<EntityId> = (0..=n).map(|j| fx.plane(p3(0.0, (j as f64 - 0.5) * PITCH, 0.0), v3(0.0, 1.0, 0.0))).collect();
    let zlo = fx.plane(p3(0.0, 0.0, -ZH), v3(0.0, 0.0, 1.0));
    let zhi = fx.plane(p3(0.0, 0.0, ZH), v3(0.0, 0.0, 1.0));
    for j in 0..n {
        for i in 0..n {
            let c = (i as f64 * PITCH, j as f64 * PITCH);
            let axis = Frame::at(p3(c.0, c.1, 0.0));
            let cf = fx.cylinder(axis, RF);
            let cc = fx.cylinder(axis, RC);
            let b = vec![hs(xs[i], false), hs(xs[i + 1], true), hs(ys[j], false), hs(ys[j + 1], true), hs(zlo, false), hs(zhi, true)];
            let with = |extra: HalfSpace| {
                let mut v = b.clone();
                v.push(extra);
                v
            };
            fx.cell1("fuel", &with(hs(cf, true)));
            let mut clad = with(hs(cc, true));
            clad.push(hs(cf, false));
            fx.cell1("clad", &clad);
            fx.cell1("mod", &with(hs(cc, false)));
        }
    }
    fx
}

fn lattice_spec(n: usize, root_union: bool) -> WorldSpec {
    let mut comps = vec![
        ComponentKind::Leaf { input: 0 },
        ComponentKind::Lattice(Lattice::new(
            LatticeKind::Rect { pitch: [PITCH, PITCH, 2.0 * ZH] },
            p3(-HALF, -HALF, -ZH),
            [n as u32, n as u32, 1],
            vec![ComponentId(0); n * n],
            None,
        )),
    ];
    let mut root = 1;
    if root_union {
        comps.push(ComponentKind::Union { placements: vec![Placement::new(ComponentId(1), Transform::IDENTITY)] });
        root = 2;
    }
    WorldSpec::new(comps, ComponentId(root), InstancePolicy::Auto)
}

fn world_of_pins(n: usize, root_union: bool) -> (Pins, WorldTracker) {
    let leaf_store = pin_leaf();
    let spec = lattice_spec(n, root_union);
    let bnd = BoundarySpec::default();
    let t = {
        let leaf = CsgLeafInput::new(&leaf_store.store, &leaf_store.cells, leaf_store.bounds, &leaf_store.complement);
        let leaves = [LeafInput::Csg(leaf)];
        let h = HierInput::new(&spec, &leaves, &bnd);
        MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).unwrap().0
    };
    (Pins { n }, t)
}

#[test]
fn a_moderated_lattice_has_the_flat_events_and_the_walls_are_slot_changes() {
    let n = 5;
    let (pins, w) = world_of_pins(n, false);
    let flat = flat_pins(n).build();
    assert_eq!((w.info().cells, w.info().copies, w.info().prototypes), (3 * n * n, n * n, 1));
    let mut r = rng("csg-world-moderated");
    let (mut compared, mut slot_changes) = (0, 0usize);
    let mut sc = Scratch::new();
    for k in 0..800 {
        let o = p3(u01(&mut r) * (n as f64 * PITCH - 0.02) - HALF + 0.01, u01(&mut r) * (n as f64 * PITCH - 0.02) - HALF + 0.01, (2.0 * u01(&mut r) - 1.0) * 4.5);
        let d = random_dir(&mut r);
        let (Some(a), Some(c)) = (events(&w, o, d), events(&flat, o, d)) else { continue };
        assert_same(&a, &c, &format!("ray {k}"));
        compared += 1;
        // Closed form: the moderator chord inside one element wall-to-wall is the flat cell's.
        // Count the element runs: pins touched = 1 + SlotChange count (the adapter's counter).
        let runs = 1 + c.0.windows(2).filter(|p| p[0].0 / 3 != p[1].0 / 3).count();
        let mut st = w.init_state(o, d, None, &mut sc).unwrap();
        let (mut touched, mut guard) = (1usize, 0);
        loop {
            let s = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
            match s.end {
                StepEnd::SlotChange { .. } => {
                    touched += 1;
                    slot_changes += 1;
                }
                StepEnd::CellFace { .. } | StepEnd::RegionBoundary { .. } => {}
                _ => break,
            }
            guard += 1;
            assert!(guard < 10_000);
        }
        assert_eq!(touched, runs, "ray {k}");
    }
    assert!(compared > 700 && slot_changes > 500, "{compared} rays, {slot_changes} wall crossings");
    assert_eq!(pins.n, n);
}

#[test]
fn volumes_regions_and_locate_of_the_moderated_lattice() {
    let n = 4;
    let (_, w) = world_of_pins(n, true);
    // Analytic: one chord along x through the centre row at height 0 is the sum of the diameters.
    let mut sc = Scratch::new();
    let o = p3(-HALF + 1e-6, 0.0, 0.3);
    let (segs, kind) = events(&w, o, v3(1.0, 0.0, 0.0)).unwrap();
    assert_eq!(kind, "boundary");
    let sum = |name: &str| segs.iter().filter(|s| s.1 == name).map(|s| s.2).sum::<f64>();
    let len = n as f64 * PITCH - 1e-6;
    assert!((sum("fuel") - n as f64 * 2.0 * RF).abs() < 1e-9);
    assert!((sum("clad") - n as f64 * 2.0 * (RC - RF)).abs() < 1e-9);
    assert!((sum("mod") - (len - n as f64 * 2.0 * RC)).abs() < 1e-9);
    // Locate: the centre of pin (2, 1) is fuel; its cell id is 3 * (1 * n + 2); levels name the element.
    let l = w.locate(p3(2.0 * PITCH, PITCH, 0.0), None, None, &mut sc).unwrap();
    assert_eq!(l.cell, CellId::new(3 * (n + 2)));
    assert_eq!(w.regions().get(l.region).unwrap().name, "fuel");
    assert_eq!(l.levels.len(), 2, "the union and the lattice");
    // Outside the lattice there is nothing.
    assert!(w.locate(p3(-1.0, 0.0, 0.0), None, None, &mut sc).unwrap().cell.is_none());
}

// ---- unions, overlaps and refusals --------------------------------------------------------------------

#[test]
fn a_union_of_one_identity_placement_equals_the_bare_leaf() {
    let leaf = pin_leaf();
    let bare = {
        let mut f = pin_leaf();
        f.complement = sm_rt_api::ComplementPolicy::Outside;
        f.build()
    };
    let spec = WorldSpec::new(
        vec![ComponentKind::Leaf { input: 0 }, ComponentKind::Union { placements: vec![Placement::new(ComponentId(0), Transform::IDENTITY)] }],
        ComponentId(1),
        InstancePolicy::Auto,
    );
    let bnd = BoundarySpec::default();
    let lf = CsgLeafInput::new(&leaf.store, &leaf.cells, leaf.bounds, &leaf.complement);
    let leaves = [LeafInput::Csg(lf)];
    let h = HierInput::new(&spec, &leaves, &bnd);
    let (w, _) = MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).unwrap();
    let mut r = rng("csg-world-union");
    let mut n = 0;
    for k in 0..300 {
        let o = p3(0.5 * u01(&mut r) - 0.25, 0.5 * u01(&mut r) - 0.25, 4.0 * u01(&mut r) - 2.0);
        let d = random_dir(&mut r);
        let (Some(a), Some(b)) = (events(&w, o, d), events(&bare, o, d)) else { continue };
        assert_same(&a, &b, &format!("ray {k}"));
        n += 1;
    }
    assert!(n > 250);
}

#[test]
fn overlapping_csg_placements_are_never_resolved_by_order() {
    let leaf = pin_leaf();
    // The second placement is shifted by a third of the pitch: the boxes overlap in a slab.
    let shifted = Transform { t: v3(PITCH / 3.0, 0.0, 0.0), ..Transform::IDENTITY };
    let spec = WorldSpec::new(
        vec![
            ComponentKind::Leaf { input: 0 },
            ComponentKind::Union { placements: vec![Placement::new(ComponentId(0), Transform::IDENTITY), Placement::new(ComponentId(0), shifted)] },
        ],
        ComponentId(1),
        InstancePolicy::Auto,
    );
    let bnd = BoundarySpec::default();
    let lf = CsgLeafInput::new(&leaf.store, &leaf.cells, leaf.bounds, &leaf.complement);
    let leaves = [LeafInput::Csg(lf)];
    let h = HierInput::new(&spec, &leaves, &bnd);
    // Built: the vertex-based narrow phase has no vertices of a CSG leaf; the step finds it.
    let (w, _) = MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).unwrap();
    let mut sc = Scratch::new();
    // Strictly inside both moderator boxes: not located.
    let both = w.locate(p3(0.5 * PITCH / 3.0 + 0.2, 0.55, 0.0), Some(v3(0.267, 0.534, 0.802)), None, &mut sc).unwrap();
    assert!(both.cell.is_none(), "{both:?}");
    // In the first placement only: located. Stepping towards the overlap ends Lost(Overlap), the
    // state unchanged.
    let o = p3(-0.6, 0.55, 0.0);
    let mut st = w.init_state(o, v3(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert!(!st.cell.is_none());
    let before = st.clone();
    let s = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
    match s.end {
        StepEnd::Lost(l) => assert_eq!(l.cause, LostCause::Overlap),
        other => panic!("{other:?}"),
    }
    assert_eq!(st, before);
}

#[test]
fn refusals_partition_complement_fit_and_hex() {
    let leaf = pin_leaf();
    let bnd = BoundarySpec::default();
    let build = |spec: &WorldSpec, opts: &BuildOptions, f: &Fx| {
        let lf = CsgLeafInput::new(&f.store, &f.cells, f.bounds, &f.complement);
        let leaves = [LeafInput::Csg(lf)];
        let h = HierInput::new(spec, &leaves, &bnd);
        MeshTrackerBuilder.build_world(&BuildInput::Hier(h), opts).map(|_| ())
    };
    // A partition of a CSG leaf is refused, typed.
    let mut part = PartitionOptions::default();
    part.enabled = true;
    let e = build(&lattice_spec(2, false), &BuildOptions::default().with_partition(part), &leaf).unwrap_err();
    assert!(matches!(e, RtError::Unsupported(m) if m.contains("partition")), "{e:?}");
    // A pitch smaller than the pin box: world.lattice_fit.
    let mut spec = lattice_spec(2, false);
    if let ComponentKind::Lattice(l) = &mut spec.components[1] {
        l.kind = LatticeKind::Rect { pitch: [0.9 * PITCH, PITCH, 2.0 * ZH] };
    }
    match build(&spec, &BuildOptions::default(), &leaf).unwrap_err() {
        RtError::Build(v) => assert!(v.iter().any(|i| i.code == "world.lattice_fit")),
        other => panic!("{other:?}"),
    }
    // A bridged world with a complement region cannot be hosted: typed, and the oracle still builds.
    let fix = pin_assembly(1, PITCH, RF, RC, H).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { bodies: None, complement: Some("void".into()) }).unwrap();
    assert!(matches!(w.world_tracker(&bnd, &BuildOptions::default()), Err(RtError::Unsupported(_))));
    assert!(w.analytic_tracker(&bnd, &BuildOptions::default(), MAX_FLAT_CELLS).is_ok());
    // The analytic builder keeps refusing a placed leaf and points to the world walker.
    let spec = lattice_spec(2, false);
    let lf = CsgLeafInput::new(&leaf.store, &leaf.cells, leaf.bounds, &leaf.complement);
    let leaves = [LeafInput::Csg(lf)];
    let h = HierInput::new(&spec, &leaves, &bnd);
    assert!(matches!(sm_rt::AnalyticTrackerBuilder.build(&BuildInput::Hier(h), &BuildOptions::default()), Err(RtError::Unsupported(_))));
}
