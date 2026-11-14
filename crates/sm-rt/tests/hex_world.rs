//! W4-RT-HEX: hexagonal lattices (both orientations) in the world walker.
//!
//! The reference is analytic and independent of the tracker's cube rounding: for every element the
//! ray is clipped against the six half-planes of its hexagonal prism and the pin cylinder, and the
//! per-element intervals are sorted along the ray. Element centres come from the public
//! `Lattice::element_center`.
#[path = "common/csg.rs"]
mod csg;
#[path = "common/hex.rs"]
mod hex;

use csg::*;
use sm_core::Point3;
use sm_geom_api::HalfSpace;
use sm_rt::{MeshTrackerBuilder, WorldTracker};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, CellId, ComplementPolicy, ComponentId, ComponentKind, CsgLeafInput, Flow, HierInput, InstancePolicy, Lattice, LatticeKind, LeafInput, RtError, Scratch, Segment,
    StepEnd, TraceEnd, TraceOptions, Tracker, WorldSpec,
};

/// `(cell, chord length)` of a segment.
type Seg = (u32, f64);

const P: f64 = 1.5;
const RF: f64 = 0.4;
const ZH: f64 = 2.0;
use hex::S3;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Or {
    X,
    Y,
}

fn kind(o: Or, z_pitch: f64) -> LatticeKind {
    match o {
        Or::X => LatticeKind::Hex { pitch: P, z_pitch },
        Or::Y => LatticeKind::HexYRows { pitch: P, z_pitch },
    }
}

fn flats(o: Or) -> [[f64; 2]; 6] {
    hex::flats(o == Or::X)
}

/// A pin in a hexagonal prism of flat-to-flat `pitch` (fuel radius `RF`, z in `[-ZH, ZH]`).
fn hex_pin(o: Or, pitch: f64, name: &str) -> Fx {
    hex::hex_pin(o == Or::X, pitch, RF, ZH, name)
}

fn build(spec: &WorldSpec, leaves: &[&Fx]) -> Result<WorldTracker, RtError> {
    let bnd = BoundarySpec::default();
    let ins: Vec<LeafInput<'_>> = leaves.iter().map(|f| LeafInput::Csg(CsgLeafInput::new(&f.store, &f.cells, f.bounds, &f.complement))).collect();
    let h = HierInput::new(spec, &ins, &bnd);
    MeshTrackerBuilder.build_world(&BuildInput::Hier(h), &BuildOptions::default()).map(|r| r.0)
}

/// An `n x n x nz` rhombus of the pin, element `(0, 0, 0)` centred at `origin`.
fn lattice_spec(o: Or, n: u32, nz: u32, origin: Point3) -> WorldSpec {
    let zp = 2.0 * ZH;
    let fills = vec![ComponentId(0); (n * n * nz) as usize];
    WorldSpec::new(
        vec![ComponentKind::Leaf { input: 0 }, ComponentKind::Lattice(Lattice::new(kind(o, zp), origin, [n, n, nz], fills, None))],
        ComponentId(1),
        InstancePolicy::Auto,
    )
}

fn world(o: Or, n: u32, nz: u32, origin: Point3) -> (WorldTracker, Lattice) {
    let spec = lattice_spec(o, n, nz, origin);
    let ComponentKind::Lattice(l) = &spec.components[1] else { unreachable!() };
    let l = l.clone();
    let w = build(&spec, &[&hex_pin(o, P, "")]).expect("hex world builds");
    (w, l)
}

/// `(cell, length)` of the segments of one trace from `o` along `d`, and how it ended.
fn trace(w: &dyn Tracker, o: Point3, d: sm_core::Vector3) -> Option<(Vec<(u32, f64)>, &'static str)> {
    let mut sc = Scratch::new();
    let mut st = w.init_state(o, d, None, &mut sc).ok()?;
    if st.cell.is_none() {
        return None;
    }
    let mut segs = Vec::new();
    let end = w
        .trace(
            &mut st,
            &TraceOptions::default(),
            &mut |s: &Segment| {
                segs.push((s.cell.0, s.length));
                Flow::Continue
            },
            &mut sc,
        )
        .expect("trace");
    Some((
        segs,
        match end {
            TraceEnd::Boundary { .. } => "boundary",
            TraceEnd::LeftModel => "left",
            TraceEnd::Limit => "limit",
            TraceEnd::Lost(_) => "lost",
            _ => "other",
        },
    ))
}

/// Clip `[t0, t1]` by the slab `lo <= a + b t <= hi`.
fn clip(t0: f64, t1: f64, a: f64, b: f64, lo: f64, hi: f64) -> Option<(f64, f64)> {
    if b.abs() < 1e-300 {
        return (a >= lo && a <= hi).then_some((t0, t1));
    }
    let (x, y) = ((lo - a) / b, (hi - a) / b);
    let (s0, s1) = (t0.max(x.min(y)), t1.min(x.max(y)));
    (s0 < s1).then_some((s0, s1))
}

/// Analytic `(element lin, region 0 fuel / 1 mod, t0, t1)` of the ray, `t >= 0`, per element.
fn reference(o: Or, l: &Lattice, org: Point3, d: [f64; 3]) -> Vec<(usize, u32, f64, f64)> {
    let mut out = Vec::new();
    let n = l.dims[0] as usize;
    let nl = flats(o);
    for lin in 0..l.element_count().unwrap() {
        let idx = [(lin % n) as i32, ((lin / n) % l.dims[1] as usize) as i32, (lin / (n * l.dims[1] as usize)) as i32];
        let c = l.element_center(idx);
        let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
        let mut ok = true;
        for f in &nl {
            let a = (org.x - c.x) * f[0] + (org.y - c.y) * f[1];
            let b = d[0] * f[0] + d[1] * f[1];
            match clip(t0, t1, a, b, -0.5 * P, 0.5 * P) {
                Some((a, b)) => (t0, t1) = (a, b),
                None => ok = false,
            }
        }
        match clip(t0, t1, org.z - c.z, d[2], -ZH, ZH) {
            Some((a, b)) => (t0, t1) = (a, b),
            None => ok = false,
        }
        if !ok || t1 - t0 < 1e-13 {
            continue;
        }
        // Fuel: the infinite cylinder of radius RF about the element axis.
        let (ox, oy) = (org.x - c.x, org.y - c.y);
        let (a2, b2, c2) = (d[0] * d[0] + d[1] * d[1], 2.0 * (ox * d[0] + oy * d[1]), ox * ox + oy * oy - RF * RF);
        let disc = b2 * b2 - 4.0 * a2 * c2;
        let fuel = (a2 > 1e-300 && disc > 0.0).then(|| ((-b2 - disc.sqrt()) / (2.0 * a2), (-b2 + disc.sqrt()) / (2.0 * a2)));
        let fuel = if a2 <= 1e-300 && c2 < 0.0 { Some((f64::NEG_INFINITY, f64::INFINITY)) } else { fuel };
        match fuel {
            Some((f0, f1)) if f1 > t0 && f0 < t1 => {
                let (f0, f1) = (f0.max(t0), f1.min(t1));
                if f0 - t0 > 1e-13 {
                    out.push((lin, 1, t0, f0));
                }
                out.push((lin, 0, f0, f1));
                if t1 - f1 > 1e-13 {
                    out.push((lin, 1, f1, t1));
                }
            }
            _ => out.push((lin, 1, t0, t1)),
        }
    }
    out.sort_by(|a, b| a.2.total_cmp(&b.2));
    // A start outside every element has no run.
    if out.first().is_none_or(|s| s.2 > 1e-9) {
        return Vec::new();
    }
    // Keep the run from t = 0 that has no gap: the tracker stops where it leaves the lattice.
    let mut run = Vec::new();
    let mut end = 0.0;
    for s in out {
        if !run.is_empty() && s.2 > end + 1e-9 {
            break;
        }
        end = s.3;
        run.push(s);
    }
    run
}

fn check_ray(o: Or, w: &WorldTracker, l: &Lattice, org: Point3, d: sm_core::Vector3, what: &str) -> usize {
    let da = [d.x, d.y, d.z];
    let want = reference(o, l, org, da);
    let Some((got, end)) = trace(w, org, d) else {
        assert!(want.is_empty(), "{what}: the tracker found no cell, the reference has {} segments", want.len());
        return 0;
    };
    assert_ne!(end, "lost", "{what}");
    assert_eq!(got.len(), want.len(), "{what}: segment count\n{got:?}\n{want:?}");
    for (g, r) in got.iter().zip(&want) {
        assert_eq!(u64::from(g.0), 2 * r.0 as u64 + u64::from(r.1), "{what}: cell");
        assert!((g.1 - (r.3 - r.2)).abs() < 1e-12, "{what}: chord {} vs {}", g.1, r.3 - r.2);
    }
    got.len()
}

const ORIGIN: fn() -> Point3 = || p3(0.3, -0.2, 0.1);

#[test]
fn chords_along_the_three_lattice_directions_are_the_analytic_ones() {
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 5, 1, ORIGIN());
        // A row of elements along each lattice direction: the neighbours of element (2, 2) at
        // +-(1,0), +-(0,1), +-(1,-1) lie on the line through (2, 2) with that direction.
        for (dq, dr) in [(1, 0), (0, 1), (1, -1)] {
            let (c0, c1) = (l.element_center([2 - 2 * dq, 2 - 2 * dr, 0]), l.element_center([2 + 2 * dq, 2 + 2 * dr, 0]));
            let len = ((c1.x - c0.x).powi(2) + (c1.y - c0.y).powi(2)).sqrt();
            let d = v3((c1.x - c0.x) / len, (c1.y - c0.y) / len, 0.0);
            // Start at the centre of the first element: fuel RF, then per element mod, fuel, mod.
            let (got, end) = trace(&w, c0, d).unwrap();
            assert_eq!(end, "boundary", "{o:?} ({dq},{dr})");
            assert_eq!(got.len(), 2 + 3 * 4 + 1 - 1, "{o:?} ({dq},{dr}): {got:?}");
            let (fuel, md): (Vec<&Seg>, Vec<&Seg>) = got.iter().partition(|g| g.0 % 2 == 0);
            assert_eq!(fuel.len(), 5);
            // 5 elements, centre of the first to the flat of the last: 4.5 pitches.
            let total: f64 = got.iter().map(|g| g.1).sum();
            assert!((total - 4.5 * P).abs() < 1e-12, "{o:?} ({dq},{dr}) total {total}");
            let sf: f64 = fuel.iter().map(|g| g.1).sum();
            assert!((sf - (4.0 * 2.0 * RF + RF)).abs() < 1e-12, "{o:?} fuel {sf}");
            let sm: f64 = md.iter().map(|g| g.1).sum();
            assert!((sm - (4.5 * P - 9.0 * RF)).abs() < 1e-12);
            // Every element on the row is visited once, in order.
            let cells: Vec<u32> = got.iter().map(|g| g.0 / 2).collect();
            let want: Vec<u32> = (0..5).map(|k| (2 + (k - 2) * dq + 5 * (2 + (k - 2) * dr)) as u32).collect();
            let mut dedup = cells.clone();
            dedup.dedup();
            assert_eq!(dedup, want, "{o:?} ({dq},{dr})");
            check_ray(o, &w, &l, c0, d, "row");
        }
    }
}

#[test]
fn seeded_random_rays_cross_elements_at_the_analytic_distances() {
    let mut total = 0usize;
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 6, 1, ORIGIN());
        let mut r = rng(&format!("hex-world-rays-{o:?}"));
        let (mut k, mut done) = (0usize, 0usize);
        while done < 1000 {
            k += 1;
            assert!(k < 20_000);
            // A start inside the array: the centre of a random element plus an offset.
            let lin = ((0.5 * (r.unit() + 1.0)) * 36.0).floor().min(35.0) as i32;
            let c = l.element_center([lin % 6, lin / 6, 0]);
            let org = p3(c.x + 0.45 * P * r.unit(), c.y + 0.45 * P * r.unit(), (0.9 * ZH) * r.unit());
            let d = random_dir(&mut r);
            let n = check_ray(o, &w, &l, org, d, &format!("{o:?} ray {k}"));
            if n > 0 {
                done += 1;
                total += n;
            }
        }
    }
    assert!(total > 8000, "{total} segments");
}

#[test]
fn locate_names_the_element_and_the_layer() {
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 4, 3, ORIGIN());
        let mut sc = Scratch::new();
        let n = 4;
        for lin in 0..4 * 4 * 3 {
            let idx = [lin % n, (lin / n) % n, lin / (n * n)];
            let c = l.element_center(idx);
            let loc = w.locate(p3(c.x, c.y, c.z), None, None, &mut sc).unwrap();
            assert_eq!(loc.cell, CellId::new(2 * lin as usize), "{o:?} {idx:?}");
            let lv = &loc.levels[0];
            assert_eq!((lv.lattice, lv.instance.get()), (idx, lin as usize));
            // A point in the moderator corner of the hexagon (r > RF) of the same element.
            let f = flats(o)[0];
            let q = w.locate(p3(c.x + 0.45 * P * f[0], c.y + 0.45 * P * f[1], c.z + 0.2), None, None, &mut sc).unwrap();
            assert_eq!(q.cell, CellId::new(2 * lin as usize + 1));
        }
        // Layer centres are `origin.z + k z_pitch`: the z walls are at half pitches.
        let zp = 2.0 * ZH;
        let c = l.element_center([1, 1, 1]);
        let (up, dn) = (w.locate(p3(c.x, c.y, c.z + 0.5 * zp - 0.01), None, None, &mut sc).unwrap(), w.locate(p3(c.x, c.y, c.z - 0.5 * zp + 0.01), None, None, &mut sc).unwrap());
        assert_eq!((up.levels[0].lattice, dn.levels[0].lattice), ([1, 1, 1], [1, 1, 1]));
        // Outside the stack in z, outside the rhombus in the plane.
        assert!(w.locate(p3(c.x, c.y, ORIGIN().z + 2.0 * zp + 0.5 * zp + 0.2), None, None, &mut sc).unwrap().cell.is_none());
        assert!(w.locate(p3(-3.0 * P, -3.0 * P, 0.0), None, None, &mut sc).unwrap().cell.is_none());
    }
}

#[test]
fn a_ray_down_the_stack_changes_layer_at_the_z_walls() {
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 3, 3, ORIGIN());
        let c = l.element_center([1, 2, 0]);
        let start = p3(c.x + 0.7 * RF, c.y, c.z - ZH + 0.25);
        let (got, end) = trace(&w, start, v3(0.0, 0.0, 1.0)).unwrap();
        assert_eq!(end, "boundary");
        assert_eq!(got.len(), 3, "{got:?}");
        let n = 3u32;
        let cells: Vec<u32> = got.iter().map(|g| g.0).collect();
        let e = |k: u32| 2 * (1 + 2 * n + k * n * n);
        assert_eq!(cells, vec![e(0), e(1), e(2)], "{o:?}");
        let lens: Vec<f64> = got.iter().map(|g| g.1).collect();
        assert!((lens[0] - (2.0 * ZH - 0.25)).abs() < 1e-12 && (lens[1] - 2.0 * ZH).abs() < 1e-12 && (lens[2] - 2.0 * ZH).abs() < 1e-12, "{lens:?}");
    }
}

/// Vertex rule: at a point where three elements meet, `locate` returns the element the direction
/// enters (nearest centre of `p + 4 tol dir`); on an edge between two, the one entered; along the
/// shared face it is a tie, tried highest linear index first and settled by the leaf's rule for a
/// point on its flat (deterministic).
#[test]
fn vertex_ties_follow_the_documented_rule() {
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 5, 1, ORIGIN());
        let mut sc = Scratch::new();
        let el = |i: i32, j: i32| l.element_center([i, j, 0]);
        let (a, b, c) = (el(2, 2), el(3, 2), el(2, 3));
        let lin = |i: i32, j: i32| CellId::new(2 * (i + 5 * j) as usize);
        let cell_of = |loc: sm_rt_api::Located| CellId::new(loc.cell.0 as usize & !1);
        // The vertex shared by the three: the circumcentre of their centres.
        let v = p3((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, 0.1);
        for ((i, j), ce) in [((2, 2), a), ((3, 2), b), ((2, 3), c)] {
            let d = v3(ce.x - v.x, ce.y - v.y, 0.0);
            let n = d.x.hypot(d.y);
            let loc = w.locate(v, Some(v3(d.x / n, d.y / n, 0.0)), None, &mut sc).unwrap();
            assert_eq!(cell_of(loc), lin(i, j), "{o:?}: heading into ({i},{j})");
        }
        // An edge midpoint between (2,2) and (3,2): the entered side leads.
        let m = p3(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.1);
        let (dab, dba) = (v3(b.x - a.x, b.y - a.y, 0.0), v3(a.x - b.x, a.y - b.y, 0.0));
        assert_eq!(cell_of(w.locate(m, Some(dab), None, &mut sc).unwrap()), lin(3, 2), "{o:?}");
        assert_eq!(cell_of(w.locate(m, Some(dba), None, &mut sc).unwrap()), lin(2, 2), "{o:?}");
        // Along the shared face: either element or neither (each leaf rules on its own flat with a
        // hashed direction of its local point, as for a Cartesian wall), and the same each time.
        let along = v3(-dab.y, dab.x, 0.0);
        let first = cell_of(w.locate(m, Some(along), None, &mut sc).unwrap());
        assert!(first == lin(2, 2) || first == lin(3, 2) || first == CellId::new(CellId::NONE.0 as usize & !1), "{o:?}: {first:?}");
        assert_eq!(first, cell_of(w.locate(m, Some(along), None, &mut sc).unwrap()));
        // A walk from a centre through the vertex is carried along the shared face (not lost, not
        // ended mid-lattice: `rays_along_or_near_shared_faces_reach_the_lattice_boundary` has the
        // closed-form totals), and repeats exactly.
        let d = v3(v.x - a.x, v.y - a.y, 0.0);
        let n = d.x.hypot(d.y);
        let d = v3(d.x / n, d.y / n, 0.0);
        let org = p3(a.x, a.y, 0.1);
        let (got, end) = trace(&w, org, d).unwrap();
        assert_ne!(end, "lost", "{o:?}");
        let into_first: f64 = got.iter().filter(|g| g.0 / 2 == (2 + 5 * 2) as u32).map(|g| g.1).sum();
        assert!((into_first - P / S3).abs() < 1e-12, "{o:?}: {into_first}");
        assert_eq!(trace(&w, org, d).unwrap().0, got);
    }
}

#[test]
fn fit_oversize_fills_are_refused_and_exact_prisms_accepted() {
    for o in [Or::X, Or::Y] {
        // A prism of pitch 1.05 P in lattice pitch P pokes through the flats: the box test passes
        // for one axis but the probe outside the flats finds cells (CSG fill).
        let spec = lattice_spec(o, 2, 1, ORIGIN());
        let e = build(&spec, &[&hex_pin(o, 1.05 * P, "")]).expect_err("oversize refused");
        match e {
            RtError::Build(v) => assert!(v.iter().any(|i| i.code == "world.lattice_fit"), "{v:?}"),
            other => panic!("{other:?}"),
        }
        // Exact fill: accepted.
        assert!(build(&spec, &[&hex_pin(o, P, "")]).is_ok());
        // A box fill the size of the flats (square) pokes out of the corners: refused. (Probe.)
        let mut sq = Fx::new(10.0);
        sq.complement = ComplementPolicy::Outside;
        sq.bounds = None;
        let b = sq.box_planes(p3(-0.5 * P, -0.5 * P, -ZH), p3(0.5 * P, 0.5 * P, ZH));
        let sides: Vec<HalfSpace> = b.iter().map(|&s| hs(s, true)).collect();
        sq.cell1("sq", &sides);
        assert!(build(&spec, &[&sq]).is_err(), "{o:?}");
    }
    // `outer` stays refused.
    let mut spec = lattice_spec(Or::X, 2, 1, ORIGIN());
    if let ComponentKind::Lattice(l) = &mut spec.components[1] {
        l.outer = Some(ComponentId(0));
    }
    assert!(matches!(build(&spec, &[&hex_pin(Or::X, P, "")]), Err(RtError::Unsupported(_))));
}

#[test]
fn the_two_orientations_and_two_builds_hash_differently_and_repeatably() {
    let h = |o: Or| world(o, 3, 1, ORIGIN()).0.info().structure_hash;
    assert_eq!(h(Or::X), h(Or::X));
    assert_eq!(h(Or::Y), h(Or::Y));
    assert_ne!(h(Or::X), h(Or::Y));
}

#[test]
fn hex_traces_are_bit_identical_for_one_thread_and_many() {
    let (w, l) = world(Or::Y, 5, 2, ORIGIN());
    let mut r = rng("hex-world-det");
    let rays: Vec<sm_geom_api::Ray> = (0..800)
        .map(|_| {
            let c = l.element_center([2, 2, 0]);
            sm_geom_api::Ray { origin: p3(c.x + 2.0 * r.unit(), c.y + 2.0 * r.unit(), 2.0 * r.unit()), dir: random_dir(&mut r) }
        })
        .collect();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let b = w.trace_bundle(&rays, &TraceOptions::default()).unwrap();
            let mut bits: Vec<u64> = b.length.iter().map(|x| x.to_bits()).collect();
            bits.extend(b.cell.iter().map(|&c| u64::from(c)));
            bits.extend(b.region.iter().map(|&c| u64::from(c)));
            bits.extend(b.end.iter().map(|e| format!("{e:?}").len() as u64));
            bits
        })
    };
    let base = sm_testkit::determinism::check(2, 4, run).unwrap();
    assert!(base.len() > 1000);
}

#[test]
fn slot_changes_count_the_elements_visited() {
    let (w, l) = world(Or::X, 5, 1, ORIGIN());
    let (c0, c1) = (l.element_center([0, 2, 0]), l.element_center([4, 2, 0]));
    let mut sc = Scratch::new();
    let d = v3(c1.x - c0.x, c1.y - c0.y, 0.0);
    let d = v3(d.x / d.x.hypot(d.y), d.y / d.x.hypot(d.y), 0.0);
    let mut st = w.init_state(c0, d, None, &mut sc).unwrap();
    let mut changes = 0;
    for _ in 0..100 {
        match w.step(&mut st, f64::INFINITY, &mut sc).unwrap().end {
            StepEnd::SlotChange { .. } => changes += 1,
            StepEnd::CellFace { .. } | StepEnd::RegionBoundary { .. } => {}
            _ => break,
        }
    }
    assert_eq!(changes, 4);
}

/// Closed-form `(lo, hi)` lengths from `org` along `d` of the chain of closed hexagons (slack
/// `slack`, zero-width along-face runs included) that starts at `t = 0`. `hi` is where the ray
/// leaves the lattice footprint; `lo` is where it first runs along an outer flat (only one element
/// holds the run), where ending is as legitimate as continuing, as along a Cartesian wall. Without
/// such a run `lo == hi`.
fn footprint_run(o: Or, l: &Lattice, org: Point3, d: [f64; 2], slack: f64) -> (f64, f64) {
    let nl = flats(o);
    let n = l.dims[0] as usize;
    let centres: Vec<Point3> = (0..l.element_count().unwrap()).map(|lin| l.element_center([(lin % n) as i32, ((lin / n) % l.dims[1] as usize) as i32, 0])).collect();
    let mut ivs: Vec<(f64, f64)> = Vec::new();
    for c in &centres {
        let (mut t0, mut t1) = (0.0f64, f64::INFINITY);
        let mut ok = true;
        for f in &nl {
            let a = (org.x - c.x) * f[0] + (org.y - c.y) * f[1];
            let b = d[0] * f[0] + d[1] * f[1];
            match clip(t0, t1, a, b, -0.5 * P - slack, 0.5 * P + slack) {
                Some((x, y)) => (t0, t1) = (x, y),
                None => ok = false,
            }
        }
        if ok {
            ivs.push((t0, t1));
        }
    }
    ivs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let sd = |c: &Point3, t: f64| nl.iter().map(|f| ((org.x + t * d[0] - c.x) * f[0] + (org.y + t * d[1] - c.y) * f[1]).abs()).fold(0.0, f64::max) - 0.5 * P;
    let (mut end, mut lo) = (0.0, f64::INFINITY);
    for (a, b) in ivs {
        if a > end + 1e-9 {
            break;
        }
        // The first place where only one element holds the ray and it is on that element's flat.
        let on_outer1 = |t: f64| {
            let h: Vec<&Point3> = centres.iter().filter(|c| sd(c, t) <= slack).collect();
            h.len() == 1 && sd(h[0], t) >= -slack
        };
        // A run, not a grazing pass of a vertex: it holds over the next 0.05 as well.
        let on_outer = |t: f64| on_outer1(t) && on_outer1(t + 0.05);
        let steps = ((b - a) / 1e-2).ceil() as usize + 1;
        if let Some(k) = (0..=steps).find(|&k| on_outer((a + 1e-2 * k as f64).min(b))) {
            let (mut u, mut v) = ((a + 1e-2 * (k as f64 - 1.0)).max(a), (a + 1e-2 * k as f64).min(b));
            for _ in 0..60 {
                let m = 0.5 * (u + v);
                if on_outer(m) { v = m } else { u = m }
            }
            lo = lo.min(v);
        }
        end = end.max(b);
    }
    (lo.min(end), end)
}

/// Rays from interior element centres along the six vertex directions and a band of directions
/// around them leave at the outer boundary of the lattice, never through a `Boundary` mid-lattice.
#[test]
fn rays_along_or_near_shared_faces_reach_the_lattice_boundary() {
    let mut exact = 0;
    for o in [Or::X, Or::Y] {
        let (w, l) = world(o, 7, 1, ORIGIN());
        let base = if o == Or::X { 30.0f64 } else { 0.0 };
        for (ci, cj) in [(3, 3), (2, 3), (3, 2)] {
            let c = l.element_center([ci, cj, 0]);
            let org = p3(c.x, c.y, 0.1);
            for k in 0..6 {
                for eps in [0.0, 1e-15, -1e-15, 1e-13, -1e-13, 1e-11, -1e-11, 1e-9, -1e-9] {
                    let a = (base + 60.0 * f64::from(k)).to_radians() + eps;
                    let d = [a.cos(), a.sin()];
                    let what = format!("{o:?} centre ({ci},{cj}) dir {k} eps {eps:e}");
                    // A run within 1e-7 of an outer flat is ambiguous (the tolerance band, 1e-9,
                    // blurs where a ray along a flat is judged to leave it): ending there or
                    // continuing along it are both fine, truncating before it is not.
                    let (lo, wide) = footprint_run(o, &l, org, d, 1e-7);
                    let tight = footprint_run(o, &l, org, d, 1e-12).1;
                    assert!(lo > 1.0, "{what}: closed form {lo}");
                    let (got, end) = trace(&w, org, v3(d[0], d[1], 0.0)).unwrap();
                    assert_ne!(end, "lost", "{what}");
                    assert_eq!(end, "boundary", "{what}");
                    let total: f64 = got.iter().map(|g| g.1).sum();
                    if lo < wide - 1e-6 {
                        assert!(total >= lo - 1e-6 && total <= wide + 1e-6, "{what}: total {total} outside [{lo}, {wide}]");
                    } else {
                        // The tolerance band also blurs the exit of a ray 1e-9 rad off a flat.
                        let tol = if eps.abs() >= 1e-9 { 2e-8 } else { 2e-9 };
                        assert!((total - tight).abs() < tol, "{what}: total {total} vs closed form {tight}");
                        exact += 1;
                    }
                }
            }
        }
    }
    // Most rays have no ambiguity at all: those must match the closed form.
    assert!(exact > 150, "{exact}");
}
