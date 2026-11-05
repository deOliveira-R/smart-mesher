//! A closed (non-periodic) B-spline face with a large control net and many short boundary edges
//! must decode in time proportional to the file, or be refused with `step.decode_budget`. The
//! closed-face split projects every boundary edge onto the surface; a global projection per edge
//! would cost `E x 64 nu nv` surface evaluations.

use std::fmt::Write as _;

mod common;

use sm_cadio::{NoteCode, NoteSeverity, ReadOptions, StepError, read_step};

fn r(x: f64) -> String {
    format!("{x:.15E}")
}

struct W {
    s: String,
    id: u64,
}

impl W {
    fn rec(&mut self, body: &str) -> u64 {
        self.id += 1;
        let _ = writeln!(self.s, "#{} = {body};", self.id);
        self.id
    }
    fn point(&mut self, p: [f64; 3]) -> u64 {
        self.rec(&format!("CARTESIAN_POINT('',({},{},{}))", r(p[0]), r(p[1]), r(p[2])))
    }
    fn dir(&mut self, d: [f64; 3]) -> u64 {
        self.rec(&format!("DIRECTION('',({},{},{}))", r(d[0]), r(d[1]), r(d[2])))
    }
    fn vertex(&mut self, p: [f64; 3]) -> u64 {
        let pid = self.point(p);
        self.rec(&format!("VERTEX_POINT('',#{pid})"))
    }
    fn edge(&mut self, va: u64, vb: u64, a: [f64; 3], b: [f64; 3]) -> u64 {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let pt = self.point(a);
        let dr = self.dir([d[0] / l, d[1] / l, d[2] / l]);
        let vc = self.rec(&format!("VECTOR('',#{dr},{})", r(l)));
        let ln = self.rec(&format!("LINE('',#{pt},#{vc})"));
        self.rec(&format!("EDGE_CURVE('',#{va},#{vb},#{ln},.T.)"))
    }
    fn face(&mut self, oes: &[(u64, bool)], surf: u64) -> u64 {
        let list: Vec<String> = oes.iter().map(|&(e, f)| format!("#{}", self.rec(&format!("ORIENTED_EDGE('',*,*,#{e},{})", if f { ".T." } else { ".F." })))).collect();
        let lp = self.rec(&format!("EDGE_LOOP('',({}))", list.join(",")));
        let fb = self.rec(&format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
        self.rec(&format!("ADVANCED_FACE('',(#{fb}),#{surf},.T.)"))
    }
    fn plane(&mut self, z: f64, up: bool) -> u64 {
        let o = self.point([0.0, 0.0, z]);
        let n = self.dir([0.0, 0.0, if up { 1.0 } else { -1.0 }]);
        let x = self.dir([1.0, 0.0, 0.0]);
        let ax = self.rec(&format!("AXIS2_PLACEMENT_3D('',#{o},#{n},#{x})"));
        self.rec(&format!("PLANE('',#{ax})"))
    }
}

/// A polygonal prism of `sides` sides: its lateral surface is one closed B-spline face (degree 1
/// around, degree 3 along the axis, `rows` control rows), bounded by both polygons and the seam
/// used twice; every polygon side is cut into `k` line edges. Returns the file, the exact volume
/// and the number of edges.
fn prism(sides: usize, rows: usize, k: usize) -> (Vec<u8>, f64, usize) {
    stack(sides, rows, k, &[0])
}

/// [`prism`] with one layer of height 20 per entry of `seams`, stacked: neighbouring layers
/// share the polygon between them, and layer `l` has its seam at corner `seams[l]`, so its
/// generators cut the shared polygon at other points than its neighbour's.
fn stack(sides: usize, rows: usize, k: usize, seams: &[usize]) -> (Vec<u8>, f64, usize) {
    let (rad, h) = (10.0, 20.0);
    let nu = sides + 1;
    let mut w = W { s: String::with_capacity(seams.len() * (nu * rows * 70 + sides * k * 600)), id: 5 };
    w.s.push_str("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('generated'),'2;1');\nFILE_NAME('prism','1970-01-01T00:00:00',('x'),(''),'gen','',' ');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    w.s.push_str("#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n#3 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n");
    w.s.push_str("#4 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','confusion accuracy');\n");
    w.s.push_str("#5 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#4)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );\n");
    let corner = |i: usize, z: f64| {
        let a = core::f64::consts::TAU * (i % sides) as f64 / sides as f64;
        [rad * a.cos(), rad * a.sin(), z]
    };
    let ends = |n: usize, e: &str| -> String { (0..n).map(|i| if i == 0 || i == n - 1 { e.to_owned() } else { "1".to_owned() }).collect::<Vec<_>>().join(",") };
    let knots = |n: usize| -> String { (0..n).map(|i| r(i as f64)).collect::<Vec<_>>().join(",") };
    let m = sides * k;
    let point_at = |q: usize, z: f64| {
        let (i, f) = ((q % m) / k, (q % k) as f64 / k as f64);
        let (a, b) = (corner(i, z), corner(i + 1, z));
        [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), z]
    };
    let ring = |w: &mut W, z: f64| -> (Vec<u64>, Vec<u64>) {
        let vs: Vec<u64> = (0..m).map(|q| w.vertex(point_at(q, z))).collect();
        let es: Vec<u64> = (0..m).map(|q| w.edge(vs[q], vs[(q + 1) % m], point_at(q, z), point_at(q + 1, z))).collect();
        (vs, es)
    };
    let mut rings = vec![ring(&mut w, 0.0)];
    let mut faces = Vec::new();
    for (l, &o) in seams.iter().enumerate() {
        let (z0, z1) = (h * l as f64, h * (l + 1) as f64);
        let mut net = Vec::with_capacity(nu);
        for i in 0..nu {
            let col: Vec<String> = (0..rows).map(|j| format!("#{}", w.point(corner(o + i, z0 + h * j as f64 / (rows - 1) as f64)))).collect();
            net.push(format!("({})", col.join(",")));
        }
        let nkv = rows - 2;
        let surf = w.rec(&format!("B_SPLINE_SURFACE_WITH_KNOTS('',1,3,({}),.UNSPECIFIED.,.T.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)", net.join(","), ends(nu, "2"), ends(nkv, "4"), knots(nu), knots(nkv)));
        let top = ring(&mut w, z1);
        let (vb, eb) = &rings[l];
        let (vt, et) = &top;
        let q0 = o * k;
        let seam = w.edge(vb[q0 % m], vt[q0 % m], point_at(q0, z0), point_at(q0, z1));
        // Lateral loop, counterclockwise in (u, v): bottom forward from the seam corner, up the
        // seam at u = hi, top backward, down the seam at u = lo.
        let mut lat: Vec<(u64, bool)> = (0..m).map(|i| (eb[(q0 + i) % m], true)).collect();
        lat.push((seam, true));
        lat.extend((0..m).rev().map(|i| (et[(q0 + i) % m], false)));
        lat.push((seam, false));
        faces.push(w.face(&lat, surf));
        rings.push(top);
    }
    let pb = w.plane(0.0, false);
    let bottom: Vec<(u64, bool)> = rings[0].1.iter().rev().map(|&e| (e, false)).collect();
    faces.push(w.face(&bottom, pb));
    let pt = w.plane(h * seams.len() as f64, true);
    let top: Vec<(u64, bool)> = rings[seams.len()].1.iter().map(|&e| (e, true)).collect();
    faces.push(w.face(&top, pt));
    let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
    let sh = w.rec(&format!("CLOSED_SHELL('',({}))", list.join(",")));
    let so = w.rec(&format!("MANIFOLD_SOLID_BREP('prism',#{sh})"));
    w.rec(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{so}),#5)"));
    w.s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    let vol = 0.5 * sides as f64 * rad * rad * (core::f64::consts::TAU / sides as f64).sin() * h * seams.len() as f64;
    (w.s.into_bytes(), vol, (seams.len() + 1) * m + seams.len())
}

fn env(name: &str, default: usize) -> usize {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// Decode `prism(sides, rows, k)`; panics unless it reads (the closed face split into four
/// pieces) or is refused with `DecodeBudget`. Returns the work done, the model
/// when it read, and the exact volume.
fn run(sides: usize, rows: usize, k: usize) -> (sm_cadio::Work, Option<sm_cadio::BrepModel>, f64) {
    let (bytes, vol, edges) = prism(sides, rows, k);
    let (res, dt) = sm_cadio::read_step_metered(&bytes, &ReadOptions::default());
    match res {
        Ok(m) => {
            let split = m.notes.iter().any(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Info);
            eprintln!("prism sides={sides} rows={rows} k={k}: {} bytes, {edges} edges: read, {dt:?}, {} faces, split={split}", bytes.len(), m.solids[0].faces.len());
            assert!(split, "the closed face was not split: {:?}", m.notes);
            assert_eq!(m.solids[0].faces.len(), 2 + 4);
            (dt, Some(m), vol)
        }
        Err(e) => {
            eprintln!("prism sides={sides} rows={rows} k={k}: {} bytes, {edges} edges: refused, {dt:?}: {e}", bytes.len());
            assert!(matches!(e, StepError::DecodeBudget { .. }), "{e:?}");
            (dt, None, vol)
        }
    }
}

/// A small instance reads, splits, realizes valid, and has the prism's exact volume (to the
/// 1e-6 of the platform quadrature over piecewise-linear patches).
#[test]
fn a_small_closed_bspline_prism_reads_splits_and_measures() {
    let (_, m, vol) = run(5, 4, 3);
    let m = m.expect("reads");
    let (h, r) = common::import_into(common::cell_complex, &m);
    let v = h.as_view();
    assert!(common::issues(v).is_empty(), "{:?}", common::issues(v));
    let got = common::volume(v, r.volumes[0]);
    assert!(common::rel(got, vol) <= 1e-6, "{got:.12} vs {vol:.12}");
}

/// A 200 x 200 net bounded by 1,195 edges (no vertex on a generator): decoded in time
/// proportional to the file, or refused. Before round 4 every edge paid a global projection (a
/// 1601 x 1601 grid): 173 s at 100 x 100. Also checked against a half-size instance, so a return
/// to `E nu nv` work (8x per doubling) fails independently of machine speed. Sizes can be
/// overridden for measurement with `SPLIT_SIDES`, `SPLIT_ROWS`, `SPLIT_K`.
#[test]
fn a_large_closed_bspline_face_with_many_edges_decodes_in_linear_time() {
    let (sides, rows, k) = (env("SPLIT_SIDES", 199), env("SPLIT_ROWS", 200), env("SPLIT_K", 3));
    let (w_half, w_full) = (run(sides / 2, rows / 2, k).0, run(sides, rows, k).0);
    // Global (grid search) projections are capped per face, however many edges bound it.
    assert!(w_full.global_projections <= sm_cadio::MAX_GLOBAL as u64, "{w_full:?}");
    // Linear work: 2x the edges is about 2x the Newton runs (`E nu nv` work would show up as
    // far more runs or global searches).
    assert!(w_full.newton_runs <= 3 * w_half.newton_runs.max(100), "full {w_full:?} vs half {w_half:?}");
}

/// Plain prisms whose generators pass through boundary vertices (polygon corners, and the shared
/// ring of same-seam layers) are split like any other: the crossing is the vertex itself.
#[test]
fn generators_through_boundary_vertices_split_the_face() {
    // (sides, k, seams): square and triangle with one edge per side, corner cuts with sub-vertices,
    // and layers whose seams coincide (the generators of each layer meet the shared ring at the
    // vertices the layer below created).
    let cases: [(usize, usize, &[usize]); 7] = [(4, 1, &[0]), (3, 1, &[0]), (4, 3, &[1]), (8, 3, &[0]), (4, 1, &[0, 0]), (4, 3, &[0, 0, 0]), (6, 2, &[0, 3, 3])];
    for (ci, (sides, k, seams)) in cases.into_iter().enumerate() {
        let (bytes, vol, _) = stack(sides, 4, k, seams);
        let m = read_step(&bytes, &ReadOptions::default()).expect("reads");
        let splits = m.notes.iter().filter(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Info).count();
        assert_eq!(splits, seams.len(), "sides={sides} k={k} seams={seams:?}: {:?}", m.notes);
        assert!(m.notes.iter().all(|n| n.severity != NoteSeverity::Warning), "{:?}", m.notes);
        assert_eq!(m.solids[0].faces.len(), 2 + 4 * seams.len());
        // Realizing measures the volume (twice); that is slow in a debug build, so only the first case (set `SPLIT_ALL` for all).
        if cfg!(debug_assertions) && ci > 0 && std::env::var_os("SPLIT_ALL").is_none() {
            continue;
        }
        let (h, r) = common::import_into(common::cell_complex, &m);
        let v = h.as_view();
        assert!(common::issues(v).is_empty(), "sides={sides} k={k}: {:?}", common::issues(v));
        let got = common::volume(v, r.volumes[0]);
        assert!(common::rel(got, vol) <= 1e-6, "sides={sides} k={k} seams={seams:?}: {got:.12} vs {vol:.12}");
    }
}

/// Two closed B-spline faces stacked on a shared polygon, with their seams at different corners:
/// both split (the second one cuts the sub-edges the first one made), the solid stays closed,
/// realizes valid and has the exact volume.
#[test]
fn two_split_faces_sharing_cut_edges_both_split() {
    let (bytes, vol, _) = stack(5, 4, 3, &[0, 2]);
    let m = read_step(&bytes, &ReadOptions::default()).expect("reads");
    let splits = m.notes.iter().filter(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Info).count();
    assert_eq!(splits, 2, "{:?}", m.notes);
    let s = &m.solids[0];
    assert_eq!(s.faces.len(), 2 + 2 * 4);
    // 3 rings of 15 edges, each cut 3 times per adjacent split face (the shared ring 6 times),
    // 2 seams, 2 x 3 generators.
    assert_eq!(s.edges.len(), 3 * 15 + 3 + 6 + 3 + 2 + 6);
    let (h, r) = common::import_into(common::cell_complex, &m);
    let v = h.as_view();
    assert!(common::issues(v).is_empty(), "{:?}", common::issues(v));
    let got = common::volume(v, r.volumes[0]);
    assert!(common::rel(got, vol) <= 1e-6, "{got:.12} vs {vol:.12}");
}

/// Many split faces in one solid: the split is one pass with one closure check, so doubling
/// the layers about doubles the closure-check work (before round 4 every split cloned and checked the whole
/// solid). `SPLIT_LAYERS` overrides the size.
#[test]
fn many_split_faces_in_one_solid_cost_linear_time() {
    let layers = env("SPLIT_LAYERS", 200);
    let time = |n: usize| {
        let seams: Vec<usize> = (0..n).map(|l| 2 * (l % 2)).collect();
        let (bytes, _, _) = stack(5, 4, 3, &seams);
        let (m, w) = sm_cadio::read_step_metered(&bytes, &ReadOptions::default());
        let m = m.expect("reads");
        assert_eq!(m.solids[0].faces.len(), 2 + 4 * n);
        eprintln!("{n} layers: {} bytes, {w:?}", bytes.len());
        w.closure_coedges
    };
    let (half, full) = (time(layers / 2), time(layers));
    // Quadratic work (one whole-solid check per split) would be 4x per doubling.
    assert!(full <= 3 * half.max(1), "full {full} vs half {half} coedge visits");
}

/// Counters are per read: the same file gives the same totals every time, whatever else runs
/// (no process-wide accumulation), and `newton_runs` is the cost of that one read.
#[test]
fn work_is_counted_per_run_and_does_not_accumulate() {
    let (bytes, _, _) = prism(5, 4, 3);
    let o = ReadOptions::default();
    let (_, a) = sm_cadio::read_step_metered(&bytes, &o);
    let (_, b) = sm_cadio::read_step_metered(&bytes, &o);
    assert!(a.newton_runs > 0 && a.solids_decoded == 1, "{a:?}");
    assert_eq!(a, b);
    // Concurrent reads do not mix their counters.
    let totals: Vec<sm_cadio::Work> = std::thread::scope(|s| {
        let hs: Vec<_> = (0..4).map(|_| s.spawn(|| sm_cadio::read_step_metered(&bytes, &o).1)).collect();
        hs.into_iter().map(|h| h.join().expect("thread")).collect()
    });
    assert!(totals.iter().all(|t| *t == a), "{totals:?} vs {a:?}");
}
