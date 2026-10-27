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

/// The tests of this binary run one at a time: two of them compare wall times.
fn serial() -> std::sync::MutexGuard<'static, ()> {
    static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());
    SERIAL.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

fn env(name: &str, default: usize) -> usize {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// Decode `prism(sides, rows, k)`; panics unless it reads (the closed face split into four
/// pieces) or is refused with `DecodeBudget`. Returns the decode time in seconds, the model
/// when it read, and the exact volume.
fn run(sides: usize, rows: usize, k: usize) -> (f64, Option<sm_cadio::BrepModel>, f64) {
    let (bytes, vol, edges) = prism(sides, rows, k);
    let t = std::time::Instant::now();
    let res = read_step(&bytes, &ReadOptions::default());
    let dt = t.elapsed().as_secs_f64();
    match res {
        Ok(m) => {
            let split = m.notes.iter().any(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Info);
            eprintln!("prism sides={sides} rows={rows} k={k}: {} bytes, {edges} edges: read in {dt:.3} s, {} faces, split={split}", bytes.len(), m.solids[0].faces.len());
            assert!(split, "the closed face was not split: {:?}", m.notes);
            assert_eq!(m.solids[0].faces.len(), 2 + 4);
            (dt, Some(m), vol)
        }
        Err(e) => {
            eprintln!("prism sides={sides} rows={rows} k={k}: {} bytes, {edges} edges: refused in {dt:.3} s: {e}", bytes.len());
            assert!(matches!(e, StepError::DecodeBudget { .. }), "{e:?}");
            (dt, None, vol)
        }
    }
}

/// A small instance reads, splits, realizes valid, and has the prism's exact volume (to the
/// 1e-6 of the platform quadrature over piecewise-linear patches).
#[test]
fn a_small_closed_bspline_prism_reads_splits_and_measures() {
    let _serial = serial();
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
    let _serial = serial();
    let (sides, rows, k) = (env("SPLIT_SIDES", 199), env("SPLIT_ROWS", 200), env("SPLIT_K", 3));
    let (t_half, _, _) = run(sides / 2, rows / 2, k);
    let (t_full, _, _) = run(sides, rows, k);
    let limit = if cfg!(debug_assertions) { 60.0 } else { 10.0 };
    assert!(t_full < limit, "{t_full:.3} s over {limit} s");
    // Linear work: 2x the edges, 4x the net and the file. `E nu nv` work gives 8x.
    assert!(t_full < 6.0 * t_half.max(0.05), "full {t_full:.3} s vs half {t_half:.3} s");
}

/// Two closed B-spline faces stacked on a shared polygon, with their seams at different corners:
/// both split (the second one cuts the sub-edges the first one made), the solid stays closed,
/// realizes valid and has the exact volume.
#[test]
fn two_split_faces_sharing_cut_edges_both_split() {
    let _serial = serial();
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
/// the layers about doubles the time (before round 4 every split cloned and checked the whole
/// solid). `SPLIT_LAYERS` overrides the size.
#[test]
fn many_split_faces_in_one_solid_cost_linear_time() {
    let _serial = serial();
    let layers = env("SPLIT_LAYERS", 200);
    let time = |n: usize| {
        let seams: Vec<usize> = (0..n).map(|l| 2 * (l % 2)).collect();
        let (bytes, _, _) = stack(5, 4, 3, &seams);
        let t = std::time::Instant::now();
        let m = read_step(&bytes, &ReadOptions::default()).expect("reads");
        let dt = t.elapsed().as_secs_f64();
        assert_eq!(m.solids[0].faces.len(), 2 + 4 * n);
        eprintln!("{n} layers: {} bytes, read in {dt:.3} s", bytes.len());
        dt
    };
    let (half, full) = (time(layers / 2), time(layers));
    assert!(full < 4.0 * half.max(0.05), "full {full:.3} s vs half {half:.3} s");
}
