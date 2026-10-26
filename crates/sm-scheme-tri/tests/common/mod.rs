//! Shared helpers for the integration tests.
#![allow(dead_code)]

use sm_core::Point3;
use sm_math::Sign;
use sm_math::predicates::{incircle, orient2d};
use sm_math::sum::NeumaierSum;
use sm_scheme_tri::cdt::{Cdt, Geometry2, Pslg, Segment, SegmentKind};
use sm_testkit::loops2d::Region2;
use std::collections::BTreeMap;

/// The plane z = 0, chart = identity.
pub struct Flat(pub f64);

impl Geometry2 for Flat {
    fn lift(&self, q: [f64; 2]) -> Point3 {
        Point3::new(q[0], q[1], 0.0)
    }
    fn size(&self, _p: Point3) -> f64 {
        self.0
    }
    fn is_isometric(&self) -> bool {
        true
    }
}

/// PSLG of a region: all loops as boundary segments.
pub fn pslg_of(r: &Region2) -> Pslg {
    let mut p = Pslg::default();
    for l in r.loops() {
        let base = p.points.len() as u32;
        let n = l.len() as u32;
        for q in l {
            p.points.push([q.0, q.1]);
        }
        for i in 0..n {
            p.segments.push(Segment {
                a: base + i,
                b: base + (i + 1) % n,
                kind: SegmentKind::Boundary,
            });
        }
    }
    p
}

/// xorshift64* for reproducible shuffles.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// Shuffle the points of a PSLG (segments remapped).
pub fn shuffled(p: &Pslg, seed: u64) -> Pslg {
    let n = p.points.len();
    let mut perm: Vec<usize> = (0..n).collect();
    let mut rng = Rng(seed | 1);
    for i in (1..n).rev() {
        perm.swap(i, rng.below(i + 1));
    }
    // perm[new] = old
    let mut inv = vec![0u32; n];
    for (new, &old) in perm.iter().enumerate() {
        inv[old] = new as u32;
    }
    let points = perm.iter().map(|&o| p.points[o]).collect();
    let mut segments: Vec<Segment> = p
        .segments
        .iter()
        .map(|s| Segment { a: inv[s.a as usize], b: inv[s.b as usize], kind: s.kind })
        .collect();
    for i in (1..segments.len()).rev() {
        segments.swap(i, rng.below(i + 1));
    }
    Pslg { points, segments }
}

/// Triangles as sorted coordinate triples (bits), sorted: comparable across relabellings.
pub fn tri_set(c: &Cdt) -> Vec<[[u64; 2]; 3]> {
    let pts = c.points();
    let mut out: Vec<[[u64; 2]; 3]> = c
        .triangles()
        .iter()
        .map(|t| {
            let mut v = [0, 1, 2].map(|k| {
                let p = pts[t[k] as usize];
                [p[0].to_bits(), p[1].to_bits()]
            });
            v.sort();
            v
        })
        .collect();
    out.sort();
    out
}

/// Every triangle is exactly counter-clockwise.
pub fn all_positive(c: &Cdt) -> bool {
    let pts = c.points();
    c.triangles().iter().all(|t| {
        orient2d(pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]) == Sign::Positive
    })
}

/// Area (compensated) of the triangulation.
pub fn area(c: &Cdt) -> f64 {
    let pts = c.points();
    let mut s = NeumaierSum::new();
    for t in c.triangles() {
        let (a, b, d) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        s.add(0.5 * ((b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0])));
    }
    s.value()
}

/// Count unconstrained interior edges whose opposite vertex is strictly inside the circumcircle.
pub fn delaunay_violations(c: &Cdt) -> usize {
    let pts = c.points();
    let tris = c.triangles();
    let nb = c.neighbors();
    let mut bad = 0;
    for (t, tv) in tris.iter().enumerate() {
        for i in 0..3 {
            let n = nb[t][i];
            if n == Cdt::NO_NEIGHBOR {
                continue;
            }
            let (a, b) = (tv[(i + 1) % 3], tv[(i + 2) % 3]);
            if c.is_constrained(a, b) {
                continue;
            }
            let nv = tris[n as usize];
            let opp = nv.iter().copied().find(|&x| x != a && x != b).unwrap();
            if incircle(
                pts[tv[0] as usize],
                pts[tv[1] as usize],
                pts[tv[2] as usize],
                pts[opp as usize],
            ) == Sign::Positive
            {
                bad += 1;
            }
        }
    }
    bad
}

/// Edges (as unordered point-id pairs) that belong to exactly one triangle.
pub fn boundary_edges(c: &Cdt) -> Vec<(u32, u32)> {
    let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in c.triangles() {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    count.into_iter().filter(|(_, n)| *n == 1).map(|(e, _)| e).collect()
}

// ---------------------------------------------------------------- driver helpers

use sm_core::EntityId;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellIx, MeshRead, SerialAlloc, cell_volume};
use sm_model_api::Model;
use sm_scheme_api::{
    Driver, Executor, Firm, Firmness, IntervalRequest, Meshed, MeshInput, MeshIntent,
    ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeRegistry, WholeFaces,
};
use sm_topo_api::CellComplex;

/// Registry with the real curve schemes and this crate's schemes.
pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).expect("curve");
    sm_scheme_tri::register(&mut r).expect("tri");
    r
}

/// Mesh `faces` of `topo` surface-only with `trimesh`, edges given hard counts.
pub fn mesh_faces(
    topo: &CellComplex,
    faces: &[EntityId],
    counts: &[(EntityId, u32)],
    scheme: &str,
    exec: &dyn Executor,
) -> Result<Meshed<ReferenceMesh>, String> {
    let reg = registry();
    mesh_faces_with(&reg, topo, faces, counts, scheme, exec, None)
}

pub fn mesh_faces_with(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    faces: &[EntityId],
    counts: &[(EntityId, u32)],
    scheme: &str,
    exec: &dyn Executor,
    size: Option<&dyn sm_scheme_api::SizeField>,
) -> Result<Meshed<ReferenceMesh>, String> {
    let model = Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let mut driver = Driver::new(reg, &solver, &picker);
    driver.size = size;
    let input = MeshInput::new(topo, &model, &WholeFaces, faces);
    let mut intent = MeshIntent::default();
    for &(e, n) in counts {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    }
    for &f in faces {
        intent.set_scheme(f, SchemeChoice::new(scheme, Firmness::Hard));
    }
    driver
        .mesh(&input, &intent, exec, ReferenceBuilder::surface(), &mut SerialAlloc::new())
        .map_err(|e| e.to_string())
}

/// Triangle areas via the mesh (compensated).
pub fn mesh_area(m: &ReferenceMesh) -> f64 {
    let mut s = NeumaierSum::new();
    for c in 0..m.cell_count() {
        s.add(cell_volume(m, CellIx::new(c)));
    }
    s.value()
}

// ---------------------------------------------------------------- mesh extraction

use sm_core::Vector3;
use sm_mesh_api::{FaceIx, FaceKind, VertexIx};

/// Vertex indices of every triangle of a surface mesh.
pub fn tri_vertices(m: &ReferenceMesh) -> Vec<[usize; 3]> {
    (0..m.cell_count())
        .map(|c| {
            let v = m.cell_corners(CellIx::new(c)).expect("corners");
            assert_eq!(v.len(), 3, "cell {c} is not a triangle");
            [v[0].get(), v[1].get(), v[2].get()]
        })
        .collect()
}

pub fn tri_points(m: &ReferenceMesh) -> Vec<[Point3; 3]> {
    tri_vertices(m)
        .iter()
        .map(|t| [m.point(t[0]), m.point(t[1]), m.point(t[2])])
        .collect()
}

/// Angles of a 3-D triangle in degrees.
pub fn angles_deg(t: &[Point3; 3]) -> [f64; 3] {
    let ang = |a: Point3, b: Point3, c: Point3| {
        let (u, v): (Vector3, Vector3) = (b - a, c - a);
        u.cross(v).norm().atan2(u.dot(v)).to_degrees()
    };
    [ang(t[0], t[1], t[2]), ang(t[1], t[2], t[0]), ang(t[2], t[0], t[1])]
}

pub fn min_angle_deg(m: &ReferenceMesh) -> f64 {
    tri_points(m)
        .iter()
        .flat_map(|t| angles_deg(t))
        .fold(f64::INFINITY, f64::min)
}

/// Circumradius of a 3-D triangle.
pub fn circumradius(t: &[Point3; 3]) -> f64 {
    let (a, b, c) = ((t[1] - t[2]).norm(), (t[2] - t[0]).norm(), (t[0] - t[1]).norm());
    let k = 0.5 * (t[1] - t[0]).cross(t[2] - t[0]).norm();
    a * b * c / (4.0 * k)
}

/// Mesh edges `(lo, hi)` of a dim-2 mesh with their number of cells.
pub fn edge_use(m: &ReferenceMesh) -> BTreeMap<(usize, usize), u32> {
    let mut out: BTreeMap<(usize, usize), u32> = BTreeMap::new();
    for t in tri_vertices(m) {
        for i in 0..3 {
            let (a, b) = (t[i], t[(i + 1) % 3]);
            *out.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    out
}

/// The vertex pairs of the mesh facets that are on the boundary of the mesh.
pub fn boundary_pairs(m: &ReferenceMesh) -> Vec<(usize, usize)> {
    let mut v: Vec<(usize, usize)> = (0..m.face_count())
        .filter(|&f| m.face_kind(FaceIx::new(f)) != FaceKind::Interior)
        .map(|f| {
            let fv = m.face_vertices(FaceIx::new(f));
            let (a, b): (VertexIx, VertexIx) = (fv[0], fv[1]);
            (a.get().min(b.get()), a.get().max(b.get()))
        })
        .collect();
    v.sort();
    v
}

/// The consecutive node pairs of the given edges (the boundary segments the face must keep),
/// as mesh vertex pairs.
pub fn expected_boundary(
    out: &Meshed<ReferenceMesh>,
    topo: &CellComplex,
    edges: &[EntityId],
) -> Vec<(usize, usize)> {
    use sm_topo_api::TopologyViewExt;
    let _ = topo;
    let mut v = Vec::new();
    for &e in edges {
        let n = out.plan.intervals(e).expect("intervals");
        let (a, b) = topo.edge_ends(e).expect("ends");
        let mut keys = vec![sm_scheme_api::NodeKey::vertex(a)];
        keys.extend((1..n).map(|i| sm_scheme_api::NodeKey::new(e, i)));
        keys.push(sm_scheme_api::NodeKey::vertex(b));
        for w in keys.windows(2) {
            let (x, y) = (
                out.vertex_of(w[0]).expect("vertex").0 as usize,
                out.vertex_of(w[1]).expect("vertex").0 as usize,
            );
            v.push((x.min(y), x.max(y)));
        }
    }
    v.sort();
    v
}

pub fn euler(m: &ReferenceMesh) -> i64 {
    m.vertex_count() as i64 - m.face_count() as i64 + m.cell_count() as i64
}

// ---------------------------------------------------------------- probe scheme

use sm_scheme_api::{
    Applicability, Caps, GenCtx, GenerateFailure, Params, Patch, PlanCtx, Scheme, SchemeDim,
    Target,
};
use sm_scheme_tri::{TriParams, TriScheme, triangulate_chart};
use std::sync::{Arc, Mutex};

/// What the probe saw of one face.
#[derive(Clone, Debug, Default)]
pub struct Seen {
    pub triangles: usize,
    pub points: usize,
    pub delaunay_violations: usize,
    pub unfixable: usize,
    pub min_angle: f64,
    pub isometric: bool,
    pub uv_area: f64,
}

/// `trimesh` that records chart-space facts about every face it generates.
pub struct Probe {
    pub seen: Arc<Mutex<Vec<Seen>>>,
}

impl Scheme for Probe {
    fn name(&self) -> &str {
        "trimesh"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, p: &Params) -> Caps {
        TriScheme::trimesh().caps(p)
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, p: &Params) -> Applicability {
        TriScheme::trimesh().applicability(cx, t, p)
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut sm_scheme_api::Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        TriScheme::trimesh().requirements(cx, t, p, out)
    }
    fn generate(&self, cx: &GenCtx<'_>, t: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        let Target::Surface(face) = t else { panic!("surface") };
        let prm = TriParams::from_params(p).expect("params");
        let mut ft = triangulate_chart(cx, face, &prm)
            .map_err(|e| GenerateFailure::new(t, "trimesh", e.kind(), e.to_string()))?;
        let (min_angle, _) = {
            let c = &ft.chart;
            ft.cdt.min_angle(c).unwrap_or((0.0, [0; 3]))
        };
        // Area in (u, v): sum of triangle areas with boundary uv exact and Steiner uv from the chart.
        let n_in = ft.keys.len();
        let uv_of = |i: usize, ft: &sm_scheme_tri::FaceTriangulation<'_>| -> [f64; 2] {
            let q = ft.cdt.points()[i];
            ft.chart.to_uv(q)
        };
        let _ = n_in;
        let mut area = sm_math::sum::NeumaierSum::new();
        for tr in ft.cdt.triangles() {
            let (a, b, c) = (uv_of(tr[0] as usize, &ft), uv_of(tr[1] as usize, &ft), uv_of(tr[2] as usize, &ft));
            area.add(0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])));
        }
        self.seen.lock().unwrap().push(Seen {
            triangles: ft.cdt.triangles().len(),
            points: ft.cdt.points().len(),
            delaunay_violations: delaunay_violations(&ft.cdt),
            unfixable: ft.report.unfixable.len(),
            min_angle,
            isometric: ft.chart.is_isometric(),
            uv_area: area.value(),
        });
        ft.into_patch(cx, face, &prm, t, "trimesh")
    }
}

/// Real curve schemes plus the probe in place of `trimesh`.
pub fn probe_registry() -> (SchemeRegistry, Arc<Mutex<Vec<Seen>>>) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).expect("curve");
    r.register(Box::new(Probe { seen: seen.clone() })).expect("probe");
    (r, seen)
}

/// Like [`mesh_faces_with`] with scheme parameters.
pub fn mesh_faces_params(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    faces: &[EntityId],
    counts: &[(EntityId, u32)],
    params: serde_json::Value,
    exec: &dyn Executor,
    size: Option<&dyn sm_scheme_api::SizeField>,
) -> Result<Meshed<ReferenceMesh>, sm_scheme_api::SchemeError> {
    let model = Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let mut driver = Driver::new(reg, &solver, &picker);
    driver.size = size;
    let input = MeshInput::new(topo, &model, &WholeFaces, faces);
    let mut intent = MeshIntent::default();
    for &(e, n) in counts {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    }
    for &f in faces {
        intent.set_scheme(f, SchemeChoice::with_params("trimesh", params.clone(), Firmness::Hard));
    }
    driver.mesh(&input, &intent, exec, ReferenceBuilder::surface(), &mut SerialAlloc::new())
}

// ---------------------------------------------------------------- fixtures

use sm_scheme_api::geom;
use sm_testkit::topo::FaceFixture;
use sm_topo_api::fixtures::with_op;
use sm_topo_api::{CurveCarrier, OpId, Sense, TopologyEdit};

/// Interval counts `round(length / h)` (at least 1) for every edge of the fixture.
pub fn counts_for(fx: &FaceFixture, h: f64) -> Vec<(EntityId, u32)> {
    fx.edges
        .iter()
        .flatten()
        .map(|&e| {
            let len = geom::edge_length(&fx.complex, e).expect("length");
            (e, ((len / h).round() as u32).max(1))
        })
        .collect()
}

/// Every edge of the fixture, flat.
pub fn all_edges(fx: &FaceFixture) -> Vec<EntityId> {
    fx.edges.iter().flatten().copied().collect()
}

/// Embed straight wires (between new interior vertices) and isolated points in the face of a
/// planar fixture at `z`. Returns `(wire edges, isolated vertices)`.
pub fn embed_wires(
    fx: &mut FaceFixture,
    z: f64,
    wires: &[([f64; 2], [f64; 2])],
    points: &[[f64; 2]],
) -> (Vec<EntityId>, Vec<EntityId>) {
    use sm_geom_api::{CurveDef, Interval, Line};
    let face = fx.face;
    with_op(&mut fx.complex, OpId(2), |t| {
        let mut es = Vec::new();
        let mut vs = Vec::new();
        for (a, b) in wires {
            let (pa, pb) = (Point3::new(a[0], a[1], z), Point3::new(b[0], b[1], z));
            let va = t.make_vertex(pa)?;
            let vb = t.make_vertex(pb)?;
            let line = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(pa, pb - pa)?))?;
            let e = t.make_edge(
                va,
                vb,
                Some(CurveCarrier {
                    curve: line,
                    range: Interval::new(0.0, (pb - pa).norm()),
                    sense: Sense::Forward,
                }),
            )?;
            t.embed(face, e)?;
            es.push(e);
        }
        for p in points {
            let v = t.make_vertex(Point3::new(p[0], p[1], z))?;
            t.embed(face, v)?;
            vs.push(v);
        }
        Ok((es, vs))
    })
    .expect("embed")
}

/// Plan and generate without committing or re-planning: the structured scheme failure.
pub fn generate_only(
    reg: &SchemeRegistry,
    topo: &CellComplex,
    faces: &[EntityId],
    counts: &[(EntityId, u32)],
    params: serde_json::Value,
) -> Result<sm_scheme_api::PatchSet, sm_scheme_api::SchemeError> {
    let model = Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(reg, &solver, &picker);
    let input = MeshInput::new(topo, &model, &WholeFaces, faces);
    let mut intent = MeshIntent::default();
    for &(e, n) in counts {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    }
    for &f in faces {
        intent.set_scheme(f, SchemeChoice::with_params("trimesh", params.clone(), Firmness::Hard));
    }
    let plan = driver.plan(&input, &intent)?;
    driver.generate(&input, &plan, &sm_scheme_api::Sequential)
}
