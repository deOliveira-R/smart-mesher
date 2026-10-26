//! Stage-0 S4 contracts of sm-scheme-api: `GenCtx::loop_uvs` and `GenerateFailure::with_nodes`.

use std::sync::{Arc, Mutex};

use sm_core::{EntityId, Point3};
use sm_geom_api::{Interval, Line2, PCurve, Uv};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_scheme_api::geom;
use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    Applicability, Caps, Driver, ElemShape, FailureKind, Firm, Firmness, GenCtx, GenerateFailure,
    IntervalRequest, MeshInput, MeshIntent, NodeKey, Params, Patch, PatchBuilder, PlanCtx,
    ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Scheme, SchemeChoice, SchemeDim,
    Sequential, Target, WholeFaces,
};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::{Carrier, OpId, TopologyEdit, TopologyView, TopologyViewExt};

type Probe = Arc<Mutex<Vec<Option<Vec<([f64; 2], Point3)>>>>>;

/// A polygon scheme that records `loop_uvs` of its face's outer loop with the node positions.
struct UvProbe(Probe);

impl Scheme for UvProbe {
    fn name(&self) -> &str {
        "uvprobe"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::POLY
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(1, "test probe")
    }
    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        _t: Target,
        _p: &Params,
        _out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        Ok(())
    }
    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        _p: &Params,
    ) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "uvprobe", r);
        let Target::Surface(f) = target else {
            return Err(fatal("not a face".into()));
        };
        let loops = cx.loops(f).map_err(|x| fatal(x.to_string()))?;
        let lp = &loops[0];
        let rec = cx.loop_uvs(f, lp).map(|uv| {
            uv.into_iter()
                .zip(&lp.nodes)
                .map(|(uv, k)| {
                    let p = cx
                        .point(*k)
                        .or_else(|| geom::vertex_point(cx.input.topo, k.owner).ok())
                        .expect("node position");
                    (uv, p)
                })
                .collect()
        });
        self.0.lock().expect("lock").push(rec);
        let mut pb = PatchBuilder::new(target);
        let poly: Vec<u32> = lp.nodes.iter().map(|k| pb.refer(*k)).collect();
        // A seam repeats nodes in the loop, which no polygon may do: record only.
        let mut sorted = poly.clone();
        sorted.sort_unstable();
        sorted.dedup();
        if sorted.len() == poly.len() {
            pb.push(ElemShape::Polygon, &poly);
        }
        Ok(pb.finish())
    }
}

fn run_probe(
    topo: &sm_topo_api::CellComplex,
    ids: &sm_topo_api::fixtures::BrickIds,
) -> Option<Vec<([f64; 2], Point3)>> {
    run_probe_on(topo, ids.faces[4], &ids.edges)
}

fn run_probe_on(
    topo: &sm_topo_api::CellComplex,
    face: EntityId,
    edges: &[EntityId],
) -> Option<Vec<([f64; 2], Point3)>> {
    let probe: Probe = Arc::default();
    let mut reg = registry();
    reg.register(Box::new(UvProbe(probe.clone())))
        .expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = sm_model_api::Model::new();
    let sel = [face];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for e in edges {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(3)));
    }
    intent.set_scheme(face, SchemeChoice::new("uvprobe", Firmness::Hard));
    driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::surface(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let got = probe.lock().expect("lock");
    assert_eq!(got.len(), 1);
    got[0].clone()
}

fn surface_of(topo: &dyn TopologyView, face: EntityId) -> &dyn sm_geom_api::ParametricSurface {
    let Carrier::Surface(sc) = topo.carrier(face) else {
        panic!("planar face has a surface carrier");
    };
    topo.shapes().surface(sc.surface).expect("surface")
}

/// Set an exact Line2 pcurve (shifted by `shift`) on every coedge of `face` except those in
/// `skip_shift`, which get `shift2` instead.
fn set_pcurves(
    topo: &mut sm_topo_api::CellComplex,
    face: EntityId,
    op: u64,
    shift: [f64; 2],
    odd_one: Option<[f64; 2]>,
) {
    let keys: Vec<_> = topo
        .loops(face)
        .flat_map(|l| l.keys().collect::<Vec<_>>())
        .collect();
    let mut pcs = Vec::new();
    for (i, k) in keys.iter().enumerate() {
        let (c, cc) = geom::edge_curve(topo, k.edge).expect("curve");
        let s = surface_of(topo, face);
        let (lo, hi) = (cc.range.lo, cc.range.hi);
        let a = s.project(c.eval(lo), None).expect("proj").uv;
        let b = s.project(c.eval(hi), None).expect("proj").uv;
        let sh = if i == 0 {
            odd_one.unwrap_or(shift)
        } else {
            shift
        };
        pcs.push((
            *k,
            PCurve::Line2(Line2 {
                origin: Uv::new(a.u + sh[0], a.v + sh[1]),
                dir: Uv::new((b.u - a.u) / (hi - lo), (b.v - a.v) / (hi - lo)),
                range: Interval::new(lo, hi),
            }),
        ));
    }
    with_op(topo, OpId(op), |t| {
        for (k, p) in pcs {
            t.set_pcurve(k, Some(p))?;
        }
        Ok(())
    })
    .expect("set pcurves");
}

#[test]
fn loop_uvs_projects_on_a_planar_face_and_prefers_consistent_pcurves() {
    let (mut topo, ids, _) = brick_setup([1.0, 2.0, 3.0]);
    let face = ids.faces[4];
    // No pcurves: projection fallback equals the surface projection of each node.
    let got = run_probe(&topo, &ids).expect("projection fallback on a plane");
    assert_eq!(got.len(), 4 * 3, "one uv per node of the loop");
    {
        let s = surface_of(&topo, face);
        for (uv, p) in &got {
            let w = s.project(*p, None).expect("project").uv;
            assert!((uv[0] - w.u).abs() < 1e-12 && (uv[1] - w.v).abs() < 1e-12);
        }
    }
    // Pcurves win: a consistent shift of (5, 7) shows up exactly.
    set_pcurves(&mut topo, face, 2, [5.0, 7.0], None);
    let got2 = run_probe(&topo, &ids).expect("consistent pcurves");
    assert_eq!(got2.len(), got.len());
    for ((a, _), (b, _)) in got.iter().zip(&got2) {
        assert!((b[0] - a[0] - 5.0).abs() < 1e-9 && (b[1] - a[1] - 7.0).abs() < 1e-9);
    }
    // One coedge on another branch: the pcurves disagree at a vertex, so no answer.
    set_pcurves(&mut topo, face, 3, [5.0, 7.0], Some([5.0 + 1.0, 7.0]));
    assert!(run_probe(&topo, &ids).is_none());
}

#[test]
fn generate_failure_nodes_roundtrip_and_old_attempts_still_load() {
    let t = Target::Surface(EntityId::new(sm_core::EntityKind::TopoFace, 3).expect("id"));
    let v = EntityId::new(sm_core::EntityKind::TopoVertex, 1).expect("id");
    let f = GenerateFailure::fallback(t, "pave", "inverted element");
    assert!(f.nodes.is_empty());
    let js = serde_json::to_value(&f).expect("ser");
    assert!(
        js.get("nodes").is_none(),
        "an empty list is not serialized: {js}"
    );
    let g = f
        .clone()
        .with_nodes(vec![NodeKey::vertex(v), NodeKey::new(v, 2)]);
    assert_eq!(g.nodes.len(), 2);
    assert_eq!(g.kind, FailureKind::Quality);
    let back: GenerateFailure =
        serde_json::from_value(serde_json::to_value(&g).expect("ser")).expect("de");
    assert_eq!(back, g);
    // A failure persisted before the field existed deserializes with no nodes.
    let back_old: GenerateFailure = serde_json::from_value(js).expect("old form");
    assert_eq!(back_old, f);
}

/// Signed shoelace area of a uv polygon.
fn uv_area(pts: &[([f64; 2], Point3)]) -> f64 {
    pts.iter()
        .zip(pts.iter().cycle().skip(1))
        .map(|((a, _), (b, _))| a[0] * b[1] - b[0] * a[1])
        .sum::<f64>()
        / 2.0
}

#[test]
fn loop_uvs_on_a_cylinder_follows_the_seam_branches() {
    let (r, h) = (1.5, 2.0);
    let mut topo = sm_topo_api::CellComplex::new();
    let c = with_op(&mut topo, OpId(1), |t| {
        sm_topo_api::fixtures::cylinder(t, r, h)
    })
    .expect("cylinder");
    let got = run_probe_on(&topo, c.faces[2], &c.edges).expect("pcurves on the seam");
    // 3 segments on each of bottom, seam, top, seam.
    assert_eq!(got.len(), 12);
    for (uv, p) in &got {
        assert!((r * uv[0].cos() - p.x).abs() < 1e-9, "x {uv:?} {p:?}");
        assert!((r * uv[0].sin() - p.y).abs() < 1e-9, "y {uv:?} {p:?}");
        assert!((uv[1] - p.z).abs() < 1e-9, "z {uv:?} {p:?}");
    }
    // The seam is read as u = 2 pi on the forward side and u = 0 on the backward side.
    let tau = std::f64::consts::TAU;
    let at_tau = got
        .iter()
        .filter(|(uv, _)| (uv[0] - tau).abs() < 1e-12)
        .count();
    let at_zero = got.iter().filter(|(uv, _)| uv[0].abs() < 1e-12).count();
    assert!(
        at_tau >= 3 && at_zero >= 3,
        "seam branches: {at_tau} at 2pi, {at_zero} at 0"
    );
    let a = uv_area(&got);
    assert!(
        (a.abs() - tau * h).abs() < 1e-9,
        "uv polygon area {a}, want {}",
        tau * h
    );
}

#[test]
fn loop_uvs_on_a_sphere_puts_poles_on_the_outgoing_coedge() {
    use std::f64::consts::FRAC_PI_2;
    let r = 2.0;
    let mut topo = sm_topo_api::CellComplex::new();
    let s = with_op(&mut topo, OpId(1), |t| sm_topo_api::fixtures::sphere(t, r)).expect("sphere");
    let got = run_probe_on(&topo, s.face, &s.edges).expect("pcurves with poles");
    for (uv, p) in &got {
        let (cv, sv) = (uv[1].cos(), uv[1].sin());
        assert!((r * cv * uv[0].cos() - p.x).abs() < 1e-9, "x {uv:?} {p:?}");
        assert!((r * cv * uv[0].sin() - p.y).abs() < 1e-9, "y {uv:?} {p:?}");
        assert!((r * sv - p.z).abs() < 1e-9, "z {uv:?} {p:?}");
    }
    // Poles: the outgoing coedge of the south pole is the seam at u = 2 pi, of the north pole
    // the backward seam at u = 0.
    let south = got
        .iter()
        .find(|(_, p)| (p.z + r).abs() < 1e-9)
        .expect("south pole node");
    let north = got
        .iter()
        .find(|(_, p)| (p.z - r).abs() < 1e-9)
        .expect("north pole node");
    assert!(
        (south.0[1] + FRAC_PI_2).abs() < 1e-12
            && (south.0[0] - std::f64::consts::TAU).abs() < 1e-12,
        "south {:?}",
        south.0
    );
    assert!(
        (north.0[1] - FRAC_PI_2).abs() < 1e-12 && north.0[0].abs() < 1e-12,
        "north {:?}",
        north.0
    );
}
