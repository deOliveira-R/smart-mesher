//! ACR-V2a: composite faces through `GenCtx::loop_uvs` / `geom::face_normal`, and the planner
//! mapping `PartitionFace` intent to `Target::Surface`.

use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_geom_api::{Circle, CurveDef, Cylinder, Frame, Interval, Line, Plane, SurfaceDef};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    BoundaryLoop, Driver, Firm, Firmness, GenCtx, IntervalRequest, MeshInput, MeshIntent, PatchSet,
    ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, SchemeError, Sequential, Target,
    WholeFaces, geom,
};
use sm_topo_api::fixtures::{BrickIds, with_op};
use sm_topo_api::{
    Carrier, CellComplex, CoedgeSpec, CurveCarrier, FaceCut, InnerLoopAssign, OpId, Overlay, Sense,
    SurfaceCarrier, TopologyEdit, TopologyView,
};
use std::sync::Arc;

fn diagonal() -> CurveDef {
    CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())
}

fn all_counts(ids: &BrickIds, n: u32) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for e in ids.edges {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(n)));
    }
    intent
}

/// Loop uvs of face `f` of `topo` (a surface-only selection of `[f]`) and its normal at `p`.
fn uvs_and_normal(
    topo: &dyn TopologyView,
    ids: &BrickIds,
    f: EntityId,
    p: Point3,
) -> (Vec<[f64; 2]>, Vector3) {
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = sm_model_api::Model::new();
    let sel = [f];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let out = driver
        .mesh(
            &input,
            &all_counts(ids, 3),
            &Sequential,
            ReferenceBuilder::surface(),
            &mut SerialAlloc::new(),
        )
        .expect("surface mesh");
    // One face's mesh: 3 x 3 quads.
    assert_eq!(out.finished.mesh.cell_count(), 9);
    let cx = GenCtx::new(&input, &out.plan, &out.patches, None);
    let loops = cx.loops(f).expect("loops");
    assert_eq!(loops.len(), 1);
    let uvs = cx.loop_uvs(f, &loops[0]).expect("loop uvs");
    (uvs, geom::face_normal(topo, f, p).expect("normal"))
}

#[test]
fn planar_composite_meshes_like_the_host_face() {
    // Reference: the unsplit top face of a unit brick.
    let (host, ids, _) = brick_setup([1.0; 3]);
    let top = ids.faces[5];
    let p = Point3::new(0.3, 0.6, 1.0);
    let (want_uv, want_n) = uvs_and_normal(&host, &ids, top, p);
    assert_eq!(want_uv.len(), 12);

    // The same brick with its top face split in two, then composited back.
    let (mut t, ids2, _) = brick_setup([1.0; 3]);
    let halves = with_op(&mut t, OpId(2), |t| {
        let curve = t.shapes_mut().unwrap().add_curve(diagonal()).unwrap();
        let chord = t.make_edge(
            ids2.vertices[4],
            ids2.vertices[7],
            Some(CurveCarrier {
                curve,
                range: Interval::new(0.0, 2f64.sqrt()),
                sense: Sense::Forward,
            }),
        )?;
        Ok(t.split_face(
            ids2.faces[5],
            FaceCut::Chord { edge: chord },
            InnerLoopAssign::None,
        )?)
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&halves).unwrap();
    assert!(matches!(ov.carrier(vf), Carrier::Composite(_)));
    let (got_uv, got_n) = uvs_and_normal(&ov, &ids2, vf, p);

    assert_eq!(got_uv.len(), want_uv.len());
    for (g, w) in got_uv.iter().zip(&want_uv) {
        assert!(
            (g[0] - w[0]).abs() < 1e-12 && (g[1] - w[1]).abs() < 1e-12,
            "{g:?} vs {w:?}"
        );
    }
    assert!((got_n - want_n).norm() < 1e-12, "{got_n:?} vs {want_n:?}");
    assert!((got_n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
}

/// Two quads sharing an edge, each on its own surface (topology only), as a composite.
/// `bend` lifts the outline into a V so it is not planar.
fn two_quads(surfs: [SurfaceDef; 2], bend: bool) -> (Overlay, EntityId) {
    let p = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (0.0, 1.0),
    ];
    let z = |q: (f64, f64)| {
        if bend {
            0.5 * (q.0 - 1.0) * (q.0 - 1.0)
        } else {
            0.0
        }
    };
    composite_of(p.map(|q| (q.0, q.1, z(q))), surfs, false)
}

/// Two faces over six corner points (`0 1 2 / 5 4 3` rows, shared edge `1-4`), as a composite.
/// Edges are straight lines, except with `arcs` an edge between two points of equal height on
/// the unit cylinder is its circular arc (so every member is consistent with a cylinder carrier).
fn composite_of(
    p: [(f64, f64, f64); 6],
    surfs: [SurfaceDef; 2],
    arcs: bool,
) -> (Overlay, EntityId) {
    let mut t = CellComplex::new();
    let faces = with_op(&mut t, OpId(1), |t| {
        let v: Vec<EntityId> = p
            .iter()
            .map(|q| t.make_vertex(Point3::new(q.0, q.1, q.2)).unwrap())
            .collect();
        let edge = |t: &mut CellComplex, a: usize, b: usize| {
            let (pa, pb) = (
                Point3::new(p[a].0, p[a].1, p[a].2),
                Point3::new(p[b].0, p[b].1, p[b].2),
            );
            let arc = arcs && pa.z == pb.z && (pa.x, pa.y) != (pb.x, pb.y);
            let (def, range, sense) = if arc {
                let c = Circle::new(Frame::at(Point3::new(0.0, 0.0, pa.z)), 1.0).unwrap();
                let sense = if pa.x > pb.x {
                    Sense::Forward
                } else {
                    Sense::Reversed
                };
                (
                    CurveDef::Circle(c),
                    Interval::new(0.0, std::f64::consts::FRAC_PI_2),
                    sense,
                )
            } else {
                let line = Line::new(pa, pb - pa).unwrap();
                (
                    CurveDef::Line(line),
                    Interval::new(0.0, 1.0),
                    Sense::Forward,
                )
            };
            let curve = t.shapes_mut().unwrap().add_curve(def).unwrap();
            t.make_edge(
                v[a],
                v[b],
                Some(CurveCarrier {
                    curve,
                    range,
                    sense,
                }),
            )
        };
        let e = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0), (1, 4)]
            .map(|(a, b)| edge(t, a, b).unwrap());
        let mut sc = [None; 2];
        for (slot, def) in sc.iter_mut().zip(surfs) {
            let surface = t.shapes_mut().unwrap().add_surface(def).unwrap();
            *slot = Some(SurfaceCarrier {
                surface,
                sense: Sense::Forward,
            });
        }
        let f = |i: usize, fwd: bool| CoedgeSpec {
            edge: e[i],
            sense: if fwd { Sense::Forward } else { Sense::Reversed },
        };
        let a = t.make_face(&[&[f(0, true), f(6, true), f(4, true), f(5, true)]], sc[0])?;
        let b = t.make_face(&[&[f(1, true), f(2, true), f(3, true), f(6, false)]], sc[1])?;
        Ok([a, b])
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&faces).unwrap();
    (ov, vf)
}

/// A quarter cylinder (r = 1, angle 0..pi/2, z 0..2) on ONE cylinder handle, split at z = 1
/// into two faces and composited. Its six corners lie on the plane x + y = 1, so an outline-only
/// planarity test would wrongly accept it.
fn cylinder_composite() -> (Overlay, EntityId) {
    let cyl =
        SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
    let p = [
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 1.0),
        (0.0, 1.0, 2.0),
        (1.0, 0.0, 2.0),
        (1.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
    ];
    let (ov, vf) = composite_of(p, [cyl.clone(), cyl], true);
    // The corners really are coplanar (and on the cylinder).
    for q in p {
        assert!((q.0 + q.1 - 1.0).abs() < 1e-15 && (q.0 * q.0 + q.1 * q.1 - 1.0).abs() < 1e-15);
    }
    (ov, vf)
}

fn plane(z: f64) -> SurfaceDef {
    SurfaceDef::Plane(Plane {
        frame: Frame::at(Point3::new(0.0, 0.0, z)),
    })
}

#[test]
fn coplanar_members_on_different_plane_handles_keep_the_newell_normal() {
    let (ov, vf) = two_quads([plane(0.0), plane(0.0)], false);
    // Two handles: no single surface, but one plane.
    assert_eq!(ov.composite_surface(vf), None);
    let n = geom::face_normal(&ov, vf, Point3::new(1.0, 0.5, 0.0)).expect("planar fallback");
    assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12, "{n:?}");
}

#[test]
fn non_planar_mixed_composite_is_refused() {
    // Members on parallel but offset planes, outline flat: not one plane. V2b would not help.
    for bend in [true, false] {
        let (ov, vf) = two_quads([plane(0.0), plane(0.5)], bend);
        assert_eq!(ov.composite_surface(vf), None);
        match geom::face_normal(&ov, vf, Point3::new(1.0, 0.5, 0.0)) {
            Err(SchemeError::Input(m)) => {
                assert!(m.contains("not planar") && !m.contains("V2b"), "{m}")
            }
            other => panic!("expected a typed refusal, got {other:?}"),
        }
    }
}

#[test]
fn periodic_composite_is_refused_not_guessed() {
    let (ov, vf) = cylinder_composite();
    assert_eq!(ov.composite_surface(vf), None);
    // face_normal: typed refusal (curved members, whatever the corners say), not the Newell chord-plane normal (-0.7, -0.7, 0).
    match geom::face_normal(&ov, vf, Point3::new(1.0, 0.0, 0.5)) {
        Err(SchemeError::Input(m)) => {
            assert!(m.contains("not planar") && !m.contains("V2b"), "{m}")
        }
        other => panic!("expected a typed refusal, got {other:?}"),
    }
    // loop_uvs: None (no uv at all), through a GenCtx over the overlay.
    let (host, ids, model) = brick_setup([1.0; 3]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[5]];
    let input = MeshInput::new(&host, &model, &WholeFaces, &sel);
    let plan = driver.plan(&input, &all_counts(&ids, 2)).unwrap();
    let ov_input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let patches = PatchSet::default();
    let cx = GenCtx::new(&ov_input, &plan, &patches, None);
    let lp = BoundaryLoop {
        ring: 0,
        nodes: Vec::new(),
        uses: Vec::new(),
    };
    assert_eq!(cx.loop_uvs(vf, &lp), None);
}

/// Plan a surface-only selection of the bottom and top faces of a brick, each a composite of
/// two host halves (`PartitionFace` ids; the bottom composite has the lower id), tied by an
/// explicit `copy` intent on the bottom composite. A face with the copy intent is never the
/// class root; without the intent the lowest target is. That is only seen if the planner treats
/// the intent on a composite as one on a `Surface` target.
fn copy_class_root(with_intent: bool) -> (Target, EntityId, EntityId) {
    let (mut t, ids, model) = brick_setup([1.0; 3]);
    let split = |t: &mut CellComplex, op: u64, f: usize, a: usize, b: usize, z: f64| {
        with_op(t, OpId(op), |t| {
            let line = Line::new(Point3::new(0.0, 0.0, z), Vector3::new(1.0, 1.0, 0.0)).unwrap();
            let curve = t
                .shapes_mut()
                .unwrap()
                .add_curve(CurveDef::Line(line))
                .unwrap();
            let chord = t.make_edge(
                ids.vertices[a],
                ids.vertices[b],
                Some(CurveCarrier {
                    curve,
                    range: Interval::new(0.0, 2f64.sqrt()),
                    sense: Sense::Forward,
                }),
            )?;
            Ok(t.split_face(
                ids.faces[f],
                FaceCut::Chord { edge: chord },
                InnerLoopAssign::None,
            )?)
        })
        .unwrap()
    };
    let top = split(&mut t, 2, 5, 4, 7, 1.0);
    let bottom = split(&mut t, 3, 4, 0, 3, 0.0);
    let mut ov = Overlay::new(Arc::new(t));
    let vb = ov.composite_faces(&bottom).unwrap();
    let vt = ov.composite_faces(&top).unwrap();
    assert!(vb < vt);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [vb, vt];
    let input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let mut intent = all_counts(&ids, 2);
    if with_intent {
        intent.set_scheme(
            vb,
            SchemeChoice::with_params(
                "copy",
                serde_json::json!({
                    "source": vt,
                    "transform": Transform::translation(Vector3::new(0.0, 0.0, -1.0)),
                }),
                Firmness::Hard,
            ),
        );
    } else {
        intent.set_scheme(
            vt,
            SchemeChoice::with_params(
                "copy",
                serde_json::json!({
                    "source": vb,
                    "transform": Transform::translation(Vector3::new(0.0, 0.0, 1.0)),
                }),
                Firmness::Hard,
            ),
        );
    }
    let plan = driver.plan(&input, &intent).expect("plan");
    let (class, _) = plan
        .copy_of(Target::Surface(vb))
        .or_else(|| plan.copy_of(Target::Surface(vt)))
        .expect("class");
    (class.root, vb, vt)
}

#[test]
fn copy_intent_on_a_composite_face_keeps_it_out_of_the_root() {
    // Control: the copy intent is on the higher id, so the lower one stays the root.
    let (root, vb, _) = copy_class_root(false);
    assert_eq!(root, Target::Surface(vb));
    // The intent on the lower id moves the root to the other composite.
    let (root, _, vt) = copy_class_root(true);
    assert_eq!(root, Target::Surface(vt));
}

/// Brick whose top face is split by the diagonal and composited back; returns the overlay and
/// the composite face.
fn brick_with_composite_top() -> (Overlay, BrickIds, EntityId, sm_model_api::Model) {
    let (mut t, ids, model) = brick_setup([1.0; 3]);
    let halves = with_op(&mut t, OpId(2), |t| {
        let curve = t.shapes_mut().unwrap().add_curve(diagonal()).unwrap();
        let chord = t.make_edge(
            ids.vertices[4],
            ids.vertices[7],
            Some(CurveCarrier {
                curve,
                range: Interval::new(0.0, 2f64.sqrt()),
                sense: Sense::Forward,
            }),
        )?;
        Ok(t.split_face(
            ids.faces[5],
            FaceCut::Chord { edge: chord },
            InnerLoopAssign::None,
        )?)
    })
    .unwrap();
    let mut ov = Overlay::new(Arc::new(t));
    let vf = ov.composite_faces(&halves).unwrap();
    (ov, ids, vf, model)
}

#[test]
fn volume_with_a_composite_face_plans_it_as_a_surface_target() {
    // T2 acceptance 5: the volume's face list reaches `region.faces`, so the composite is a
    // `Target::Surface` and its hidden halves are not targets.
    let (ov, ids, vf, model) = brick_with_composite_top();
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let plan = driver.plan(&input, &all_counts(&ids, 2)).expect("plan");
    assert!(
        plan.target(Target::Surface(vf)).is_some(),
        "composite not planned"
    );
    assert!(plan.layout(vf).is_some());
    for h in ov.live_cover(vf).into_iter().filter(|h| *h != vf) {
        assert!(
            plan.target(Target::Surface(h)).is_none(),
            "hidden member {h} planned"
        );
    }
}

fn partition_copy_root(with_intent: bool) -> (Target, EntityId, EntityId) {
    // Opposite x-edges of the top face, each split in two; the copy intent sits on halves.
    let (host, ids, model) = brick_setup([1.0; 3]);
    let mut ov = Overlay::new(Arc::new(host));
    let (_, a) = ov.partition_edge(ids.edges[2], 0.5).unwrap();
    let (_, b) = ov.partition_edge(ids.edges[3], 0.5).unwrap();
    assert!(a[0] < b[0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[5]];
    let input = MeshInput::new(&ov, &model, &WholeFaces, &sel);
    let mut intent = all_counts(&ids, 2);
    for h in a.iter().chain(&b) {
        intent.set_intervals(*h, Firm::hard(IntervalRequest::Count(2)));
    }
    let (dst, src, dy) = if with_intent {
        (a[0], b[0], -1.0)
    } else {
        (b[0], a[0], 1.0)
    };
    intent.set_scheme(
        dst,
        SchemeChoice::with_params(
            "copy",
            serde_json::json!({
                "source": src,
                "transform": Transform::translation(Vector3::new(0.0, dy, 0.0)),
            }),
            Firmness::Hard,
        ),
    );
    let plan = driver.plan(&input, &intent).expect("plan");
    let (class, _) = plan
        .copy_of(Target::Curve(a[0]))
        .or_else(|| plan.copy_of(Target::Curve(b[0])))
        .expect("class");
    (class.root, a[0], b[0])
}

#[test]
fn copy_intent_on_a_partition_edge_is_honoured() {
    // Control: the intent is on the higher id, so the lower one stays the root.
    let (root, a, _) = partition_copy_root(false);
    assert_eq!(root, Target::Curve(a));
    // The intent on the lower id moves the root to the other half.
    let (root, _, b) = partition_copy_root(true);
    assert_eq!(root, Target::Curve(b));
}
