//! W4-VT-TIDY: PR #11 carry-forwards. Virtual cells in `model.measure` / `topo.related`, the
//! delete of a body with a realized split, the cost of transforming many overlay ops, the
//! collapse named by a tolerance refusal, and a transaction failure inside a transform with a
//! touched overlay.

mod common;

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Instant;

use common::{Factory, cell_complex, id, meta, topo_store};
use serde_json::json;
use sm_boolean::BooleanPlugin;
use sm_boolean::ops::{AxisPlane, PlaneAxis, PlaneSpec, Unite, WebcutPlane};
use sm_core::{EntityId, EntityKind, Point3};
use sm_layer_api::{CoreLayers, ModelLayer};
use sm_model::{CollapseEdge, CompositeFaces, CreateBrick, CreateCylinder, Delete, Measurement, ModelPlugin, Move, PartitionEdge, PartitionFace, SplitRealized};
use sm_op_api::{DerivedCache, Engine, Op, QuietControl, Registry, Services, Session};
use sm_topo_api::{Carrier, Dim, TopologyView};

fn session_for(factory: Factory) -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).unwrap();
    reg.install(&ModelPlugin).unwrap();
    reg.install(&BooleanPlugin).unwrap();
    Session::new(Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default())).unwrap())
}

fn go(s: &mut Session, op: &dyn Op) -> sm_op_api::Delta {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control()).map(|(_, d)| d).unwrap_or_else(|e| panic!("{e}"))
}

fn try_go(s: &mut Session, op: &dyn Op) -> Result<sm_op_api::Delta, sm_op_api::TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control()).map(|(_, d)| d)
}

fn layer(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().unwrap()
}

fn list(s: &Session, kind: &str) -> Vec<EntityId> {
    serde_json::from_value(common::query(s, "model.list", json!({ "kind": kind })).unwrap()).unwrap()
}

fn measure(s: &Session, e: EntityId) -> Result<Measurement, sm_op_api::RegistryError> {
    common::query(s, "model.measure", json!({ "entity": e })).map(|v| serde_json::from_value(v).unwrap())
}

fn related(s: &Session, kind: &str, within: &[EntityId]) -> Result<Vec<EntityId>, sm_op_api::RegistryError> {
    common::query(s, "topo.related", json!({ "kind": kind, "within": within })).map(|v| serde_json::from_value(v).unwrap())
}

fn vertex_point(t: &dyn TopologyView, v: EntityId) -> Point3 {
    match t.carrier(v) {
        Carrier::Point(p) => *p,
        c => panic!("vertex carrier {c:?}"),
    }
}

fn vertex_at(s: &Session, p: [f64; 3]) -> EntityId {
    let t = layer(s).topo();
    *t.cells(Dim::Vertex).iter().find(|&&v| (vertex_point(t, v) - Point3::new(p[0], p[1], p[2])).norm() < 1e-9).expect("vertex")
}

fn verts_of(t: &dyn TopologyView, c: EntityId, out: &mut BTreeSet<EntityId>) {
    if t.dim_of(c) == Some(Dim::Vertex) {
        out.insert(c);
        return;
    }
    for i in t.boundary(c) {
        verts_of(t, i.cell, out);
    }
}

fn face_with_vertices(s: &Session, vs: &[EntityId]) -> EntityId {
    let t = layer(s).topo();
    *t.cells(Dim::Face)
        .iter()
        .find(|&&f| {
            let mut found = BTreeSet::new();
            verts_of(t, f, &mut found);
            vs.iter().all(|v| found.contains(v))
        })
        .expect("face")
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// 2x2x2 brick on [0,2]^3.
fn brick2(factory: Factory) -> Session {
    let mut s = session_for(factory);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    s
}

fn top_corners(s: &Session) -> [EntityId; 4] {
    [vertex_at(s, [0., 0., 2.]), vertex_at(s, [2., 2., 2.]), vertex_at(s, [2., 0., 2.]), vertex_at(s, [0., 2., 2.])]
}

// --------------------------------------------------------------------- (1) virtual measure

#[test]
fn measure_of_a_cylinder_composite_is_two_pi_r_h() {
    for factory in [cell_complex as Factory, topo_store] {
        let (r, h) = (1.5, 4.0);
        let mut s = session_for(factory);
        go(&mut s, &CreateCylinder { height: h, radius: r, center: None });
        go(&mut s, &WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 0.0 }), keep: None, merge: None, imprint: None });
        let bodies = list(&s, "TopoBody");
        go(&mut s, &Unite { bodies, keep: None, regularize: Some(false), tol: None });
        let lat: Vec<EntityId> = {
            let t = layer(&s).topo();
            t.cells(Dim::Face)
                .iter()
                .copied()
                .filter(|&f| matches!(t.carrier(f), Carrier::Surface(sc) if matches!(t.shapes().surface_def(sc.surface), Some(sm_geom_api::SurfaceDef::Cylinder(_)))))
                .collect()
        };
        assert!(lat.len() >= 2);
        let comp = go(&mut s, &CompositeFaces { faces: lat.clone() }).outputs["face"][0];
        let m = measure(&s, comp).unwrap();
        let exact = 2.0 * std::f64::consts::PI * r * h;
        assert!(close(m.area.unwrap(), exact, 1e-9 * exact), "{:?} vs {exact}", m.area);
        // The bounding box covers the full circle in x and y.
        assert!(close(m.bbox.hi[0] - m.bbox.lo[0], 2.0 * r, 1e-9) && close(m.bbox.hi[1] - m.bbox.lo[1], 2.0 * r, 1e-9), "{:?}", m.bbox);
        // Members are hidden from the view: the typed refusal is the unknown-entity one.
        assert!(measure(&s, lat[0]).is_err());
        // The face is related to its boundary edges (virtual traversal) and the volume.
        let vols = related(&s, "TopoVolume", &[comp]).unwrap();
        assert_eq!(vols.len(), 1);
        let edges = related(&s, "TopoEdge", &[comp]).unwrap();
        assert!(!edges.is_empty());
    }
}

#[test]
fn measure_of_the_halves_of_a_partitioned_square() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = brick2(factory);
        let [a, b, c, e] = top_corners(&s);
        let top = face_with_vertices(&s, &[a, b, c, e]);
        let d = go(&mut s, &PartitionFace { face: top, start: a, end: b });
        let kids = d.outputs["faces"].clone();
        let chord = d.outputs["chord"][0];
        let ms: Vec<Measurement> = kids.iter().map(|&k| measure(&s, k).unwrap()).collect();
        // Each half is a right triangle of area 2; together the whole 2x2 square.
        assert!(ms.iter().all(|m| close(m.area.unwrap(), 2.0, 1e-12)), "{ms:?}");
        assert!(close(ms.iter().map(|m| m.area.unwrap()).sum::<f64>(), 4.0, 1e-12));
        // Centroids: (4/3, 2/3, 2) and (2/3, 4/3, 2) in some order; their mean is the square's.
        let cs: Vec<[f64; 3]> = ms.iter().map(|m| m.centroid.unwrap()).collect();
        let mid = [(cs[0][0] + cs[1][0]) / 2.0, (cs[0][1] + cs[1][1]) / 2.0, (cs[0][2] + cs[1][2]) / 2.0];
        assert!(close(mid[0], 1.0, 1e-9) && close(mid[1], 1.0, 1e-9) && close(mid[2], 2.0, 1e-9), "{cs:?}");
        let mut xs: Vec<f64> = cs.iter().map(|c| c[0]).collect();
        xs.sort_by(f64::total_cmp);
        assert!(close(xs[0], 2.0 / 3.0, 1e-9) && close(xs[1], 4.0 / 3.0, 1e-9), "{xs:?}");
        // The chord: length 2 sqrt 2, bbox the diagonal; its end vertices are virtual-free hosts.
        let mc = measure(&s, chord).unwrap();
        assert!(close(mc.length.unwrap(), 8f64.sqrt(), 1e-12), "{:?}", mc.length);
        assert!(close(mc.bbox.hi[0] - mc.bbox.lo[0], 2.0, 1e-12));
        // Related: a half is bounded by the chord and two host edges, and lies in the volume.
        assert_eq!(related(&s, "PartitionEdge", &[kids[0]]).unwrap(), vec![chord]);
        assert_eq!(related(&s, "TopoEdge", &[kids[0]]).unwrap().len(), 2);
        assert_eq!(related(&s, "TopoVolume", &[kids[0]]).unwrap(), vec![id(EntityKind::TopoVolume, 1)]);
        let mut both = related(&s, "PartitionFace", &[chord]).unwrap();
        both.sort();
        let mut want = kids.clone();
        want.sort();
        assert_eq!(both, want);
        // A real kind returns real cells only, a virtual kind virtual cells only.
        assert!(related(&s, "TopoFace", &[chord]).unwrap().is_empty());
        assert!(related(&s, "PartitionVertex", &[chord]).unwrap().is_empty());
    }
}

#[test]
fn measure_of_a_virtual_vertex_and_edge_halves() {
    // A partition vertex and its halves on a brick edge.
    let mut s = brick2(cell_complex);
    let [a, ..] = top_corners(&s);
    let t = layer(&s).topo();
    let edge = *t.cells(Dim::Edge).iter().find(|&&e| t.boundary(e).iter().any(|i| i.cell == a) && t.boundary(e).iter().any(|i| i.cell == vertex_at(&s, [2., 0., 2.]))).expect("edge");
    let Carrier::Curve(cc) = layer(&s).topo().carrier(edge).clone() else { panic!("curve") };
    let d = go(&mut s, &PartitionEdge { edge, param: cc.range.lo + 0.5 });
    let (v, halves) = (d.outputs["vertex"][0], d.outputs["edges"].clone());
    let mv = measure(&s, v).unwrap();
    assert!(close(mv.bbox.lo[0], 0.5, 1e-12) && close(mv.bbox.lo[1], 0.0, 1e-12) && close(mv.bbox.lo[2], 2.0, 1e-12), "{:?}", mv.bbox);
    let lens: Vec<f64> = halves.iter().map(|&h| measure(&s, h).unwrap().length.unwrap()).collect();
    assert!(close(lens[0] + lens[1], 2.0, 1e-12) && lens.iter().any(|l| close(*l, 0.5, 1e-12)), "{lens:?}");
}

#[test]
fn a_half_of_a_partitioned_composite_is_refused_with_its_id() {
    let mut s = brick2(cell_complex);
    go(&mut s, &WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 1.0 }), keep: None, merge: None, imprint: None });
    let bodies = list(&s, "TopoBody");
    go(&mut s, &Unite { bodies, keep: None, regularize: Some(false), tol: None });
    let v = |x, y| vertex_at(&s, [x, y, 2.0]);
    let left = face_with_vertices(&s, &[v(0., 0.), v(1., 0.), v(1., 2.), v(0., 2.)]);
    let right = face_with_vertices(&s, &[v(1., 0.), v(2., 0.), v(2., 2.), v(1., 2.)]);
    let (p0, p1) = (v(0., 0.), v(2., 2.));
    let comp = go(&mut s, &CompositeFaces { faces: vec![left, right] }).outputs["face"][0];
    // The composite itself measures: the whole 2x2 square.
    assert!(close(measure(&s, comp).unwrap().area.unwrap(), 4.0, 1e-12));
    let halves = go(&mut s, &PartitionFace { face: comp, start: p0, end: p1 }).outputs["faces"].clone();
    let err = measure(&s, halves[0]).unwrap_err().to_string();
    assert!(err.contains(&halves[0].to_string()) && err.contains("composite"), "{err}");
}

// --------------------------------------------------------------------- (2) delete

#[test]
fn deleting_a_body_with_a_realized_split_says_unsplit_realized_first() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session_for(factory);
        go(&mut s, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
        go(&mut s, &SplitRealized { piece: id(EntityKind::TopoVolume, 1), normal: [1.0, 0.0, 0.0], offset: 1.0 });
        let hash = s.state().hash();
        let e = try_go(&mut s, &Delete { bodies: vec![id(EntityKind::TopoBody, 1)] }).unwrap_err().to_string();
        assert!(e.contains("is decomposed: model.unsplit_realized first") && e.contains("TopoVolume"), "{e}");
        assert_eq!(s.state().hash(), hash);
    }
}

// --------------------------------------------------------------------- (4) tolerance

#[test]
fn the_tolerance_refusal_names_the_collapse_whose_scaled_gap_is_too_large() {
    // A 2 x 5e-5 x 1e-4 plate: a short y-edge (gap 5e-5) collapsed first, a short z-edge (gap
    // 1e-4) second, far from each other. Scaling by 15 gives gaps 7.5e-4 (< linear_max 1e-3) and
    // 1.5e-3 (>): only the second collapse is to blame.
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(5e-5), z: Some(1e-4), center: Some([1.0, 2.5e-5, 5e-5]) });
    let find = |s: &Session, len: f64, at_x: f64| {
        let t = layer(s).topo();
        *t.cells(Dim::Edge)
            .iter()
            .find(|&&e| close(t.edge_sampler(e).unwrap().length(), len, 1e-12) && t.boundary(e).iter().all(|i| close(vertex_point(t, i.cell).x, at_x, 1e-9)))
            .expect("edge")
    };
    let (small, large) = (find(&s, 5e-5, 0.0), find(&s, 1e-4, 2.0));
    let first = layer(&s).topo().boundary(small)[0].cell;
    go(&mut s, &CollapseEdge { edge: small, into_vertex: first });
    let into = layer(&s).topo().boundary(large)[0].cell;
    go(&mut s, &CollapseEdge { edge: large, into_vertex: into });
    let hash = s.state().hash();
    let e = try_go(&mut s, &sm_model::Scale { entities: vec![id(EntityKind::TopoBody, 1)], factor: 15.0, origin: None }).unwrap_err().to_string();
    assert!(e.contains(&large.to_string()) && !e.contains(&small.to_string()), "{e} (small {small}, large {large})");
    assert!(e.contains("linear_max") && e.contains("model.remove_virtual"), "{e}");
    assert_eq!(s.state().hash(), hash);
    // A factor that stays under the cap still follows.
    go(&mut s, &sm_model::Scale { entities: vec![id(EntityKind::TopoBody, 1)], factor: 5.0, origin: None });
}

// --------------------------------------------------------------------- (5) txn failure

#[test]
fn a_failing_transform_with_a_touched_overlay_leaves_state_and_overlay_alone() {
    // Two half cylinders from an unmerged webcut share the cylinder's surface and circle
    // handles without sharing a cell: moving one is refused inside the topology transaction.
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateCylinder { height: 2.0, radius: 1.0, center: None });
    go(&mut s, &WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 0.0 }), keep: None, merge: Some(false), imprint: None });
    let bodies = list(&s, "TopoBody");
    assert_eq!(bodies.len(), 2);
    // Touch the overlay of the first body: partition one of its edges.
    let edge = {
        let t = layer(&s).topo();
        let es = related(&s, "TopoEdge", &[bodies[0]]).unwrap();
        *es.iter().find(|&&e| matches!(t.carrier(e), Carrier::Curve(_)) && t.edge_sampler(e).unwrap().length() > 1.0).expect("long edge")
    };
    let Carrier::Curve(cc) = layer(&s).topo().carrier(edge).clone() else { panic!("curve") };
    go(&mut s, &PartitionEdge { edge, param: cc.range.lo + (cc.range.hi - cc.range.lo) / 3.0 });
    let hash = s.state().hash();
    let ops_before = layer(&s).overlay().unwrap().ops().to_vec();
    let e = try_go(&mut s, &Move { entities: vec![bodies[0]], delta: [5.0, 0.0, 0.0] }).unwrap_err().to_string();
    assert!(e.contains("shared"), "{e}");
    assert_eq!(s.state().hash(), hash, "{e}");
    let ov = layer(&s).overlay().expect("overlay still there");
    assert_eq!(ov.ops(), &ops_before[..]);
    assert!(!ov.is_stale_against(layer(&s).host()));
    // The session still works: moving the other way round is refused the same way, and an
    // unrelated edit goes through.
    go(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([20.0, 0.0, 0.0]) });
    assert_eq!(layer(&s).overlay().unwrap().ops(), &ops_before[..]);
}

// --------------------------------------------------------------------- (3) cost

/// A 2x2x2 brick whose top face is cut by `n` chords of a fan from `(0,2,2)` to `n` points on
/// the opposite edge: `2n` overlay ops (n edge partitions, n face partitions).
fn fan(n: usize) -> Session {
    let mut s = brick2(cell_complex);
    let [a, _b, c, e] = top_corners(&s);
    let t = layer(&s).topo();
    let mut edge = *t.cells(Dim::Edge).iter().find(|&&x| t.boundary(x).iter().any(|i| i.cell == a) && t.boundary(x).iter().any(|i| i.cell == c)).expect("edge");
    let step = 2.0 / (n as f64 + 1.0);
    let mut points = Vec::new();
    for _ in 0..n {
        let Carrier::Curve(cc) = layer(&s).topo().carrier(edge).clone() else { panic!("curve") };
        let d = go(&mut s, &PartitionEdge { edge, param: cc.range.lo + step });
        points.push(d.outputs["vertex"][0]);
        edge = d.outputs["edges"][1];
    }
    let mut face = {
        let t = layer(&s).topo();
        let all: Vec<EntityId> = t.cells(Dim::Face).iter().copied().filter(|&f| {
            let mut vs = BTreeSet::new();
            verts_of(t, f, &mut vs);
            vs.contains(&a) && vs.contains(&c) && vs.contains(&e)
        }).collect();
        all[0]
    };
    for i in 0..n {
        let d = go(&mut s, &PartitionFace { face, start: e, end: points[i] });
        let kids = d.outputs["faces"].clone();
        if i + 1 < n {
            let t = layer(&s).topo();
            face = *kids
                .iter()
                .find(|&&k| {
                    let mut vs = BTreeSet::new();
                    verts_of(t, k, &mut vs);
                    vs.contains(&points[i + 1])
                })
                .expect("remainder");
        }
    }
    s
}

#[test]
fn a_fan_of_cuts_follows_a_move_and_a_reflection() {
    let n = 10;
    let mut s = fan(n);
    let ops = layer(&s).overlay().unwrap().ops().len();
    assert_eq!(ops, 2 * n);
    let before: Vec<(EntityId, Point3)> = layer(&s).topo().cells(Dim::Vertex).iter().filter(|v| v.kind() == EntityKind::PartitionVertex).map(|&v| (v, vertex_point(layer(&s).topo(), v))).collect();
    assert_eq!(before.len(), n);
    go(&mut s, &Move { entities: vec![id(EntityKind::TopoBody, 1)], delta: [3.0, 0.0, 1.0] });
    for (v, p) in before {
        assert!((vertex_point(layer(&s).topo(), v) - Point3::new(p.x + 3.0, p.y, p.z + 1.0)).norm() < 1e-9);
    }
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
    // Areas of the 11 fan triangles still add up to the square.
    let faces = list(&s, "PartitionFace");
    let total: f64 = faces.iter().map(|&f| measure(&s, f).unwrap().area.unwrap()).sum();
    assert!(close(total, 4.0, 1e-9), "{total}");
    // A reflection reverses every loop: each id still names the mirror of what it named, and
    // the record replays cold to the same hash.
    let before: Vec<(f64, [f64; 3])> = faces.iter().map(|&f| measure(&s, f).map(|m| (m.area.unwrap(), m.centroid.unwrap())).unwrap()).collect();
    go(&mut s, &sm_model::Reflect { entities: vec![id(EntityKind::TopoBody, 1)], normal: [0.0, 1.0, 0.0], origin: Some([0.0, 0.0, 0.0]) });
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
    assert!(sm_topo_api::validate_geometry(layer(&s).topo()).is_empty());
    assert_eq!(list(&s, "PartitionFace"), faces);
    for (&f, (area, c)) in faces.iter().zip(&before) {
        let m = measure(&s, f).unwrap();
        let mc = m.centroid.unwrap();
        assert!(close(m.area.unwrap(), *area, 1e-9) && close(mc[0], c[0], 1e-9) && close(mc[1], -c[1], 1e-9) && close(mc[2], c[2], 1e-9), "{f}: {m:?} vs {area} {c:?}");
    }
    let full = s.state().hash();
    let q = QuietControl::new();
    let head = s.position().unwrap();
    assert_eq!(s.state_at_from_root(head, &q.control()).expect("cold replay").hash(), full);
}

/// Timing of a move over a fan of cuts (run with `--ignored --nocapture`): the cost must grow
/// about linearly in the number of cuts.
#[test]
#[ignore = "timing probe"]
fn timing_of_a_move_over_many_cuts() {
    for n in [10usize, 100, 300] {
        let mut s = fan(n);
        let t0 = Instant::now();
        go(&mut s, &Move { entities: vec![id(EntityKind::TopoBody, 1)], delta: [1.0, 0.0, 0.0] });
        eprintln!("cuts {n}: move {:?}", t0.elapsed());
    }
}
