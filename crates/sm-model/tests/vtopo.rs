//! W4 VTOPO-OPS: the virtual-topology ops on the layer's overlay (analytic checks, replay, fork,
//! undo, host-edit refusal, random sequences).

mod common;

use std::collections::BTreeSet;
use std::sync::Arc;

use common::{Factory, cell_complex, id, meta, topo_store};
use proptest::prelude::*;
use serde_json::json;
use sm_boolean::BooleanPlugin;
use sm_boolean::ops::{AxisPlane, PlaneAxis, PlaneSpec, Subtract, Unite, WebcutPlane};
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::SurfaceDef;
use sm_layer_api::{CoreLayers, ModelLayer};
use sm_model::{
    CollapseEdge, CompositeEdges, CompositeFaces, CreateBrick, CreateCylinder, Delete, ModelPlugin, PartitionEdge, PartitionFace,
    RemoveVirtual, SplitRealized, UnsplitRealized, VirtualInfoOut,
};
use sm_model_api::PieceRef;
use sm_op_api::{DerivedCache, Engine, Hash256, Op, OpRecord, QuietControl, Registry, Rev, Services, Session};
use sm_topo_api::{Carrier, Dim, TopologyView, TopologyViewExt};

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

fn info(s: &Session, id: Option<EntityId>) -> VirtualInfoOut {
    let v = common::query(s, "model.virtual_info", id.map_or(json!({}), |i| json!({ "id": i }))).unwrap();
    serde_json::from_value(v).unwrap()
}

fn list(s: &Session, kind: &str) -> Vec<EntityId> {
    serde_json::from_value(common::query(s, "model.list", json!({ "kind": kind })).unwrap()).unwrap()
}

/// Cylinder of radius `r`, height `h`, split by the plane x = 0 and united without
/// regularizing: one volume whose lateral face is in pieces.
fn halved_cylinder(s: &mut Session, r: f64, h: f64) {
    go(s, &CreateCylinder { height: h, radius: r, center: None });
    go(s, &WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 0.0 }), keep: None, merge: None, imprint: None });
    let bodies = list(s, "TopoBody");
    go(s, &Unite { bodies, keep: None, regularize: Some(false), tol: None });
}

fn lateral_faces(s: &Session) -> Vec<EntityId> {
    let t = layer(s).topo();
    t.cells(Dim::Face)
        .iter()
        .copied()
        .filter(|&f| matches!(t.carrier(f), Carrier::Surface(sc) if matches!(t.shapes().surface_def(sc.surface), Some(SurfaceDef::Cylinder(_)))))
        .collect()
}

fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

fn vertex_point(t: &dyn TopologyView, v: EntityId) -> Point3 {
    match t.carrier(v) {
        Carrier::Point(p) => *p,
        c => panic!("vertex carrier {c:?}"),
    }
}

/// The view's canonical hash and the listing of every real kind.
fn view_fingerprint(s: &Session) -> (u64, Vec<Vec<EntityId>>) {
    (layer(s).topo().canonical_hash(), ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume"].iter().map(|k| list(s, k)).collect())
}

fn long_edge(s: &Session) -> EntityId {
    let t = layer(s).topo();
    *t.cells(Dim::Edge).iter().find(|&&e| e.kind() == EntityKind::TopoEdge && t.edge_sampler(e).is_some_and(|x| x.length() > 4.0)).expect("edge")
}

// ------------------------------------------------------------------ acceptance 1

#[test]
fn composite_of_the_cylinder_lateral_halves_has_the_analytic_area() {
    for f in [cell_complex as Factory, topo_store] {
        let (r, h) = (1.5, 4.0);
        let mut s = session_for(f);
        halved_cylinder(&mut s, r, h);
        let lat = lateral_faces(&s);
        assert!(lat.len() >= 2, "lateral face is in pieces: {lat:?}");
        let before = list(&s, "TopoFace").len();
        let d = go(&mut s, &CompositeFaces { faces: lat.clone() });
        let comp = d.outputs["face"][0];
        assert_eq!(comp.kind(), EntityKind::PartitionFace);
        let area = layer(&s).topo().face_moments(comp).unwrap().area;
        let exact = 2.0 * std::f64::consts::PI * r * h;
        assert!((area - exact).abs() <= 1e-12 * exact, "area {area} vs {exact}");
        // One PartitionFace in the listing, in place of the members.
        assert_eq!(list(&s, "PartitionFace"), vec![comp]);
        let faces = list(&s, "TopoFace");
        assert!(lat.iter().all(|m| !faces.contains(m)));
        assert_eq!(faces.len(), before - lat.len(), "model.list of a real kind lists real cells only");
        // The host keeps them.
        assert!(lat.iter().all(|m| layer(&s).host().is_alive(*m)));
        // virtual_info names the members and the op.
        let i = info(&s, Some(comp));
        assert!(i.has_overlay && !i.stale);
        assert_eq!(i.members, lat);
        assert_eq!(i.ops.len(), 1);
        assert_eq!(i.ops[0].op, "composite_faces");
        assert_eq!(i.ops[0].created, vec![comp]);
    }
}

// ------------------------------------------------------------------ acceptance 2

#[test]
fn partition_edge_of_a_circle_splits_the_length_and_puts_the_vertex_on_the_curve() {
    let (r, h) = (1.25, 2.0);
    let exact = 2.0 * std::f64::consts::PI * r;
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateCylinder { height: h, radius: r, center: None });
    let circle = {
        let t = layer(&s).host();
        let e: Vec<EntityId> = t.cells(Dim::Edge).iter().copied().filter(|&e| (t.edge_sampler(e).unwrap().length() - exact).abs() < 1e-9).collect();
        assert_eq!(e.len(), 2);
        e[0]
    };
    let Carrier::Curve(cc) = layer(&s).host().carrier(circle).clone() else { panic!("curve") };
    let param = cc.range.lo + 1.0;
    let d = go(&mut s, &PartitionEdge { edge: circle, param });
    let v = d.outputs["vertex"][0];
    let halves = d.outputs["edges"].clone();
    assert_eq!((v.kind(), halves.len()), (EntityKind::PartitionVertex, 2));
    let t = layer(&s).topo();
    let total: f64 = halves.iter().map(|&e| t.edge_sampler(e).unwrap().length()).sum();
    assert!((total - exact).abs() <= 1e-12 * exact, "{total} vs {exact}");
    // The new vertex is C(t) to 1e-12.
    let want = layer(&s).host().shapes().curve(cc.curve).unwrap().eval(param);
    assert!(dist(vertex_point(t, v), want) < 1e-12);
    // One half is the arc of 1 radian (length r).
    let first = t.edge_sampler(halves[0]).unwrap().length();
    assert!((first - r).abs() < 1e-12 || (first - (exact - r)).abs() < 1e-12, "first half {first}");
    // The original edge is hidden in the view and live in the host.
    assert!(!t.is_alive(circle) && layer(&s).host().is_alive(circle));
    // A parameter outside the range is refused, nothing changes.
    let hash = s.state().hash();
    assert!(try_go(&mut s, &PartitionEdge { edge: halves[0], param: 1.0e6 }).is_err());
    assert_eq!(s.state().hash(), hash);
}

// ------------------------------------------------------------------ partition_face, collapse

fn vertex_at(s: &Session, p: [f64; 3]) -> EntityId {
    let t = layer(s).topo();
    *t.cells(Dim::Vertex).iter().find(|&&v| dist(vertex_point(t, v), Point3::new(p[0], p[1], p[2])) < 1e-9).expect("vertex")
}

fn face_with_vertices(s: &Session, vs: &[EntityId]) -> EntityId {
    let t = layer(s).topo();
    let all = |f: EntityId| {
        let mut found = BTreeSet::new();
        for c in t.boundary(f) {
            for ec in t.boundary(c.cell) {
                found.insert(ec.cell);
            }
        }
        vs.iter().all(|v| found.contains(v))
    };
    *t.cells(Dim::Face).iter().find(|&&f| all(f)).expect("face")
}

#[test]
fn partition_face_by_a_chord_gives_two_triangles_and_a_chord() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    let (a, b) = (vertex_at(&s, [0., 0., 2.]), vertex_at(&s, [2., 2., 2.]));
    let (c, e) = (vertex_at(&s, [2., 0., 2.]), vertex_at(&s, [0., 2., 2.]));
    let top = face_with_vertices(&s, &[a, b, c, e]);
    let d = go(&mut s, &PartitionFace { face: top, start: a, end: b });
    let kids = d.outputs["faces"].clone();
    let chord = d.outputs["chord"][0];
    assert_eq!(kids.len(), 2);
    assert_eq!(chord.kind(), EntityKind::PartitionEdge);
    let t = layer(&s).topo();
    assert!(!t.is_alive(top) && kids.iter().all(|&k| t.is_alive(k)));
    for &k in &kids {
        let n = t.boundary(k).len();
        assert_eq!(n, 3, "a triangle has three coedges");
    }
    // The chord has the diagonal's length and both triangles use it.
    let len = t.edge_sampler(chord).unwrap().length();
    assert!((len - 8f64.sqrt()).abs() < 1e-12);
    assert!(kids.iter().all(|&k| t.boundary(k).iter().any(|l| l.cell == chord)));
    // A degenerate chord is refused, state untouched; a wrong kind is a typed error.
    let hash = s.state().hash();
    assert!(try_go(&mut s, &PartitionFace { face: kids[0], start: a, end: a }).is_err());
    assert!(try_go(&mut s, &PartitionFace { face: a, start: a, end: b }).is_err());
    assert_eq!(s.state().hash(), hash);
}

/// Area of a planar face from its outline corners (straight edges only).
fn outline_area(s: &Session, f: EntityId) -> f64 {
    let pts = layer(s).topo().face_outline(f);
    let mut sum = sm_core::Vector3::new(0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        sum = sum + sm_core::Vector3::new(p.x, p.y, p.z).cross(sm_core::Vector3::new(q.x, q.y, q.z));
    }
    0.5 * sum.norm()
}

fn assert_refused_untouched(s: &mut Session, face: EntityId, from: EntityId, to: EntityId) -> String {
    let hash = s.state().hash();
    let e = try_go(s, &PartitionFace { face, start: from, end: to }).unwrap_err().to_string();
    assert!(e.contains(&face.to_string()), "{e}");
    assert_eq!(s.state().hash(), hash, "a refused op leaves the state as it was");
    e
}

#[test]
fn partition_face_refuses_a_chord_that_leaves_an_l_shaped_face() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(1.0), z: Some(1.0), center: Some([1.0, 0.5, 0.5]) });
    go(&mut s, &CreateBrick { x: 1.0, y: Some(1.0), z: Some(1.0), center: Some([0.5, 1.5, 0.5]) });
    let bodies = list(&s, "TopoBody");
    go(&mut s, &Unite { bodies, keep: None, regularize: None, tol: None });
    let v = |x, y| vertex_at(&s, [x, y, 1.0]);
    let corners = [v(0., 0.), v(2., 0.), v(2., 1.), v(1., 1.), v(1., 2.), v(0., 2.)];
    let top = face_with_vertices(&s, &corners);
    assert_eq!(layer(&s).topo().coedges(top).count(), 6);
    // The chord across the notch lies outside the face; a chord that grazes the notch edge too.
    let e = assert_refused_untouched(&mut s, top, corners[2], corners[4]);
    assert!(e.contains("leaves the face") || e.contains("boundary"), "{e}");
    assert_refused_untouched(&mut s, top, corners[1], corners[4]);
    // An interior diagonal of the lower-left square is accepted and splits the area exactly.
    let kids = go(&mut s, &PartitionFace { face: top, start: corners[0], end: corners[3] }).outputs["faces"].clone();
    assert_eq!(kids.len(), 2);
    let total: f64 = kids.iter().map(|&k| outline_area(&s, k)).sum();
    assert!((total - 3.0).abs() < 1e-12, "{total}");
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
}

#[test]
fn partition_face_refuses_a_chord_along_an_existing_edge_but_accepts_one_across_an_arc() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    let (a, c) = (vertex_at(&s, [0., 0., 2.]), vertex_at(&s, [2., 0., 2.]));
    let top = face_with_vertices(&s, &[a, c, vertex_at(&s, [2., 2., 2.])]);
    let e = assert_refused_untouched(&mut s, top, a, c);
    assert!(e.contains("existing edge") || e.contains("boundary"), "{e}");

    // Quarter disc: the chord between the arc's ends bounds a real circular segment.
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateCylinder { height: 2.0, radius: 1.0, center: None });
    let plane = |s: &Session, axis| WebcutPlane { targets: list(s, "TopoBody"), plane: PlaneSpec::Axis(AxisPlane { axis, offset: 0.0 }), keep: None, merge: None, imprint: None };
    let w = plane(&s, PlaneAxis::X);
    go(&mut s, &w);
    let w = plane(&s, PlaneAxis::Y);
    go(&mut s, &w);
    let t = layer(&s).topo();
    let ztop = t.cells(Dim::Vertex).iter().map(|&v| vertex_point(t, v).z).fold(f64::MIN, f64::max);
    let (o, px, py) = (vertex_at(&s, [0., 0., ztop]), vertex_at(&s, [1., 0., ztop]), vertex_at(&s, [0., 1., ztop]));
    let quarter = face_with_vertices(&s, &[o, px, py]);
    let kids = go(&mut s, &PartitionFace { face: quarter, start: px, end: py }).outputs["faces"].clone();
    assert_eq!(kids.len(), 2);
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
}

#[test]
fn partition_face_accepts_a_coplanar_composite_and_the_children_cover_it() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    go(&mut s, &WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 1.0 }), keep: None, merge: None, imprint: None });
    let bodies = list(&s, "TopoBody");
    go(&mut s, &Unite { bodies, keep: None, regularize: Some(false), tol: None });
    let v = |x, y| vertex_at(&s, [x, y, 2.0]);
    let left = face_with_vertices(&s, &[v(0., 0.), v(1., 0.), v(1., 2.), v(0., 2.)]);
    let right = face_with_vertices(&s, &[v(1., 0.), v(2., 0.), v(2., 2.), v(1., 2.)]);
    assert_ne!(left, right);
    let (p0, p1) = (v(0., 0.), v(2., 2.));
    let comp = go(&mut s, &CompositeFaces { faces: vec![left, right] }).outputs["face"][0];
    let kids = go(&mut s, &PartitionFace { face: comp, start: p0, end: p1 }).outputs["faces"].clone();
    assert_eq!(kids.len(), 2);
    let areas: Vec<f64> = kids.iter().map(|&k| outline_area(&s, k)).collect();
    assert!(areas.iter().all(|a| (a - 2.0).abs() < 1e-12), "{areas:?}");
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
}

#[test]
fn partition_face_refuses_a_curved_face_with_its_id() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateCylinder { height: 2.0, radius: 1.0, center: None });
    let lat = lateral_faces(&s);
    assert_eq!(lat.len(), 1);
    let vs = list(&s, "TopoVertex");
    let hash = s.state().hash();
    let e = try_go(&mut s, &PartitionFace { face: lat[0], start: vs[0], end: vs[1] }).unwrap_err().to_string();
    assert!(e.contains(&lat[0].to_string()) && e.contains("not planar"), "{e}");
    assert_eq!(s.state().hash(), hash, "a refused op leaves the state as it was");
}

#[test]
fn partition_face_refuses_bent_and_curved_composites_with_the_id() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    let top = face_with_vertices(&s, &[vertex_at(&s, [0., 0., 2.]), vertex_at(&s, [2., 2., 2.]), vertex_at(&s, [2., 0., 2.]), vertex_at(&s, [0., 2., 2.])]);
    let front = face_with_vertices(&s, &[vertex_at(&s, [0., 0., 0.]), vertex_at(&s, [2., 0., 0.]), vertex_at(&s, [2., 0., 2.]), vertex_at(&s, [0., 0., 2.])]);
    let comp = go(&mut s, &CompositeFaces { faces: vec![top, front] }).outputs["face"][0];
    let (a, b) = (vertex_at(&s, [0., 2., 2.]), vertex_at(&s, [2., 0., 0.]));
    let hash = s.state().hash();
    let e = try_go(&mut s, &PartitionFace { face: comp, start: a, end: b }).unwrap_err().to_string();
    assert!(e.contains(&comp.to_string()) && e.contains("not planar"), "{e}");
    assert_eq!(s.state().hash(), hash, "a refused op leaves the state as it was");

    // A composite of the halves of a cylinder's lateral face is curved.
    let mut s = session_for(cell_complex);
    halved_cylinder(&mut s, 1.0, 2.0);
    let lat = lateral_faces(&s);
    assert!(lat.len() >= 2);
    let comp = go(&mut s, &CompositeFaces { faces: lat }).outputs["face"][0];
    let vs = list(&s, "TopoVertex");
    let hash = s.state().hash();
    let e = try_go(&mut s, &PartitionFace { face: comp, start: vs[0], end: vs[1] }).unwrap_err().to_string();
    assert!(e.contains(&comp.to_string()) && e.contains("not planar"), "{e}");
    assert_eq!(s.state().hash(), hash);
}

#[test]
fn collapse_edge_hides_the_edge_records_the_gap_and_remove_restores() {
    let mut s = session_for(cell_complex);
    // A thin plate: its four z-edges are 1e-4 long.
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(1e-4), center: Some([1.0, 1.0, 0.0]) });
    let t = layer(&s).topo();
    let short: EntityId = *t.cells(Dim::Edge).iter().find(|&&e| (t.edge_sampler(e).unwrap().length() - 1e-4).abs() < 1e-12).expect("short edge");
    let ends: Vec<EntityId> = t.boundary(short).iter().map(|i| i.cell).collect();
    let pre = view_fingerprint(&s);
    let d = go(&mut s, &CollapseEdge { edge: short, into_vertex: ends[0] });
    let t = layer(&s).topo();
    assert!(!t.is_alive(short));
    assert!(t.is_alive(ends[0]) && !t.is_alive(ends[1]));
    assert!(d.created.iter().all(|c| c.kind() == EntityKind::PartitionEdge));
    let i = info(&s, Some(short));
    assert_eq!(i.ops.len(), 1);
    let gap = i.ops[0].gap.expect("gap");
    assert!((1e-4 - 1e-15..1e-3).contains(&gap), "gap {gap}");
    // Undo by name: the view is the host's again.
    go(&mut s, &RemoveVirtual { id: short });
    assert_eq!(view_fingerprint(&s), pre);
    assert!(info(&s, None).ops.is_empty());
}

#[test]
fn composite_edges_refusals_are_typed_and_leave_no_overlay() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: None });
    let edges: Vec<EntityId> = layer(&s).topo().cells(Dim::Edge).to_vec();
    let hash = s.state().hash();
    // Brick edges meet at degree-3 vertices, or not at all: refused by the overlay.
    let e = try_go(&mut s, &CompositeEdges { edges: edges[..2].to_vec() }).unwrap_err().to_string();
    assert!(e.contains("model.composite_edges"), "{e}");
    assert_eq!(s.state().hash(), hash);
    assert!(layer(&s).overlay().is_none(), "a refused first op does not create the overlay");
    // A wrong kind is refused before the overlay is touched.
    let v = id(EntityKind::TopoVertex, 1);
    assert!(try_go(&mut s, &CompositeEdges { edges: vec![v, v] }).is_err());
}

// ------------------------------------------------------------------ acceptance 3

fn scenario() -> Vec<Box<dyn Op>> {
    vec![
        Box::new(CreateCylinder { height: 4.0, radius: 1.5, center: None }),
        Box::new(WebcutPlane { targets: vec![id(EntityKind::TopoBody, 1)], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 0.0 }), keep: None, merge: None, imprint: None }),
        Box::new(Unite { bodies: vec![id(EntityKind::TopoBody, 2), id(EntityKind::TopoBody, 3)], keep: None, regularize: Some(false), tol: None }),
    ]
}

#[test]
fn replay_fork_and_undo_give_equal_hashes_and_remove_restores_the_view() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session_for(factory);
        for op in scenario() {
            go(&mut s, op.as_ref());
        }
        let lat = lateral_faces(&s);
        let base_view = view_fingerprint(&s);
        let base_hash = s.state().hash();
        let comp = go(&mut s, &CompositeFaces { faces: lat.clone() }).outputs["face"][0];
        let circle = long_edge(&s);
        let Carrier::Curve(cc) = layer(&s).topo().carrier(circle).clone() else { panic!() };
        let param = 0.5 * (cc.range.lo + cc.range.hi);
        let pe = go(&mut s, &PartitionEdge { edge: circle, param });
        let full = s.state().hash();
        let q = QuietControl::new();
        let head = s.position().unwrap();

        // Cold replay from the root re-checks every recorded hash.
        let cold = s.state_at_from_root(head, &q.control()).expect("cold replay");
        assert_eq!(cold.hash(), full);
        assert_eq!(cold.layer_hashes(), s.state().layer_hashes());

        // A second session fed the recorded ops (through JSON) gives the same hash and overlay.
        let mut twin = session_for(factory);
        for op in scenario() {
            go(&mut twin, op.as_ref());
        }
        let ops: [Box<dyn Op>; 2] = [Box::new(CompositeFaces { faces: lat.clone() }), Box::new(PartitionEdge { edge: circle, param })];
        for op in ops {
            let rec = OpRecord::new(op.name(), 1, op.encode().unwrap()).unwrap();
            let decoded = twin.engine().registry().decode(&rec).unwrap();
            go(&mut twin, decoded.as_ref());
        }
        assert_eq!(twin.state().hash(), full);
        assert_eq!(layer(&twin).overlay().unwrap().data(), layer(&s).overlay().unwrap().data());

        // Fork: a branch diverges (remove the partition) without touching the original node.
        twin.branch("fork").unwrap();
        go(&mut twin, &RemoveVirtual { id: pe.outputs["vertex"][0] });
        let forked = twin.state().hash();
        assert_ne!(forked, full);
        twin.checkout(Rev::Node(head), &q.control()).unwrap();
        assert_eq!(twin.state().hash(), full);
        twin.checkout(Rev::Ref("fork".into()), &q.control()).unwrap();
        assert_eq!(twin.state().hash(), forked);

        // Undo/redo restore the overlay exactly.
        s.undo().unwrap();
        assert_eq!(layer(&s).overlay().unwrap().ops().len(), 1);
        s.undo().unwrap();
        assert!(layer(&s).overlay().is_none());
        assert_eq!(s.state().hash(), base_hash);
        s.redo().unwrap();
        s.redo().unwrap();
        assert_eq!(s.state().hash(), full);

        // remove_virtual (newest first) restores the VIEW; the hash differs only by counters.
        go(&mut s, &RemoveVirtual { id: pe.outputs["vertex"][0] });
        go(&mut s, &RemoveVirtual { id: comp });
        assert_eq!(view_fingerprint(&s), base_view);
        assert_ne!(s.state().hash(), base_hash, "the overlay's id counters are state");
        assert!(info(&s, None).ops.is_empty());
        // The removed ids left the entity table, and the next minted id is fresh.
        assert!(!s.state().contains(comp));
        let again = go(&mut s, &CompositeFaces { faces: lat.clone() }).outputs["face"][0];
        assert!(again.serial() > comp.serial());
    }
}

#[test]
fn remove_virtual_is_refused_while_a_later_op_uses_the_cell_and_for_unknown_ids() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: None });
    let edge = list(&s, "TopoEdge")[0];
    let pe = go(&mut s, &PartitionEdge { edge, param: 0.5 });
    let half = pe.outputs["edges"][0];
    go(&mut s, &PartitionEdge { edge: half, param: 0.25 });
    let hash = s.state().hash();
    let e = try_go(&mut s, &RemoveVirtual { id: half }).unwrap_err().to_string();
    assert!(e.contains(&half.to_string()), "{e}");
    assert_eq!(s.state().hash(), hash);
    assert!(try_go(&mut s, &RemoveVirtual { id: id(EntityKind::TopoFace, 1) }).is_err());
    assert_eq!(s.state().hash(), hash);
}

// ------------------------------------------------------------------ acceptance 4

#[test]
fn a_host_edit_that_breaks_the_overlay_is_refused_with_the_cell_and_others_are_rebased() {
    let mut s = session_for(cell_complex);
    halved_cylinder(&mut s, 1.5, 4.0);
    let lat = lateral_faces(&s);
    let comp = go(&mut s, &CompositeFaces { faces: lat.clone() }).outputs["face"][0];
    let hash = s.state().hash();

    // Deleting the body kills the members: typed refusal that names a member, state unchanged.
    let body = list(&s, "TopoBody")[0];
    let e = try_go(&mut s, &Delete { bodies: vec![body] }).unwrap_err().to_string();
    assert!(e.contains("breaks the virtual topology") && lat.iter().any(|m| e.contains(&m.to_string())), "{e}");
    assert_eq!(s.state().hash(), hash);

    // A boolean on the body (webcut at z = 0) touches the members: refused, never corrupted.
    let e = try_go(&mut s, &WebcutPlane { targets: vec![body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::Z, offset: 0.0 }), keep: None, merge: None, imprint: None }).unwrap_err().to_string();
    assert!(e.contains("virtual topology"), "{e}");
    assert_eq!(s.state().hash(), hash);
    assert!(layer(&s).topo().is_alive(comp));

    // An edit elsewhere (a new brick) rebases the overlay onto the new host revision.
    go(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([20.0, 0.0, 0.0]) });
    let l = layer(&s);
    assert!(l.topo().is_alive(comp));
    assert_eq!(l.topo().revision(), l.host().revision());
    assert!(!info(&s, None).stale);

    // After remove_virtual the same delete goes through.
    go(&mut s, &RemoveVirtual { id: comp });
    go(&mut s, &Delete { bodies: vec![body] });
}

// ------------------------------------------------------------------ acceptance 6

#[test]
fn realized_splits_replay_with_every_cut_cell_resolving() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = session_for(factory);
        go(&mut s, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
        let vol = id(EntityKind::TopoVolume, 1);
        let d = go(&mut s, &SplitRealized { piece: vol, normal: [1.0, 0.0, 0.0], offset: 1.0 });
        let kids = d.outputs["pieces"].clone();
        assert_eq!(kids.len(), 2);
        assert!(kids.iter().all(|k| k.kind() == EntityKind::PartitionCell));
        // Split the positive piece again (nested).
        let d2 = go(&mut s, &SplitRealized { piece: kids[1], normal: [1.0, 0.0, 0.0], offset: 3.0 });
        assert_eq!(d2.outputs["pieces"].len(), 2);
        let full = s.state().hash();
        let head = s.position().unwrap();

        let check = |st: &sm_op_api::State| {
            let l = st.layer::<ModelLayer>().unwrap();
            let ov = l.overlay().expect("overlay");
            let dec = l.model().decomposition();
            for piece in [PieceRef::Whole(vol), PieceRef::Cell(kids[1])] {
                let cut = dec.split_of(piece).expect("split recorded").cut;
                let cc = dec.cut_cells(cut).expect("realized");
                let all: Vec<EntityId> = cc.faces.iter().chain(&cc.edges).chain(&cc.vertices).copied().collect();
                assert!(!all.is_empty());
                assert!(all.iter().all(|c| l.topo().is_alive(*c) && ov.is_alive(*c)), "cut cells resolve");
            }
            assert_eq!(l.topo().cells(Dim::Volume).len(), 3);
        };
        check(s.state());

        // Cold replay (the records alone, as after save and load) resolves the same ids.
        let q = QuietControl::new();
        let cold = s.state_at_from_root(head, &q.control()).expect("cold replay");
        assert_eq!(cold.hash(), full);
        check(&cold);

        // Unsplit in reverse order retires the virtual ids and returns the whole volume.
        let virt: Vec<EntityId> = layer(&s).overlay().unwrap().cells(Dim::Face).iter().copied().filter(|f| f.kind() == EntityKind::PartitionFace).collect();
        assert!(!virt.is_empty());
        go(&mut s, &UnsplitRealized { piece: d2.outputs["pieces"][0] });
        go(&mut s, &UnsplitRealized { piece: kids[0] });
        assert_eq!(layer(&s).topo().cells(Dim::Volume), &[vol]);
        let left: Vec<_> = virt.iter().filter(|v| s.state().contains(**v)).collect();
        assert!(left.is_empty(), "{left:?} still in table; overlay faces {:?}", layer(&s).overlay().map(|o| o.cells(Dim::Face).to_vec()));
        // A bad plane is refused whole.
        let hash = s.state().hash();
        assert!(try_go(&mut s, &SplitRealized { piece: vol, normal: [0.0, 0.0, 0.0], offset: 1.0 }).is_err());
        assert!(try_go(&mut s, &SplitRealized { piece: vol, normal: [1.0, 0.0, 0.0], offset: 100.0 }).is_err());
        assert_eq!(s.state().hash(), hash);
    }
}

// ------------------------------------------------------------------ determinism

fn determinism_run(factory: Factory) -> (Hash256, Hash256) {
    let mut s = session_for(factory);
    for op in scenario() {
        go(&mut s, op.as_ref());
    }
    let lat = lateral_faces(&s);
    go(&mut s, &CompositeFaces { faces: lat });
    let edge = long_edge(&s);
    let Carrier::Curve(cc) = layer(&s).topo().carrier(edge).clone() else { panic!() };
    go(&mut s, &PartitionEdge { edge, param: 0.5 * (cc.range.lo + cc.range.hi) });
    (s.state().hash(), s.state().layer_hash("model").unwrap())
}

#[test]
fn virtual_ops_are_deterministic_across_runs_threads_and_hosts() {
    let a = sm_testkit::determinism::check(2, 4, |_t| determinism_run(cell_complex)).expect("deterministic");
    let b = sm_testkit::determinism::check(2, 4, |_t| determinism_run(topo_store)).expect("deterministic");
    assert_eq!(a, b, "both hosts give one state hash");
}

// ------------------------------------------------------------------ acceptance 5

proptest! {
    #![proptest_config(sm_testkit::strategies::config(if cfg!(debug_assertions) { 16 } else { 64 }))]

    /// Random composite / partition / remove sequences on a brick keep the view valid and never
    /// reuse an id, including ids that were removed.
    #[test]
    fn random_virtual_sequences_keep_validate_clean_and_never_reuse_ids(
        steps in proptest::collection::vec((0u8..5, 0usize..64, 0usize..64, 0.05f64..0.95), 1..14)
    ) {
        let mut s = session_for(cell_complex);
        go(&mut s, &CreateBrick { x: 2.0, y: Some(3.0), z: Some(4.0), center: None });
        let mut ever: BTreeSet<EntityId> = s.state().entities().collect();
        for (kind, i, j, t) in steps {
            let view = layer(&s).topo();
            let pick = |d: Dim, k: usize| { let c = view.cells(d); c[k % c.len()] };
            let op: Box<dyn Op> = match kind {
                0 => {
                    let e = pick(Dim::Edge, i);
                    let Carrier::Curve(cc) = view.carrier(e).clone() else { continue };
                    Box::new(PartitionEdge { edge: e, param: cc.range.lo + t * (cc.range.hi - cc.range.lo) })
                }
                1 => Box::new(PartitionFace { face: pick(Dim::Face, i), start: pick(Dim::Vertex, i + j), end: pick(Dim::Vertex, j) }),
                2 => Box::new(CompositeFaces { faces: vec![pick(Dim::Face, i), pick(Dim::Face, j)] }),
                _ => {
                    let inf = info(&s, None);
                    let Some(o) = inf.ops.get(i % inf.ops.len().max(1)) else { continue };
                    let Some(&c) = o.created.first() else { continue };
                    Box::new(RemoveVirtual { id: c })
                }
            };
            // Refusals are fine (and leave the state alone); successes keep the invariants.
            let before = s.state().hash();
            match try_go(&mut s, op.as_ref()) {
                Err(_) => prop_assert_eq!(s.state().hash(), before),
                Ok(d) => {
                    for c in &d.created {
                        prop_assert!(ever.insert(*c), "id {:?} reused", c);
                    }
                    let issues = sm_topo_api::validate(layer(&s).topo());
                    prop_assert!(issues.is_empty(), "validate: {:?}", issues);
                }
            }
        }
    }
}

// ------------------------------------------------------------------ W4-VTOPO-FIX

fn realized_brick() -> (Session, Vec<EntityId>) {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
    let d = go(&mut s, &SplitRealized { piece: id(EntityKind::TopoVolume, 1), normal: [1.0, 0.0, 0.0], offset: 1.0 });
    let kids = d.outputs["pieces"].clone();
    (s, kids)
}

fn cut_of_whole(s: &Session) -> sm_model_api::CutCells {
    let dec = layer(s).model().decomposition();
    let cut = dec.split_of(PieceRef::Whole(id(EntityKind::TopoVolume, 1))).expect("split").cut;
    dec.cut_cells(cut).expect("realized").clone()
}

#[test]
fn plain_virtual_ops_refuse_the_cells_of_a_realized_split() {
    let (mut s, kids) = realized_brick();
    let cc = cut_of_whole(&s);
    assert!(!cc.faces.is_empty() && !cc.edges.is_empty() && !cc.split_faces.is_empty() && !cc.split_edges.is_empty());
    let hash = s.state().hash();
    let mut owned: Vec<EntityId> = cc.faces.iter().chain(&cc.edges).chain(&cc.vertices).chain(&kids).copied().collect();
    for (f, h) in &cc.split_faces {
        owned.push(*f);
        owned.extend(h);
    }
    for (e, v, h) in &cc.split_edges {
        owned.push(*e);
        owned.push(*v);
        owned.extend(h);
    }
    // remove_virtual on any cell the split created (cut face, either piece, edges, vertices, halves).
    let mut tried = 0;
    for &c in &owned {
        if layer(&s).overlay().unwrap().ops().iter().all(|o| !o.created().contains(&c)) {
            continue; // a hidden parent, not a created cell
        }
        tried += 1;
        let e = try_go(&mut s, &RemoveVirtual { id: c }).unwrap_err().to_string();
        assert!(e.contains("unsplit_realized"), "{c}: {e}");
        assert_eq!(s.state().hash(), hash, "{c}: refusal leaves the state alone");
    }
    assert!(tried >= 8, "{tried}");
    // Partition of the cut face and of a cut edge.
    let cut_face = cc.faces[0];
    let e = try_go(&mut s, &PartitionFace { face: cut_face, start: cc.vertices[0], end: cc.vertices[2] }).unwrap_err().to_string();
    assert!(e.contains("unsplit_realized"), "{e}");
    assert_eq!(s.state().hash(), hash);
    let cut_edge = cc.edges[0];
    let e = try_go(&mut s, &PartitionEdge { edge: cut_edge, param: 0.5 }).unwrap_err().to_string();
    assert!(e.contains("unsplit_realized"), "{e}");
    assert_eq!(s.state().hash(), hash);
    // Collapse: the cut edge itself, and an edge whose dropped end is a cut vertex.
    let ends: Vec<EntityId> = layer(&s).topo().boundary(cut_edge).iter().map(|i| i.cell).collect();
    let e = try_go(&mut s, &CollapseEdge { edge: cut_edge, into_vertex: ends[0] }).unwrap_err().to_string();
    assert!(e.contains("unsplit_realized"), "{e}");
    let (_, v, halves) = cc.split_edges[0];
    let ends: Vec<EntityId> = layer(&s).topo().boundary(halves[0]).iter().map(|i| i.cell).collect();
    let keep = *ends.iter().find(|&&x| x != v).expect("far end");
    let e = try_go(&mut s, &CollapseEdge { edge: halves[0], into_vertex: keep }).unwrap_err().to_string();
    assert!(e.contains("unsplit_realized"), "{e}");
    // Composites over halves of cut edges / faces.
    assert!(try_go(&mut s, &CompositeEdges { edges: vec![halves[0], halves[1]] }).is_err());
    assert!(try_go(&mut s, &CompositeFaces { faces: vec![cc.split_faces[0].1[0], cc.split_faces[0].1[1]] }).is_err());
    assert_eq!(s.state().hash(), hash);

    // The decomposition's cut cells still resolve, the view validates, and unsplit still works.
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
    assert!(cc.faces.iter().chain(&cc.edges).chain(&cc.vertices).all(|c| layer(&s).topo().is_alive(*c)));
    go(&mut s, &UnsplitRealized { piece: kids[0] });
    assert!(layer(&s).overlay().unwrap().ops().is_empty());
}

#[test]
fn virtual_info_lists_pieces_and_a_splits_cut_edges() {
    let (s, kids) = realized_brick();
    let cc = cut_of_whole(&s);
    let i = info(&s, Some(kids[0]));
    let op = i.ops.iter().find(|o| o.op == "partition_volume").expect("volume op");
    assert!(cc.edges.iter().all(|e| op.inputs.contains(e)), "cut edges are inputs: {:?}", op.inputs);
    assert!(op.created.contains(&kids[0]));
}

#[test]
fn split_ops_round_trip_through_op_records() {
    let (mut s, kids) = realized_brick();
    let ops: [Box<dyn Op>; 2] = [
        Box::new(SplitRealized { piece: id(EntityKind::TopoVolume, 1), normal: [1.0, 0.0, 0.0], offset: 1.0 }),
        Box::new(UnsplitRealized { piece: kids[0] }),
    ];
    for op in &ops {
        let rec = OpRecord::of(op.as_ref()).unwrap();
        let back = s.engine().registry().decode(&rec).unwrap();
        assert_eq!(OpRecord::of(back.as_ref()).unwrap(), rec);
    }
    // Replay the decoded forms on a twin.
    let mut twin = session_for(cell_complex);
    go(&mut twin, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
    let rec = OpRecord::of(ops[0].as_ref()).unwrap();
    go(&mut twin, s.engine().registry().decode(&rec).unwrap().as_ref());
    assert_eq!(twin.state().hash(), s.state().hash());
    let rec = OpRecord::of(ops[1].as_ref()).unwrap();
    go(&mut twin, s.engine().registry().decode(&rec).unwrap().as_ref());
    go(&mut s, ops[1].as_ref());
    assert_eq!(twin.state().hash(), s.state().hash());
}

#[test]
fn model_list_of_a_real_kind_excludes_virtual_cells() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    let (a, b) = (vertex_at(&s, [0., 0., 2.]), vertex_at(&s, [2., 2., 2.]));
    let top = face_with_vertices(&s, &[a, b, vertex_at(&s, [2., 0., 2.])]);
    let kids = go(&mut s, &PartitionFace { face: top, start: a, end: b }).outputs["faces"].clone();
    assert!(list(&s, "TopoFace").iter().all(|f| f.kind() == EntityKind::TopoFace));
    assert_eq!(list(&s, "TopoFace").len(), 5);
    assert_eq!(list(&s, "PartitionFace"), kids);
}

#[test]
fn chord_checks_use_the_exact_arc_of_a_concave_notch() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
    go(&mut s, &CreateCylinder { height: 4.0, radius: 1.0, center: Some([2.0, 0.0, 1.0]) });
    go(&mut s, &Subtract { targets: vec![id(EntityKind::TopoBody, 1)], tools: vec![id(EntityKind::TopoBody, 2)], keep: None, regularize: None, tol: None });
    let v = |x, y| vertex_at(&s, [x, y, 2.0]);
    let (p0, p1, p3, p4, p5, p6) = (v(0., 0.), v(1., 0.), v(3., 0.), v(4., 0.), v(0., 2.), v(4., 2.));
    let top = face_with_vertices(&s, &[p0, p1, p3, p4, p5]);
    // The chord between the notch ends spans the cut-out: it leaves the face.
    assert_refused_untouched(&mut s, top, p1, p3);
    // A chord from a notch end back into the notch crosses the arc: refused. One away from it is fine.
    assert_refused_untouched(&mut s, top, p1, p6);
    let kids = go(&mut s, &PartitionFace { face: top, start: p1, end: p5 }).outputs["faces"].clone();
    assert_eq!(kids.len(), 2);
    assert!(sm_topo_api::validate(layer(&s).topo()).is_empty());
}

#[test]
fn an_overlay_state_hash_is_pinned() {
    let build = |f: Factory| {
        let mut s = session_for(f);
        go(&mut s, &CreateBrick { x: 2.0, y: Some(3.0), z: Some(4.0), center: None });
        let edge = list(&s, "TopoEdge")[0];
        go(&mut s, &PartitionEdge { edge, param: 0.5 });
        s.state().hash()
    };
    let (a, b) = (build(cell_complex), build(topo_store));
    assert_eq!(a, b);
    assert_eq!(format!("{a:?}"), "Hash256(83cf0a9aba4e)", "update the pin only for a reviewed overlay change");
}
