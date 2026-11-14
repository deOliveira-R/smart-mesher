//! ACR-V1: the layer's shared virtual-topology overlay. `topo()` is the overlay view, `host()`
//! the bare host; the digest changes only when an overlay is set; host edits follow the overlay
//! or are refused; the decomposition's realized splits write into THE layer's overlay.

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane as GPlane, SurfaceDef};
use sm_layer_api::{
    CoreLayers, LayerError, LayerResultExt, ModelLayer, TopoHost, commit_topology,
    model_overlay_txn, topo_txn,
};
use sm_model_api::{Model, PieceRef, Plane};
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Layer, Meta, Op, OpError, QuietControl, Registry, Services,
    Session, TypedOp,
};
use sm_topo_api::fixtures::{brick, with_op};
use sm_topo_api::{
    CellComplex, CoedgeSpec, CurveCarrier, Dim, OpId, Overlay, Sense, SurfaceCarrier,
    TopologyEdit, TopologyView,
};

/// Two coplanar unit squares sharing one edge. Returns the host and the two faces.
fn strip() -> (CellComplex, [EntityId; 2]) {
    let mut t = CellComplex::new();
    let faces = with_op(&mut t, OpId(1), |t| Ok(make_strip(t))).unwrap();
    (t, faces)
}

fn make_strip(t: &mut dyn TopologyEdit) -> [EntityId; 2] {
    let pt = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let p = [pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(0.0, 1.0), pt(2.0, 0.0), pt(2.0, 1.0)];
    let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(*q).unwrap()).collect();
    let edge = |t: &mut dyn TopologyEdit, a: usize, b: usize| {
        let d = p[b] - p[a];
        let curve = t
            .shapes_mut()
            .unwrap()
            .add_curve(CurveDef::Line(Line::new(p[a], d).unwrap()))
            .unwrap();
        t.make_edge(
            v[a],
            v[b],
            Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }),
        )
        .unwrap()
    };
    let (e01, e12, e23, e30) = (edge(t, 0, 1), edge(t, 1, 2), edge(t, 2, 3), edge(t, 3, 0));
    let (e14, e45, e52) = (edge(t, 1, 4), edge(t, 4, 5), edge(t, 5, 2));
    let c = |e: EntityId, fwd: bool| CoedgeSpec {
        edge: e,
        sense: if fwd { Sense::Forward } else { Sense::Reversed },
    };
    let plane = SurfaceDef::Plane(GPlane {
        frame: Frame::from_z(pt(0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap(),
    });
    let surf = t.shapes_mut().unwrap().add_surface(plane).unwrap();
    let sc = Some(SurfaceCarrier { surface: surf, sense: Sense::Forward });
    let f1 = t
        .make_face(&[&[c(e01, true), c(e12, true), c(e23, true), c(e30, true)]], sc)
        .unwrap();
    let f2 = t
        .make_face(&[&[c(e14, true), c(e45, true), c(e52, true), c(e12, false)]], sc)
        .unwrap();
    [f1, f2]
}

fn layer_over(t: CellComplex) -> ModelLayer {
    ModelLayer::from_parts(Arc::new(t) as Arc<dyn TopoHost>, Arc::new(Model::new()))
}

fn same_object(a: &dyn TopologyView, b: &dyn TopologyView) -> bool {
    std::ptr::addr_eq(a as *const dyn TopologyView, b as *const dyn TopologyView)
}

#[test]
fn topo_sees_the_virtual_face_host_does_not_and_the_digest_moves_only_then() {
    let (t, faces) = strip();
    let mut layer = layer_over(t);
    let h0 = Layer::content_hash(&layer);
    // No overlay: one object, no change.
    assert!(layer.overlay().is_none());
    assert!(same_object(layer.topo(), layer.host()));
    assert_eq!(layer.topo().cells(Dim::Face).len(), 2);

    let virt = layer.overlay_mut(|ov| ov.composite_faces(&faces)).unwrap();
    assert_eq!(virt.kind(), EntityKind::PartitionFace);
    assert!(!same_object(layer.topo(), layer.host()));
    // The overlay view has the composite and hides the members; the host is untouched.
    assert!(layer.topo().is_alive(virt));
    assert!(faces.iter().all(|f| !layer.topo().is_alive(*f)));
    assert_eq!(layer.topo().cells(Dim::Face), &[virt]);
    assert!(!layer.host().is_alive(virt));
    assert!(faces.iter().all(|f| layer.host().is_alive(*f)));
    assert_eq!(layer.host().cells(Dim::Face).len(), 2);
    assert_eq!(layer.topo_host().cells(Dim::Face).len(), 2);
    let h1 = Layer::content_hash(&layer);
    assert_ne!(h0, h1);

    // Fork (clone) shares the overlay and hashes equal; editing the fork copies, not the origin.
    let mut fork = layer.clone();
    assert_eq!(Layer::content_hash(&fork), h1);
    assert!(Arc::ptr_eq(fork.overlay().unwrap(), layer.overlay().unwrap()));
    fork.overlay_mut(|ov| ov.remove(virt)).unwrap();
    assert_eq!(layer.topo().cells(Dim::Face), &[virt]);
    assert_eq!(fork.topo().cells(Dim::Face).len(), 2);
    // Removing restores the view but not the hash: the id counters are state.
    assert_ne!(Layer::content_hash(&fork), h0);
    assert_ne!(Layer::content_hash(&fork), h1);

    // Replay: the persisted record on an equal host gives the same layer hash.
    let data = layer.overlay().unwrap().data();
    let (t2, _) = strip();
    let mut replay = layer_over(t2);
    let host: Arc<dyn TopologyView> = replay.topo_host().clone();
    replay.set_overlay(Overlay::from_data(host, data).unwrap()).unwrap();
    assert_eq!(Layer::content_hash(&replay), h1);

    // An empty overlay still differs from none; clearing returns to the original hash.
    let (t3, _) = strip();
    let mut empty = layer_over(t3);
    empty.overlay_mut(|_| ());
    assert_ne!(Layer::content_hash(&empty), h0);
    assert!(!sm_op_api::LayerType::is_empty(&empty));
    empty.clear_overlay();
    assert_eq!(Layer::content_hash(&empty), h0);
}

#[test]
fn set_overlay_refuses_an_overlay_of_another_host() {
    let (t, _) = strip();
    let mut layer = layer_over(t);
    let mut other = CellComplex::new();
    with_op(&mut other, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let ov = Overlay::new(Arc::new(other));
    let e = layer.set_overlay(ov).unwrap_err();
    assert!(matches!(e, LayerError::Overlay(_)), "{e}");
    assert!(layer.overlay().is_none());
}

// ------------------------------------------------------------------ session ops

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MakeStrip {}

impl TypedOp for MakeStrip {
    const NAME: &'static str = "test.make_strip";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (faces, txn) = topo_txn(ctx, |topo, _| Ok(make_strip(topo)))?;
        commit_topology(ctx, &txn, &[("faces", &faces)])
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MakeBrick {}

impl TypedOp for MakeBrick {
    const NAME: &'static str = "test.make_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| {
            brick(topo, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0]).or_op()
        })?;
        commit_topology(ctx, &txn, &[("volume", &[ids.volume]), ("faces", &ids.faces)])
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Composite {
    faces: Vec<EntityId>,
}

impl TypedOp for Composite {
    const NAME: &'static str = "test.composite";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let layer = ctx.layer_mut::<ModelLayer>()?;
        layer
            .overlay_mut(|ov| ov.composite_faces(&self.faces))
            .map(|_| ())
            .or_op()
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct AddVertex {}

impl TypedOp for AddVertex {
    const NAME: &'static str = "test.add_vertex";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (v, txn) = topo_txn(ctx, |topo, _| {
            topo.make_vertex(Point3::new(9.0, 9.0, 9.0)).or_op()
        })?;
        commit_topology(ctx, &txn, &[("vertex", &[v])])
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct KillFace {
    face: EntityId,
}

impl TypedOp for KillFace {
    const NAME: &'static str = "test.kill_face";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (_, txn) = topo_txn(ctx, |topo, _| topo.kill(self.face).or_op())?;
        commit_topology(ctx, &txn, &[])
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct SplitRealized {
    volume: EntityId,
    x: f64,
}

impl TypedOp for SplitRealized {
    const NAME: &'static str = "test.split_realized";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let plane = Plane::new(Vector3::new(1.0, 0.0, 0.0), self.x)
            .map_err(|e| OpError::Invalid(e.to_string()))?;
        model_overlay_txn(
            ctx,
            |m, ov| {
                m.split_realized(PieceRef::Whole(self.volume), plane, ov)
                    .map_err(|e| OpError::Failed(e.to_string()))
            },
            |kids, _| vec![("pieces", kids.to_vec())],
        )
        .map(|_| ())
    }
}

fn session() -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(|| Box::new(CellComplex::new()))).unwrap();
    reg.register::<MakeStrip>().unwrap();
    reg.register::<MakeBrick>().unwrap();
    reg.register::<Composite>().unwrap();
    reg.register::<AddVertex>().unwrap();
    reg.register::<KillFace>().unwrap();
    reg.register::<SplitRealized>().unwrap();
    let engine =
        Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default())).unwrap();
    Session::new(engine)
}

fn run(s: &mut Session, op: &dyn Op) -> Result<sm_op_api::Delta, sm_op_api::TimelineError> {
    let q = QuietControl::new();
    s.apply(op, Meta::new("test", "", ""), &q.control()).map(|(_, d)| d)
}

fn model(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().expect("model layer")
}

fn face(serial: u64) -> EntityId {
    EntityId::new(EntityKind::TopoFace, serial).unwrap()
}

#[test]
fn a_host_edit_follows_the_overlay_and_one_that_breaks_it_is_refused_whole() {
    let mut s = session();
    run(&mut s, &MakeStrip {}).unwrap();
    run(&mut s, &Composite { faces: vec![face(1), face(2)] }).unwrap();
    let virt = model(&s).topo().cells(Dim::Face)[0];
    assert_eq!(virt.kind(), EntityKind::PartitionFace);

    // An unrelated edit succeeds and the overlay follows the new host revision.
    run(&mut s, &AddVertex {}).unwrap();
    let l = model(&s);
    assert!(l.topo().is_alive(virt));
    assert_eq!(l.topo().revision(), l.host().revision());
    assert_eq!(l.overlay().unwrap().ops().len(), 1);
    assert_eq!(l.host().cells(Dim::Face).len(), 2);

    // Deleting a member breaks the composite: typed refusal, nothing changes.
    let before = s.state().hash();
    let e = run(&mut s, &KillFace { face: face(1) }).unwrap_err();
    assert!(e.to_string().contains("breaks the virtual topology"), "{e}");
    assert_eq!(s.state().hash(), before);
    assert!(model(&s).host().is_alive(face(1)));
    assert!(model(&s).topo().is_alive(virt));
}

#[test]
fn a_realized_split_writes_into_the_layers_overlay() {
    let mut s = session();
    run(&mut s, &MakeBrick {}).unwrap();
    let vol = EntityId::new(EntityKind::TopoVolume, 1).unwrap();
    assert!(model(&s).overlay().is_none());
    let delta = run(&mut s, &SplitRealized { volume: vol, x: 1.0 }).unwrap();
    let l = model(&s);
    let ov = l.overlay().expect("created on first use");
    let kids = ov.cells(Dim::Volume).to_vec();
    assert_eq!(kids.len(), 2);
    assert!(kids.iter().all(|k| k.kind() == EntityKind::PartitionCell));
    // The decomposition's persisted cells resolve in THE layer's overlay, and the model ids
    // were adopted by the transaction.
    let cut = l.model().decomposition().split_of(PieceRef::Whole(vol)).unwrap().cut;
    let cc = l.model().decomposition().cut_cells(cut).unwrap();
    assert!(cc.faces.iter().all(|f| l.topo().is_alive(*f) && !l.host().is_alive(*f)));
    assert!(kids.iter().all(|k| delta.created.contains(k)));
    // Every virtual id the split minted was adopted: in the delta and in the entity table.
    let virt: Vec<EntityId> = cc.faces.iter().chain(&cc.edges).chain(&cc.vertices).copied().collect();
    assert!(!virt.is_empty());
    for id in &virt {
        assert!(delta.created.contains(id), "{id:?} not in delta");
        assert!(s.state().contains(*id), "{id:?} not in state");
    }
    // topo() sees the pieces, the bare host still has the one volume.
    assert_eq!(l.topo().cells(Dim::Volume).len(), 2);
    assert_eq!(l.host().cells(Dim::Volume), &[vol]);

    // A refused second split leaves overlay and model exactly as they were.
    let before = s.state().hash();
    let bad = run(&mut s, &SplitRealized { volume: vol, x: 1.5 });
    assert!(bad.is_err());
    assert_eq!(s.state().hash(), before);
}
