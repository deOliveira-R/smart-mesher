//! ACR-3 acceptance: two op sketches written against the layer API (`model.create_brick` and a
//! `mesh.run`-shaped op), then the orchestrator tests that gate freezing the crate.
//!
//! The sketches are what W2's sm-model and sm-mesher bodies look like. The mesh sketch drives
//! the real `Driver` with the stub schemes in `sm_scheme_api::stubs`.

use sm_scheme_api::stubs as stub_schemes;

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_layer_api::{
    CoreLayers, LayerError, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, TopoHost,
    commit_topology, insert_mesh, model_txn, retire_mesh, topo_txn,
};
use sm_mesh_api::{MeshEdit, MeshRead, PointIx, SerialAlloc};
use sm_model_api::Domain;
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Hash256, Layer, Meta, Op, OpError, QuietControl, Registry,
    Services, Session, TypedOp,
};
use sm_scheme_api::{
    Driver, Firmness, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver,
    SchemeChoice, Sequential, SizingSpec, WholeFaces,
};
use sm_topo_api::CellComplex;
use sm_topo_api::fixtures::brick;

// ------------------------------------------------------------------ sketch 1: model.create_brick

/// `model.create_brick`: a box with roles body / volume / faces / edges / vertices.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct CreateBrick {
    origin: [f64; 3],
    size: [f64; 3],
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "model.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if !self.size.iter().all(|s| s.is_finite() && *s > 0.0) {
            return Err(OpError::Invalid("brick sizes must be positive".into()));
        }
        let [x, y, z] = self.origin;
        let (ids, txn) = topo_txn(ctx, |topo, _model| {
            brick(topo, Point3::new(x, y, z), self.size).or_op()
        })?;
        commit_topology(
            ctx,
            &txn,
            &[
                ("body", &[ids.body]),
                ("volume", &[ids.volume]),
                ("faces", &ids.faces),
                ("edges", &ids.edges),
                ("vertices", &ids.vertices),
            ],
        )
    }
}

// ------------------------------------------------------------------ sketch 2: mesh.run

/// A `mesh.run`-shaped op: reads `ModelLayer`, meshes one volume with the driver, adopts the
/// minted ids and inserts the entry.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MeshRun {
    volume: EntityId,
    size: f64,
}

impl TypedOp for MeshRun {
    const NAME: &'static str = "mesh.run";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.volume, EntityKind::TopoVolume)?;
        // Borrows of ApplyCtx are exclusive: clone what we read (Arc clones), then write.
        let model = ctx.layer::<ModelLayer>()?.clone();
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();

        let registry = stub_schemes::registry();
        let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
        let driver = Driver::new(&registry, &solver, &picker);
        let targets = [self.volume];
        let input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets);
        let mut intent = MeshIntent::default();
        intent.defaults.sizing = Some(SizingSpec::Constant { size: self.size });
        intent.set_scheme(self.volume, SchemeChoice::new("mapvol", Firmness::Soft));

        let mut alloc = alloc0.clone(); // commit it only on Ok
        let out = driver
            .mesh(
                &input,
                &intent,
                &Sequential,
                sm_mesh::MeshBuilder::new(),
                &mut alloc,
            )
            .map_err(|e| OpError::Failed(e.to_string()))?;
        let mesh = out.finished.mesh;
        let entry = MeshEntry::new(
            Box::new(mesh),
            targets.to_vec(),
            Some(out.plan),
            model.hash_quanta(),
        );
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
    }
}

/// `mesh.delete`: removes a mesh and retires all its ids.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MeshDelete {
    mesh: EntityId,
}

impl TypedOp for MeshDelete {
    const NAME: &'static str = "mesh.delete";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        retire_mesh(ctx, self.mesh)
    }
}

/// Retires one id, then (optionally) tries `retire_mesh` and ignores its error.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct CellThenMesh {
    cell: EntityId,
    mesh: EntityId,
    try_mesh: bool,
}

impl TypedOp for CellThenMesh {
    const NAME: &'static str = "test.cell_then_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.retire(self.cell)?;
        if self.try_mesh {
            assert!(retire_mesh(ctx, self.mesh).is_err());
        }
        Ok(())
    }
}

/// Kills a volume, then lists it in a role (must be refused).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct KillThenRole {
    volume: EntityId,
}

impl TypedOp for KillThenRole {
    const NAME: &'static str = "test.kill_then_role";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (_, txn) = topo_txn(ctx, |topo, _| topo.kill_cascade(self.volume).or_op())?;
        commit_topology(ctx, &txn, &[("volumes", &[self.volume])])
    }
}

/// Inserts a generated hex block (no selection): the large-mesh path.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct HexBlock {
    n: [u32; 3],
}

impl TypedOp for HexBlock {
    const NAME: &'static str = "test.hex_block";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let q = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let mesh = sm_mesh::generate::hex_block_bulk(self.n, [1.0; 3], &mut alloc)
            .map_err(|e| OpError::Failed(e.to_string()))?
            .mesh;
        insert_mesh(
            ctx,
            MeshEntry::new(Box::new(mesh), vec![], None, q),
            &before,
            alloc,
            "mesh",
        )
    }
}

/// Builds a hex block, optionally over-mints ids, tries `insert_mesh` and swallows the error.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct OverMint {
    extra: usize,
    /// Pass an allocator that is not the layer's as `before`.
    stale_before: bool,
}

impl TypedOp for OverMint {
    const NAME: &'static str = "test.over_mint";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let q = ctx.layer::<ModelLayer>()?.hash_quanta();
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc = before.clone();
        let mesh = sm_mesh::generate::hex_block_bulk([2, 2, 2], [1.0; 3], &mut alloc)
            .map_err(|e| OpError::Failed(e.to_string()))?
            .mesh;
        if self.extra > 0 {
            alloc
                .take(EntityKind::MeshVertex, self.extra)
                .map_err(|e| OpError::Failed(e.to_string()))?;
        }
        let before = if self.stale_before {
            alloc.clone()
        } else {
            before
        };
        let r = insert_mesh(
            ctx,
            MeshEntry::new(Box::new(mesh), vec![], None, q),
            &before,
            alloc,
            "mesh",
        );
        let e = r.expect_err("must be refused");
        assert!(matches!(e, OpError::Failed(_)), "{e:?}");
        Ok(())
    }
}

/// `model.copy`-shaped: makes a new brick and reports `[new, old]` in one role.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct BrickAndOld {
    old: EntityId,
}

impl TypedOp for BrickAndOld {
    const NAME: &'static str = "test.brick_and_old";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| {
            brick(topo, Point3::new(9.0, 0.0, 0.0), [1.0; 3]).or_op()
        })?;
        commit_topology(ctx, &txn, &[("volumes", &[ids.volume, self.old])])
    }
}

/// `fem.block_remove`-shaped: adds a volume color in role `block`, or removes a color.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Block {
    /// Remove this color instead of adding one.
    remove: Option<EntityId>,
}

impl TypedOp for Block {
    const NAME: &'static str = "fem.block";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        match self.remove {
            None => model_txn(
                ctx,
                |model, _| model.add_color(7, "block", Domain::Volume).or_op(),
                |c, _| vec![("block", vec![*c])],
            )
            .map(|_| ()),
            Some(c) => model_txn(ctx, |model, _| model.remove_color(c).or_op(), |_, _| vec![]),
        }
    }
}

/// `model.move`-shaped: translates every vertex and reports the pre-existing volume in `volumes`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct MoveVolume {
    volume: EntityId,
    dx: f64,
    /// Also list a retired id (must be refused).
    #[serde(default)]
    bad_role: bool,
}

impl TypedOp for MoveVolume {
    const NAME: &'static str = "model.move_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.volume, EntityKind::TopoVolume)?;
        let x = Transform::translation(Vector3::new(self.dx, 0.0, 0.0));
        let ((), txn) = topo_txn(ctx, |topo, model| {
            let verts: Vec<EntityId> = topo.cells(sm_topo_api::Dim::Vertex).to_vec();
            for v in verts {
                if let sm_topo_api::Carrier::Point(p) = topo.carrier(v).clone() {
                    topo.set_vertex_point(v, x.apply_point(p)).or_op()?;
                }
            }
            model
                .apply_transform(&x, &[self.volume], topo.as_view())
                .or_op()
        })?;
        let ghost = id(EntityKind::TopoVolume, 99);
        if self.bad_role {
            return commit_topology(ctx, &txn, &[("volumes", &[self.volume, ghost])]);
        }
        commit_topology(ctx, &txn, &[("volumes", &[self.volume])])
    }
}

// ------------------------------------------------------------------ model.add_volume_color

/// Colors a volume with a new color in one model transaction. `fail_after_mint` returns an error
/// after the color was minted; `swallow` makes the op ignore that error and succeed.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct AddVolumeColor {
    volume: EntityId,
    label: u32,
    name: String,
    #[serde(default)]
    fail_after_mint: bool,
    #[serde(default)]
    swallow: bool,
    #[serde(default)]
    bad_role: bool,
}

impl TypedOp for AddVolumeColor {
    const NAME: &'static str = "model.add_volume_color";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.volume, EntityKind::TopoVolume)?;
        let r = model_txn(
            ctx,
            |model, topo| {
                let c = model
                    .add_color(self.label, &self.name, Domain::Volume)
                    .or_op()?;
                if self.fail_after_mint {
                    return Err(OpError::Invalid("boom after mint".into()));
                }
                if !topo.is_alive(self.volume) {
                    return Err(OpError::UnknownEntity(self.volume));
                }
                model.color_volumes(c, &[self.volume]).or_op()?;
                Ok(c)
            },
            |c, _new| {
                if self.bad_role {
                    // An id that is neither created by the transaction nor live.
                    vec![("color", vec![*c, id(EntityKind::TopoVolume, 99)])]
                } else {
                    vec![("color", vec![*c])]
                }
            },
        );
        match r {
            Err(_) if self.swallow => Ok(()),
            other => other.map(|_| ()),
        }
    }
}

// ------------------------------------------------------------------ webcut / delete

/// Cuts the first brick (ids 1..) with a horizontal plane: splits cells, so retirement, lineage
/// and successors all take part.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Webcut {
    frac: f64,
}

impl TypedOp for Webcut {
    const NAME: &'static str = "model.webcut_first";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        // The fixture mints a brick's cells in array order, so ascending ids rebuild its handle.
        let ids = |k, n: u64| -> Vec<EntityId> { (1..=n).map(|s| id(k, s)).collect() };
        let b = sm_topo_api::fixtures::BrickIds {
            min: Point3::new(0.0, 0.0, 0.0),
            size: [1.0, 2.0, 3.0],
            vertices: ids(EntityKind::TopoVertex, 8).try_into().expect("8"),
            edges: ids(EntityKind::TopoEdge, 12).try_into().expect("12"),
            faces: ids(EntityKind::TopoFace, 6).try_into().expect("6"),
            volume: id(EntityKind::TopoVolume, 1),
            body: id(EntityKind::TopoBody, 1),
        };
        let (cut, txn) = topo_txn(ctx, |topo, _| {
            sm_topo_api::fixtures::webcut_brick(topo, &b, self.frac).or_op()
        })?;
        commit_topology(ctx, &txn, &[("volumes", &cut.volumes), ("cap", &[cut.cap])])
    }
}

/// Deletes a volume and everything only it uses.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct DeleteVolume {
    volume: EntityId,
}

impl TypedOp for DeleteVolume {
    const NAME: &'static str = "model.delete_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.volume, EntityKind::TopoVolume)?;
        let (_, txn) = topo_txn(ctx, |topo, _| topo.kill_cascade(self.volume).or_op())?;
        commit_topology(ctx, &txn, &[])
    }
}

// ------------------------------------------------------------------ transaction misuse

/// Exercises the failure paths of `topo_txn` / `commit_topology`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Misuse {
    mode: String,
}

impl TypedOp for Misuse {
    const NAME: &'static str = "test.misuse";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        match self.mode.as_str() {
            // A closure error aborts the topology operation; the host stays usable.
            "rollback" => {
                let r = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()?;
                    Err::<(), _>(OpError::Invalid("after building".into()))
                });
                assert!(r.is_err());
                let (ids, txn) = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()
                })?;
                assert_eq!(
                    ids.volume,
                    id(EntityKind::TopoVolume, 1),
                    "the aborted cells were rolled back"
                );
                commit_topology(ctx, &txn, &[])
            }
            // A transaction must be committed before the next one starts.
            "double" => {
                let (_, _txn) = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()
                })?;
                topo_txn(ctx, |_, _| Ok(())).map(|_| ())
            }
            // ... and a model transaction is refused while one is pending.
            "model_while_pending" => {
                let (_, _txn) = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()
                })?;
                model_txn(ctx, |_, _| Ok(()), |_, _| vec![])
            }
            // Committing twice is refused.
            "commit_twice" => {
                let (_, txn) = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()
                })?;
                commit_topology(ctx, &txn, &[])?;
                commit_topology(ctx, &txn, &[])
            }
            // A role may not list an id twice or name a foreign id.
            "dup_role" => {
                let (ids, txn) = topo_txn(ctx, |topo, _| {
                    brick(topo, Point3::new(0.0, 0.0, 0.0), [1.0; 3]).or_op()
                })?;
                commit_topology(ctx, &txn, &[("a", &[ids.volume]), ("b", &[ids.volume])])
            }
            _ => Ok(()),
        }
    }
}

// ------------------------------------------------------------------ harness

type Factory = fn() -> Box<dyn TopoHost>;

fn cell_complex() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

fn topo_store() -> Box<dyn TopoHost> {
    Box::new(sm_topo::TopoStore::new())
}

fn engine(factory: Factory, cache: DerivedCache) -> Engine {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).expect("core layers");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<MeshRun>().expect("mesh.run");
    reg.register::<AddVolumeColor>().expect("color");
    reg.register::<Misuse>().expect("misuse");
    reg.register::<Webcut>().expect("webcut");
    reg.register::<DeleteVolume>().expect("delete");
    reg.register::<MeshDelete>().expect("mesh.delete");
    reg.register::<HexBlock>().expect("hex");
    reg.register::<BrickAndOld>().expect("brick_and_old");
    reg.register::<OverMint>().expect("over_mint");
    reg.register::<CellThenMesh>().expect("ctm");
    reg.register::<KillThenRole>().expect("ktr");
    reg.register::<Block>().expect("block");
    reg.register::<MoveVolume>().expect("move");
    Engine::new(Arc::new(reg), Services::new(), Arc::new(cache)).expect("engine")
}

fn meta() -> Meta {
    Meta::new("test", "", "")
}

fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

fn brick_op() -> CreateBrick {
    CreateBrick {
        origin: [0.0, 0.0, 0.0],
        size: [1.0, 2.0, 3.0],
    }
}

fn run(
    s: &mut Session,
    op: &dyn Op,
) -> Result<(Hash256, sm_op_api::Delta), sm_op_api::TimelineError> {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control())
}

fn range(kind: EntityKind, n: u64) -> Vec<EntityId> {
    (1..=n).map(|s| id(kind, s)).collect()
}

// ------------------------------------------------------------------ test 1: create_brick

fn brick_session(factory: Factory) -> (Session, sm_op_api::Delta) {
    let mut s = Session::new(engine(factory, DerivedCache::default()));
    let (_, delta) = run(&mut s, &brick_op()).expect("brick");
    (s, delta)
}

#[test]
fn brick_delta_is_exact_and_both_hosts_hash_equal() {
    let (cc, d_cc) = brick_session(cell_complex);
    let (ts, d_ts) = brick_session(topo_store);

    let mut want = Vec::new();
    for (kind, n) in [
        (EntityKind::TopoVertex, 8),
        (EntityKind::TopoEdge, 12),
        (EntityKind::TopoFace, 6),
        (EntityKind::TopoVolume, 1),
        (EntityKind::TopoBody, 1),
    ] {
        want.extend(range(kind, n));
    }
    assert_eq!(
        d_cc.created, want,
        "Delta.created is exactly the 28 new cells, ascending"
    );
    assert_eq!(d_ts.created, want);
    assert!(d_cc.removed.is_empty());
    assert_eq!(d_cc.touched_layers, vec!["model".to_owned()]);

    // Roles: ids of a role come out ascending.
    assert_eq!(d_cc.outputs["body"], range(EntityKind::TopoBody, 1));
    assert_eq!(d_cc.outputs["volume"], range(EntityKind::TopoVolume, 1));
    assert_eq!(d_cc.outputs["faces"], range(EntityKind::TopoFace, 6));
    assert_eq!(d_cc.outputs["edges"], range(EntityKind::TopoEdge, 12));
    assert_eq!(d_cc.outputs["vertices"], range(EntityKind::TopoVertex, 8));
    assert_eq!(d_cc.outputs, d_ts.outputs);

    for id in &want {
        assert!(cc.state().contains(*id));
    }
    assert_eq!(cc.state().entity_count(), 28);

    // Same ops on the reference and the production store: one state hash.
    assert_eq!(
        cc.state().hash(),
        ts.state().hash(),
        "CellComplex and TopoStore must give the same state hash"
    );
    assert_eq!(
        cc.state().layer_hash("model").expect("hash"),
        ts.state().layer_hash("model").expect("hash")
    );
}

#[test]
fn invalid_brick_fails_and_leaves_the_state_alone() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::default()));
    let before = s.state().hash();
    let bad = CreateBrick {
        origin: [0.0; 3],
        size: [1.0, -1.0, 1.0],
    };
    assert!(run(&mut s, &bad).is_err());
    assert_eq!(s.state().hash(), before);
}

#[test]
fn model_layer_is_empty_until_first_op() {
    let s = Session::new(engine(topo_store, DerivedCache::default()));
    let empty = s.state().hash();
    let (s2, _) = brick_session(topo_store);
    assert_ne!(empty, s2.state().hash());
    let l = s.state().layer::<ModelLayer>().expect("layer");
    assert_eq!(l.ops_run(), 0);
    assert!(sm_op_api::LayerType::is_empty(l));
}

// ------------------------------------------------------------------ test 2: mesh digest

#[test]
fn mesh_digest_is_quantized_but_the_exact_hash_is_not() {
    let model = ModelLayer::default();
    let quanta = model.hash_quanta();
    let build = |shift: f64| {
        let mut alloc = SerialAlloc::new();
        let mut m = sm_mesh::generate::hex_block_bulk([2, 2, 2], [0.5; 3], &mut alloc)
            .expect("hex")
            .mesh;
        if shift != 0.0 {
            // Vertex 0 sits at the origin, far from any rounding boundary.
            m.set_point(PointIx(0), Point3::new(shift, 0.0, 0.0))
                .expect("set_point");
        }
        let mid = m.mesh_id();
        let mut layer = MeshLayer::default();
        let entry = MeshEntry::new(
            Box::new(m),
            vec![id(EntityKind::TopoVolume, 1)],
            None,
            quanta,
        );
        layer.insert(mid, entry, alloc).expect("insert");
        layer
    };
    let (a, b, c) = (build(0.0), build(1e-15), build(1e-3));
    assert_eq!(
        Layer::content_hash(&a),
        Layer::content_hash(&b),
        "1e-15 is below the quantum"
    );
    assert_ne!(
        Layer::exact_hash(&a),
        Layer::exact_hash(&b),
        "the exact hash sees every bit"
    );
    assert_ne!(
        Layer::content_hash(&a),
        Layer::content_hash(&c),
        "1e-3 is above the quantum"
    );
    assert!(!sm_op_api::LayerType::is_empty(&a));
    assert!(sm_op_api::LayerType::is_empty(&MeshLayer::default()));
}

#[test]
fn mesh_layer_insert_validates() {
    let mut alloc = SerialAlloc::new();
    let m = sm_mesh::generate::hex_block_bulk([1, 1, 1], [1.0; 3], &mut alloc)
        .expect("hex")
        .mesh;
    let mid = m.mesh_id();
    let q = ModelLayer::default().hash_quanta();
    let mk = || MeshEntry::new(Box::new(m.clone()), vec![], None, q);
    let mut layer = MeshLayer::default();
    assert!(
        layer
            .insert(id(EntityKind::TopoVolume, 1), mk(), alloc.clone())
            .is_err(),
        "not a Mesh id"
    );
    assert!(
        layer
            .insert(id(EntityKind::Mesh, 99), mk(), alloc.clone())
            .is_err(),
        "id differs from the mesh"
    );
    layer.insert(mid, mk(), alloc.clone()).expect("ok");
    assert!(layer.insert(mid, mk(), alloc.clone()).is_err(), "duplicate");
    let mut m2 = sm_mesh::generate::hex_block_bulk([1, 1, 1], [1.0; 3], &mut alloc)
        .expect("hex")
        .mesh;
    let _ = &mut m2;
    assert!(
        layer
            .insert(
                m2.mesh_id(),
                MeshEntry::new(Box::new(m2.clone()), vec![], None, q),
                SerialAlloc::new()
            )
            .is_err(),
        "allocator may not move backwards"
    );
    assert_eq!(layer.mesh_of(id(EntityKind::TopoVolume, 1)), None);
}

// ------------------------------------------------------------------ mesh.run end to end

fn meshed_session(factory: Factory) -> (Session, sm_op_api::Delta) {
    let mut s = Session::new(engine(factory, DerivedCache::default()));
    run(&mut s, &brick_op()).expect("brick");
    let vol = id(EntityKind::TopoVolume, 1);
    let (_, d) = run(
        &mut s,
        &MeshRun {
            volume: vol,
            size: 0.5,
        },
    )
    .expect("mesh.run");
    (s, d)
}

#[test]
fn mesh_run_adopts_ids_in_allocation_order_and_marks_stale_on_retire() {
    let (s, d) = meshed_session(topo_store);
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    assert_eq!(layer.len(), 1);
    let (mid, e) = layer.meshes().next().expect("one");
    assert_eq!(d.outputs["mesh"], vec![mid]);
    assert_eq!(
        d.created.first(),
        Some(&mid),
        "the Mesh id is adopted first"
    );
    assert!(
        d.created.windows(2).all(|w| w[0] < w[1]),
        "ascending (kind, serial)"
    );
    let m = e.mesh();
    // Size 0.5 on a 1 x 2 x 3 brick: 2 x 4 x 6 hexahedra.
    assert_eq!((m.cell_count(), m.vertex_count()), (48, 3 * 5 * 7));
    assert_eq!(
        d.created.len(),
        1 + m.vertex_count() + m.face_count() + m.cell_count() + extra(&d, m)
    );
    let of = |k: EntityKind| d.created.iter().filter(|i| i.kind() == k).count();
    assert_eq!(of(EntityKind::MeshCell), m.cell_count());
    assert_eq!(of(EntityKind::MeshFace), m.face_count());
    assert_eq!(of(EntityKind::MeshVertex), m.vertex_count());
    assert_eq!(of(EntityKind::Mesh), 1);
    assert!(!e.stale);
    assert_eq!(layer.mesh_of(id(EntityKind::TopoVolume, 1)), Some(mid));
    assert!(s.state().contains(mid));
    // Entity table and allocator agree: the next id of every mesh kind is the allocator's.
    for (kind, next) in layer.alloc().state() {
        assert_eq!(
            s.state().peek_next(kind).expect("next").serial(),
            next,
            "{kind:?}"
        );
    }
    assert_eq!(d.touched_layers, vec!["meshes".to_owned()]);
}

/// Ids of kinds other than the four counted above (edges, sets).
fn extra(d: &sm_op_api::Delta, m: &dyn MeshRead) -> usize {
    d.created.len() - 1 - m.vertex_count() - m.face_count() - m.cell_count()
}

#[test]
fn retiring_the_selection_marks_the_mesh_stale() {
    use sm_op_api::LayerType as L;
    let (s, _) = meshed_session(cell_complex);
    let mut layer = s.state().layer::<MeshLayer>().expect("layer").clone();
    let vol = id(EntityKind::TopoVolume, 1);
    assert!(L::references_any(&layer, &[vol]));
    let before = Layer::content_hash(&layer);
    L::on_retired(&mut layer, &[vol]);
    let (_, e) = layer.meshes().next().expect("kept");
    assert!(e.stale);
    assert_ne!(Layer::content_hash(&layer), before, "stale is hashed");
    assert!(!L::references_any(&layer, &[vol]));
}

// ------------------------------------------------------------------ test 3: model_txn

#[test]
fn model_txn_adds_a_volume_color_atomically() {
    let (mut s, _) = brick_session(cell_complex);
    let vol = id(EntityKind::TopoVolume, 1);
    let good = AddVolumeColor {
        volume: vol,
        label: 1,
        name: "fuel".into(),
        fail_after_mint: false,
        swallow: false,
        bad_role: false,
    };
    let (_, d) = run(&mut s, &good).expect("color");
    let color = id(EntityKind::Color, 1);
    assert_eq!(d.created, vec![color]);
    assert_eq!(d.outputs["color"], vec![color]);
    let l = s.state().layer::<ModelLayer>().expect("layer");
    assert_eq!(l.model().volume_color(vol), Some(color));
    assert_eq!(
        l.ops_run(),
        1,
        "a model transaction runs no topology operation"
    );

    // An Err after the mint fails the op and changes nothing.
    let hash = s.state().hash();
    let model_hash = s.state().layer_hash("model").expect("hash");
    let bad = |label, name: &str, swallow, bad_role, fail| AddVolumeColor {
        volume: vol,
        label,
        name: name.into(),
        fail_after_mint: fail,
        swallow,
        bad_role,
    };
    assert!(run(&mut s, &bad(2, "clad", false, false, true)).is_err());
    assert_eq!(s.state().hash(), hash);
    // A role naming an id that is neither created nor live is rejected before anything is adopted.
    assert!(run(&mut s, &bad(2, "clad", false, true, false)).is_err());
    assert_eq!(s.state().hash(), hash);
    // An op that swallows the error still sees the model restored and nothing adopted.
    let (_, d) = run(&mut s, &bad(2, "clad", true, false, true)).expect("swallowed");
    assert!(d.created.is_empty() && d.outputs.is_empty());
    assert_eq!(s.state().layer_hash("model").expect("hash"), model_hash);
    assert_eq!(
        s.state().peek_next(EntityKind::Color),
        Some(id(EntityKind::Color, 2))
    );
    let swallowed = bad(2, "clad", true, true, false);
    let (_, d) = run(&mut s, &swallowed).expect("swallowed bad role");
    assert!(d.created.is_empty());
    assert_eq!(s.state().layer_hash("model").expect("hash"), model_hash);
    // The id space is intact: the next good color gets Color#2.
    let (_, d) = run(&mut s, &bad(2, "clad", false, false, false)).expect("second color");
    assert_eq!(d.created, vec![id(EntityKind::Color, 2)]);
}

#[test]
fn transaction_misuse_is_an_error_and_never_corrupts_the_state() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = Session::new(engine(factory, DerivedCache::default()));
        let empty = s.state().hash();
        for mode in ["double", "model_while_pending", "commit_twice", "dup_role"] {
            let r = run(&mut s, &Misuse { mode: mode.into() });
            assert!(r.is_err(), "{mode}");
            assert_eq!(s.state().hash(), empty, "{mode}");
        }
        let (_, d) = run(
            &mut s,
            &Misuse {
                mode: "rollback".into(),
            },
        )
        .expect("rollback");
        assert_eq!(d.created.len(), 28);
        let l = s.state().layer::<ModelLayer>().expect("layer");
        assert_eq!(l.ops_run(), 1, "only the committed transaction counts");
    }
}

#[test]
fn splitting_and_deleting_retire_record_successors_and_rekey_the_model() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = Session::new(engine(factory, DerivedCache::disabled()));
        let vol = id(EntityKind::TopoVolume, 1);
        run(&mut s, &brick_op()).expect("brick");
        let color = AddVolumeColor {
            volume: vol,
            label: 1,
            name: "fuel".into(),
            fail_after_mint: false,
            swallow: false,
            bad_role: false,
        };
        run(&mut s, &color).expect("color");
        run(
            &mut s,
            &MeshRun {
                volume: vol,
                size: 1.0,
            },
        )
        .expect("mesh");

        let (h_cut, d) = run(&mut s, &Webcut { frac: 0.5 }).expect("webcut");
        // The volume was split: retired, with two successors, and the mesh on it is stale.
        assert!(d.removed.contains(&vol));
        let kids = d.outputs["volumes"].clone();
        assert_eq!(kids.len(), 2);
        assert_eq!(d.successors[&vol], kids);
        assert!(d.created.windows(2).all(|w| w[0] < w[1]));
        assert!(!s.state().contains(vol));
        assert!(
            d.touched_layers.contains(&"meshes".to_owned()),
            "on_retired reached the mesh layer"
        );
        let layer = s.state().layer::<MeshLayer>().expect("layer");
        assert!(layer.meshes().all(|(_, e)| e.stale));
        // The model's volume color followed the split.
        let m = s.state().layer::<ModelLayer>().expect("layer").model();
        assert_eq!(m.volume_color(kids[0]), Some(id(EntityKind::Color, 1)));
        assert_eq!(m.volume_color(kids[1]), Some(id(EntityKind::Color, 1)));
        assert_eq!(m.volume_color(vol), None);

        // Delete one half.
        let (h_del, d) = run(&mut s, &DeleteVolume { volume: kids[0] }).expect("delete");
        assert!(d.removed.contains(&kids[0]));
        assert!(d.created.is_empty());
        let m = s.state().layer::<ModelLayer>().expect("layer").model();
        assert_eq!(m.volume_color(kids[0]), None);
        assert_eq!(m.volume_color(kids[1]), Some(id(EntityKind::Color, 1)));

        // Replay from the root, with no state cache, reproduces every node hash.
        let full = s.state().hash();
        let q = QuietControl::new();
        assert_eq!(
            s.state_at_from_root(h_del, &q.control())
                .expect("replay")
                .hash(),
            full
        );
        s.undo().expect("undo");
        assert_eq!(
            s.state().hash(),
            s.state_at_from_root(h_cut, &q.control())
                .expect("replay")
                .hash()
        );
        s.redo().expect("redo");
        assert_eq!(s.state().hash(), full);
    }
}

// ------------------------------------------------------------------ test 4: undo, redo, replay

#[test]
fn undo_redo_and_cold_replay_do_not_diverge() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = Session::new(engine(factory, DerivedCache::disabled()));
        let vol = id(EntityKind::TopoVolume, 1);
        run(&mut s, &brick_op()).expect("brick");
        run(
            &mut s,
            &CreateBrick {
                origin: [2.0, 0.0, 0.0],
                size: [1.0; 3],
            },
        )
        .expect("brick 2");
        run(
            &mut s,
            &MeshRun {
                volume: vol,
                size: 0.5,
            },
        )
        .expect("mesh");
        let color = AddVolumeColor {
            volume: vol,
            label: 1,
            name: "fuel".into(),
            fail_after_mint: false,
            swallow: false,
            bad_role: false,
        };
        let (head, _) = run(&mut s, &color).expect("color");
        let full = s.state().hash();

        s.undo().expect("undo");
        s.undo().expect("undo");
        assert_ne!(s.state().hash(), full);
        s.redo().expect("redo");
        s.redo().expect("redo");
        assert_eq!(s.state().hash(), full);

        let q = QuietControl::new();
        let cold = s
            .state_at_from_root(head, &q.control())
            .expect("cold replay verifies every node hash");
        assert_eq!(cold.hash(), full);
        // Undo to the root and replay forward through the cache-free path.
        for _ in 0..4 {
            s.undo().expect("undo");
        }
        assert_eq!(s.state().entity_count(), 0);
    }
}

// ------------------------------------------------------------------ test 5: determinism

fn flow_hash(factory: Factory) -> Hash256 {
    let mut s = Session::new(engine(factory, DerivedCache::default()));
    let vol = id(EntityKind::TopoVolume, 1);
    run(&mut s, &brick_op()).expect("brick");
    run(
        &mut s,
        &MeshRun {
            volume: vol,
            size: 0.5,
        },
    )
    .expect("mesh");
    let color = AddVolumeColor {
        volume: vol,
        label: 1,
        name: "fuel".into(),
        fail_after_mint: false,
        swallow: false,
        bad_role: false,
    };
    run(&mut s, &color).expect("color");
    s.state().hash()
}

#[test]
fn state_hash_is_deterministic_across_runs_and_threads() {
    for factory in [cell_complex as Factory, topo_store] {
        let h = sm_testkit::determinism::check(2, 4, |_threads| flow_hash(factory))
            .expect("deterministic");
        assert_eq!(h, flow_hash(factory));
    }
    // The two hosts agree on the whole flow too.
    assert_eq!(flow_hash(cell_complex), flow_hash(topo_store));
}

#[test]
fn core_layers_register_once_and_conflict_on_a_second_install() {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("first");
    assert!(reg.install(&CoreLayers::default()).is_err());
    let st = reg.new_state();
    assert!(st.layer::<ModelLayer>().is_some() && st.layer::<MeshLayer>().is_some());
    assert_eq!(reg.layer_order().expect("order"), vec!["model", "meshes"]);
}

// ------------------------------------------------------------------ polish: delete, remove, move

#[test]
fn mesh_delete_restores_counts_and_replays_clean() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = Session::new(engine(factory, DerivedCache::disabled()));
        let vol = id(EntityKind::TopoVolume, 1);
        run(&mut s, &brick_op()).expect("brick");
        let pre_count = s.state().entity_count();
        let pre_len = s.state().layer::<MeshLayer>().expect("layer").len();
        let (_, d) = run(
            &mut s,
            &MeshRun {
                volume: vol,
                size: 0.5,
            },
        )
        .expect("mesh");
        let mid = d.outputs["mesh"][0];
        assert!(s.state().entity_count() > pre_count);
        assert_eq!(
            s.state().layer::<MeshLayer>().expect("layer").len(),
            pre_len + 1
        );

        let (h_del, d) = run(&mut s, &MeshDelete { mesh: mid }).expect("mesh.delete");
        assert_eq!(s.state().entity_count(), pre_count, "all mesh ids retired");
        assert_eq!(
            s.state().layer::<MeshLayer>().expect("layer").len(),
            pre_len
        );
        assert!(!s.state().contains(mid));
        assert!(
            d.removed.windows(2).all(|w| w[0] < w[1]),
            "retired in ascending order"
        );
        assert!(d.removed.contains(&mid));
        // Deleting again fails.
        assert!(run(&mut s, &MeshDelete { mesh: mid }).is_err());

        let full = s.state().hash();
        let q = QuietControl::new();
        assert_eq!(
            s.state_at_from_root(h_del, &q.control())
                .expect("cold replay")
                .hash(),
            full
        );
        s.undo().expect("undo");
        assert_eq!(
            s.state().layer::<MeshLayer>().expect("layer").len(),
            pre_len + 1
        );
        s.redo().expect("redo");
        assert_eq!(s.state().hash(), full);
    }
}

#[test]
fn large_mesh_insert_and_retire_complete() {
    // 47 x 47 x 46 = 101 614 hexahedra: exercises the sorted-id paths of on_retired.
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    let (_, d) = run(&mut s, &HexBlock { n: [47, 47, 46] }).expect("hex");
    let mid = d.outputs["mesh"][0];
    assert!(d.created.len() > 100_000);
    let (_, d) = run(&mut s, &MeshDelete { mesh: mid }).expect("delete");
    assert!(d.removed.len() > 100_000);
    assert_eq!(s.state().entity_count(), 0);
}

#[test]
fn model_txn_retires_removed_records() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    run(&mut s, &brick_op()).expect("brick");
    let base = s.state().entity_count();
    let (_, d) = run(&mut s, &Block { remove: None }).expect("add");
    let color = d.outputs["block"][0];
    assert!(s.state().contains(color));
    assert_eq!(s.state().entity_count(), base + 1);
    let (h, d) = run(
        &mut s,
        &Block {
            remove: Some(color),
        },
    )
    .expect("remove");
    assert_eq!(d.removed, vec![color]);
    assert!(!s.state().contains(color));
    assert_eq!(s.state().entity_count(), base);
    assert!(
        s.state()
            .layer::<ModelLayer>()
            .expect("layer")
            .model()
            .color(color)
            .is_none()
    );
    // Serials are never reused.
    let (_, d) = run(&mut s, &Block { remove: None }).expect("add again");
    assert_eq!(d.outputs["block"], vec![id(EntityKind::Color, 2)]);
    let full = s.state().hash();
    let q = QuietControl::new();
    s.undo().expect("undo");
    s.redo().expect("redo");
    assert_eq!(s.state().hash(), full);
    s.state_at_from_root(h, &q.control()).expect("cold replay");
    // Removing an unknown color fails and changes nothing.
    assert!(
        run(
            &mut s,
            &Block {
                remove: Some(color)
            }
        )
        .is_err()
    );
    assert_eq!(s.state().hash(), full);
}

#[test]
fn surviving_ids_can_be_reported_in_roles() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = Session::new(engine(factory, DerivedCache::disabled()));
        let vol = id(EntityKind::TopoVolume, 1);
        run(&mut s, &brick_op()).expect("brick");
        let before = s.state().hash();
        let (h, d) = run(
            &mut s,
            &MoveVolume {
                volume: vol,
                dx: 5.0,
                bad_role: false,
            },
        )
        .expect("move");
        assert_eq!(d.outputs["volumes"], vec![vol], "pre-existing id reported");
        assert!(d.created.is_empty() && d.removed.is_empty());
        assert_ne!(s.state().hash(), before, "geometry moved");
        // A role naming a dead id is refused and leaves the state alone.
        let now = s.state().hash();
        assert!(
            run(
                &mut s,
                &MoveVolume {
                    volume: vol,
                    dx: 1.0,
                    bad_role: true
                }
            )
            .is_err()
        );
        assert_eq!(s.state().hash(), now);
        let q = QuietControl::new();
        assert_eq!(
            s.state_at_from_root(h, &q.control())
                .expect("replay")
                .hash(),
            now
        );
    }
}

#[test]
fn roles_may_not_name_ids_the_same_transaction_retires() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    run(&mut s, &brick_op()).expect("brick");
    let h = s.state().hash();
    assert!(
        run(
            &mut s,
            &KillThenRole {
                volume: id(EntityKind::TopoVolume, 1)
            }
        )
        .is_err()
    );
    assert_eq!(s.state().hash(), h);
}

#[test]
fn retire_mesh_is_atomic_when_an_id_is_already_gone() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    run(&mut s, &brick_op()).expect("brick");
    let (_, d) = run(
        &mut s,
        &MeshRun {
            volume: id(EntityKind::TopoVolume, 1),
            size: 0.5,
        },
    )
    .expect("mesh");
    let mid = d.outputs["mesh"][0];
    let cell = d
        .created
        .iter()
        .copied()
        .find(|i| i.kind() == EntityKind::MeshCell)
        .expect("cell");
    // Control: retire only the cell.
    let mut c = Session::new(engine(cell_complex, DerivedCache::disabled()));
    run(&mut c, &brick_op()).expect("brick");
    run(
        &mut c,
        &MeshRun {
            volume: id(EntityKind::TopoVolume, 1),
            size: 0.5,
        },
    )
    .expect("mesh");
    run(
        &mut c,
        &CellThenMesh {
            cell,
            mesh: mid,
            try_mesh: false,
        },
    )
    .expect("control");
    // The failing retire_mesh must change nothing beyond the cell.
    run(
        &mut s,
        &CellThenMesh {
            cell,
            mesh: mid,
            try_mesh: true,
        },
    )
    .expect("op");
    assert_eq!(s.state().hash(), c.state().hash());
    assert!(s.state().contains(mid));
    assert_eq!(s.state().layer::<MeshLayer>().expect("layer").len(), 1);
}

#[test]
fn insert_mesh_refuses_over_minted_ids_and_stale_allocators() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    run(&mut s, &brick_op()).expect("brick");
    let h = s.state().hash();
    run(
        &mut s,
        &OverMint {
            extra: 3,
            stale_before: false,
        },
    )
    .expect("op swallows the error");
    assert_eq!(
        s.state().hash(),
        h,
        "over-minted mesh refused, nothing adopted"
    );
    run(
        &mut s,
        &OverMint {
            extra: 0,
            stale_before: true,
        },
    )
    .expect("op swallows the error");
    assert_eq!(
        s.state().hash(),
        h,
        "stale allocator refused, nothing adopted"
    );
    // A well-formed insert and delete still round-trips.
    let n = s.state().entity_count();
    let (_, d) = run(&mut s, &HexBlock { n: [2, 2, 2] }).expect("hex");
    run(
        &mut s,
        &MeshDelete {
            mesh: d.outputs["mesh"][0],
        },
    )
    .expect("delete");
    assert_eq!(s.state().entity_count(), n);
}

#[test]
fn roles_mixing_created_and_surviving_ids_stay_ascending() {
    let mut s = Session::new(engine(cell_complex, DerivedCache::disabled()));
    let old = id(EntityKind::TopoVolume, 1);
    run(&mut s, &brick_op()).expect("brick");
    let (_, d) = run(&mut s, &BrickAndOld { old }).expect("op");
    let v = &d.outputs["volumes"];
    assert_eq!(v.len(), 2);
    assert!(v[0] == old && v[0] < v[1], "ascending overall: {v:?}");
}

// ------------------------------------------------------------------ add_mesh_set / replace_mesh

/// Adds a cell set to the first mesh; `swallow` ignores a refused add and succeeds.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct AddSet {
    mesh: EntityId,
    name: String,
    cells: Vec<u32>,
    #[serde(default)]
    swallow: bool,
}

impl TypedOp for AddSet {
    const NAME: &'static str = "mesh.add_set_test";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let members = sm_mesh_api::SetMembers::Cells(
            self.cells.iter().map(|&c| sm_mesh_api::CellIx(c)).collect(),
        );
        match sm_layer_api::add_mesh_set(ctx, self.mesh, &self.name, members, "set") {
            Err(_) if self.swallow => Ok(()),
            other => other.map(|_| ()),
        }
    }
}

fn set_session() -> (Session, EntityId) {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(topo_store))
        .expect("core layers");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<MeshRun>().expect("mesh.run");
    reg.register::<AddSet>().expect("add_set");
    let e = Engine::new(
        Arc::new(reg),
        Services::new(),
        Arc::new(DerivedCache::disabled()),
    )
    .expect("engine");
    let mut s = Session::new(e);
    run(&mut s, &brick_op()).expect("brick");
    run(
        &mut s,
        &MeshRun {
            volume: id(EntityKind::TopoVolume, 1),
            size: 0.5,
        },
    )
    .expect("mesh.run");
    let mid = s
        .state()
        .layer::<MeshLayer>()
        .expect("layer")
        .meshes()
        .next()
        .expect("mesh")
        .0;
    (s, mid)
}

fn add_set_op(mid: EntityId, name: &str, cells: Vec<u32>, swallow: bool) -> AddSet {
    AddSet {
        mesh: mid,
        name: name.into(),
        cells,
        swallow,
    }
}

#[test]
fn add_mesh_set_mints_one_set_id_and_is_atomic() {
    let (mut s, mid) = set_session();
    let before_hash = s.state().hash();
    let (_, d) = run(&mut s, &add_set_op(mid, "flagged", vec![0, 3, 7], false)).expect("add set");
    let sets = &d.outputs["set"];
    assert_eq!(sets.len(), 1);
    assert_eq!(sets[0].kind(), EntityKind::MeshSet);
    assert_eq!(d.created, sets.clone(), "exactly the set id is created");
    assert_ne!(s.state().hash(), before_hash, "sets are inside the digest");
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let e = layer.get(mid).expect("entry");
    assert!(!e.stale);
    assert_eq!(e.mesh().sets().len(), 1);
    assert_eq!(e.mesh().sets()[0].id, sets[0]);
    for (kind, next) in layer.alloc().state() {
        assert_eq!(s.state().peek_next(kind).expect("next").serial(), next);
    }
    // A refused add (duplicate name, bad member) changes nothing, even when swallowed.
    let h = s.state().hash();
    run(&mut s, &add_set_op(mid, "flagged", vec![1], true)).expect("swallowed");
    assert_eq!(s.state().hash(), h);
    assert!(run(&mut s, &add_set_op(mid, "bad", vec![9999], false)).is_err());
    assert_eq!(s.state().hash(), h);
    let (_, d2) = run(&mut s, &add_set_op(mid, "second", vec![2], false)).expect("second");
    assert_eq!(d2.outputs["set"][0].serial(), sets[0].serial() + 1);
}

#[test]
fn replace_mesh_checks_entry_id_and_allocator() {
    let (mut s, mid) = set_session();
    // A second mesh, whose host carries another Mesh id.
    run(
        &mut s,
        &MeshRun {
            volume: id(EntityKind::TopoVolume, 1),
            size: 0.5,
        },
    )
    .expect("second mesh");
    let layer = s.state().layer::<MeshLayer>().expect("layer").clone();
    let other = layer
        .meshes()
        .map(|(i, _)| i)
        .find(|i| *i != mid)
        .expect("second mesh id");
    let host = layer.get(mid).expect("entry").mesh.clone();
    let other_host = layer.get(other).expect("entry").mesh.clone();
    let before = layer.clone();
    let unchanged = |l: &MeshLayer| {
        assert!(l.alloc().state().eq(before.alloc().state()));
        assert_eq!(l.meshes().count(), before.meshes().count());
        for (i, e) in before.meshes() {
            assert!(Arc::ptr_eq(&l.get(i).expect("entry").mesh, &e.mesh));
        }
    };
    let mut l = layer.clone();
    assert!(
        l.replace_mesh(mid, host.clone(), layer.alloc().clone())
            .is_ok()
    );
    unchanged(&l);
    assert!(
        matches!(
            l.replace_mesh(
                id(EntityKind::Mesh, 99),
                host.clone(),
                layer.alloc().clone()
            ),
            Err(LayerError::BadMeshId(_))
        ),
        "unknown entry"
    );
    unchanged(&l);
    assert!(
        matches!(
            l.replace_mesh(mid, other_host, layer.alloc().clone()),
            Err(LayerError::BadMeshId(_))
        ),
        "host carries another mesh id"
    );
    unchanged(&l);
    assert!(
        l.replace_mesh(mid, host, SerialAlloc::new()).is_err(),
        "allocator may only advance"
    );
    unchanged(&l);
}

#[test]
fn model_layer_from_parts_shares_the_stores() {
    let (s, _) = set_session();
    let m = s.state().layer::<ModelLayer>().expect("model");
    let l = ModelLayer::from_parts(m.topo_host().clone(), Arc::new(m.model().clone()));
    assert_eq!(l.ops_run(), 0);
    assert!(Arc::ptr_eq(l.topo_host(), m.topo_host()));
    assert_eq!(l.hash_quanta(), m.hash_quanta());
}
