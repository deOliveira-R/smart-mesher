//! Shared rig: a session over `CoreLayers` + `ViewPlugin` with test ops that insert meshes.
#![allow(dead_code, missing_docs)]

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sm_core::{EntityId, EntityKind, Point3};
use sm_layer_api::{CoreLayers, LayerResultExt, MeshEntry, MeshLayer, ModelLayer, commit_topology, insert_mesh, model_txn, topo_txn};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{CellShape, MeshBuild, MeshEdit, MeshEntityRef, PointIx, SerialAlloc, VertexIx};
use sm_op_api::{ApplyCtx, Engine, Meta, OpError, QuietControl, Registry, Services, Session, State, TypedOp};
use sm_testkit::meshes::{CellSpec, build_mesh, dual_patch, hex_block_two_volumes};
use sm_topo_api::fixtures::brick;
use sm_view::ViewPlugin;

/// A brick at the origin: model entities for `view.scene`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateBrick {
    pub size: [f64; 3],
}

impl TypedOp for CreateBrick {
    const NAME: &'static str = "test.create_brick";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (ids, txn) = topo_txn(ctx, |topo, _| brick(topo, Point3::new(0.0, 0.0, 0.0), self.size).or_op())?;
        commit_topology(ctx, &txn, &[("volume", &[ids.volume]), ("faces", &ids.faces), ("edges", &ids.edges)])
    }
}

/// Insert a fixture mesh: `block`, `two_volumes`, `dual_patch` or `quad_patch`.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InsertMesh {
    pub kind: String,
    #[serde(default)]
    pub n: [u32; 3],
    #[serde(default)]
    pub moves: Vec<(usize, [f64; 3])>,
}

/// An `n x n` unit-quad patch in z = 0 on `TopoFace#1`.
/// Color a volume with a model color of the given label.
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ColorVolume {
    pub volume: EntityId,
    pub label: u32,
    pub name: String,
}

impl TypedOp for ColorVolume {
    const NAME: &'static str = "test.color_volume";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let bad = |e: sm_model_api::ModelError| OpError::Invalid(e.to_string());
        model_txn(
            ctx,
            |m, _| {
                let c = m.add_color(self.label, &self.name, sm_model_api::Domain::Volume).map_err(bad)?;
                m.color_volumes(c, &[self.volume]).map_err(bad)
            },
            |_, ids| vec![("color", ids.to_vec())],
        )
    }
}

pub fn quad_patch(n: usize, alloc: &mut SerialAlloc) -> ReferenceMesh {
    use sm_mesh_api::Classification;
    let on = |k: EntityKind, s: u64| Classification::on(EntityId::new(k, s).unwrap());
    let mut b = sm_mesh_api::reference::ReferenceBuilder::surface();
    let mut vs: Vec<VertexIx> = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            let v = b.add_vertex(Point3::new(i as f64, j as f64, 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + vs.len() as u64)).unwrap();
            vs.push(v);
        }
    }
    for j in 0..n {
        for i in 0..n {
            let a = j * (n + 1) + i;
            let cyc = [vs[a], vs[a + 1], vs[a + n + 2], vs[a + n + 1]];
            let added = b.add_polygon(&cyc).unwrap();
            b.set_class(MeshEntityRef::Cell(added.cell), on(EntityKind::TopoFace, 1)).unwrap();
        }
    }
    let mut k = 1;
    for f in 0..b.face_count() {
        let f = sm_mesh_api::FaceIx::new(f);
        if b.face_degree(f) == 1 {
            b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, k)).unwrap();
            k += 1;
        }
    }
    b.finish(alloc).unwrap().mesh
}

pub fn build(kind: &str, n: [u32; 3], moves: &[(usize, [f64; 3])], alloc: &mut SerialAlloc) -> ReferenceMesh {
    match kind {
        "block" => {
            let mut m = hex_block(n, [1.0; 3], alloc).expect("block");
            for (v, p) in moves {
                m.set_point(PointIx::new(*v), Point3::new(p[0], p[1], p[2])).expect("move");
            }
            m
        }
        "two_volumes" => hex_block_two_volumes([4, 2, 2], [1.0; 3], 2).expect("two").mesh,
        "dual_patch" => dual_patch().expect("dual").mesh,
        "quad_patch" => quad_patch(3, alloc),
        "single_tet" => {
            let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
            build_mesh(&p, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], alloc).expect("tet")
        }
        other => panic!("unknown fixture {other}"),
    }
}

/// The allocator state that covers every id of `m` (for fixtures built with their own allocator).
pub fn alloc_after(m: &ReferenceMesh) -> SerialAlloc {
    use sm_mesh_api::{MeshEntityKind as K, MeshRead};
    let mut max: std::collections::BTreeMap<EntityKind, u64> = std::collections::BTreeMap::new();
    let mut note = |id: EntityId| {
        let e = max.entry(id.kind()).or_insert(0);
        *e = (*e).max(id.serial());
    };
    note(m.mesh_id());
    for k in [K::Vertex, K::Edge, K::Face, K::Cell] {
        for i in 0..m.count(k) {
            note(m.id(k, i as u32));
        }
    }
    for s in m.sets() {
        note(s.id);
    }
    SerialAlloc::from_state(max.into_iter().map(|(k, n)| (k, n + 1))).expect("alloc")
}

impl TypedOp for InsertMesh {
    const NAME: &'static str = "test.insert_mesh";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let before = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut after = before.clone();
        let mesh = build(&self.kind, self.n, &self.moves, &mut after);
        if matches!(self.kind.as_str(), "two_volumes" | "dual_patch") {
            after = alloc_after(&mesh);
        }
        let quanta = ctx.layer::<ModelLayer>()?.hash_quanta();
        insert_mesh(ctx, MeshEntry::new(Box::new(mesh), Vec::new(), None, quanta), &before, after, "mesh")
    }
}

pub fn registry() -> Registry {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::default()).expect("layers");
    reg.install(&ViewPlugin).expect("view");
    reg.register::<CreateBrick>().expect("brick");
    reg.register::<InsertMesh>().expect("mesh");
    reg.register::<ColorVolume>().expect("color");
    reg
}

pub struct Rig {
    pub s: Session,
}

impl Rig {
    pub fn new() -> Self {
        let engine = Engine::new(Arc::new(registry()), Services::new(), Arc::default()).expect("engine");
        Self { s: Session::new(engine) }
    }

    pub fn apply<T: TypedOp>(&mut self, op: T) -> sm_op_api::Delta {
        let q = QuietControl::new();
        self.s.apply(&op, Meta::new("test", "", ""), &q.control()).expect("apply").1
    }

    pub fn with(kind: &str, n: [u32; 3], moves: &[(usize, [f64; 3])]) -> (Self, EntityId) {
        let mut r = Self::new();
        let d = r.apply(InsertMesh { kind: kind.into(), n, moves: moves.to_vec() });
        let id = d.outputs["mesh"][0];
        (r, id)
    }

    pub fn state(&self) -> &State {
        self.s.state()
    }

    pub fn query(&self, name: &str, input: Value) -> Result<Value, String> {
        let q = QuietControl::new();
        self.s.engine().run_query(self.s.state(), name, &input, &q.control()).map_err(|e| e.to_string())
    }
}

pub fn b64_f32(v: &Value) -> Vec<f32> {
    let b = sm_view::b64::decode(v.as_str().expect("string")).expect("b64");
    b.as_chunks::<4>().0.iter().map(|c| f32::from_le_bytes(*c)).collect()
}

pub fn b64_u32(v: &Value) -> Vec<u32> {
    let b = sm_view::b64::decode(v.as_str().expect("string")).expect("b64");
    b.as_chunks::<4>().0.iter().map(|c| u32::from_le_bytes(*c)).collect()
}

/// Sum of triangle areas of a skin (f64 positions).
pub fn area(s: &sm_view::Skin) -> f64 {
    let p = |i: u32| {
        let i = i as usize * 3;
        [s.positions[i], s.positions[i + 1], s.positions[i + 2]]
    };
    s.indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|t| {
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            0.5 * (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt()
        })
        .sum()
}
