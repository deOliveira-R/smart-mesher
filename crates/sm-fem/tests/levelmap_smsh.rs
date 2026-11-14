//! The `.smsh` level-map passthrough: a link of the mesh layer reaches the file of either of its
//! meshes. The link is made directly through `CatalogEdit::link` (W4-MOD-REFINE-U, which makes
//! them from an op, is a separate task); the fine mesh is a 4x2x2 block of 0.5 cubes and the
//! coarse one a 2x1x1 block of unit cubes, so every coarse cell has 8 fine children of volume
//! 0.125 and the restricted volume is exactly 1.
#![allow(missing_docs)]

mod common;

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_fem::{ExportFormat, ExportRequest, export};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh};
use sm_mesh_api::{CatalogEdit, CellIx, LevelMap, MeshCatalog, SerialAlloc};
use sm_meshio::SmshFile;
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Meta, OpError, QuietControl, Registry, Services, Session,
    TypedOp,
};

fn mesh_id(n: u64) -> EntityId {
    EntityId::new(EntityKind::Mesh, n).expect("id")
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Pair {}

impl TypedOp for Pair {
    const NAME: &'static str = "test.pair_meshes";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        for (n, h) in [([4u32, 2, 2], 0.5), ([2, 1, 1], 1.0)] {
            let before = ctx.layer::<MeshLayer>()?.alloc().clone();
            let mut alloc: SerialAlloc = before.clone();
            let m = sm_mesh::generate::hex_block_bulk(n, [h; 3], &mut alloc)
                .map_err(|e| OpError::Failed(e.to_string()))?
                .mesh;
            let q = ctx.layer::<ModelLayer>()?.hash_quanta();
            insert_mesh(
                ctx,
                MeshEntry::new(Box::new(m), vec![], None, q),
                &before,
                alloc,
                "mesh",
            )?;
        }
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Link {}

impl TypedOp for Link {
    const NAME: &'static str = "test.link_meshes";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let map = {
            let l = ctx.layer::<MeshLayer>()?;
            let f = l.mesh(mesh_id(1)).ok_or(OpError::UnknownEntity(mesh_id(1)))?;
            let c = l.mesh(mesh_id(2)).ok_or(OpError::UnknownEntity(mesh_id(2)))?;
            let parents = (0..f.cell_count())
                .map(|i| {
                    let corners = f.cell_corners(CellIx::new(i)).expect("hex");
                    let x = corners.iter().map(|p| f.point(p.get()).x).sum::<f64>() / 8.0;
                    CellIx::new(x.floor() as usize)
                })
                .collect();
            LevelMap::new(f, c, parents, None).map_err(|e| OpError::Failed(e.to_string()))?
        };
        ctx.layer_mut::<MeshLayer>()?
            .link(map)
            .map_err(|e| OpError::Failed(e.to_string()))
    }
}

fn session() -> Session {
    let mut reg: Registry = common::registry(true);
    reg.register::<Pair>().expect("pair");
    reg.register::<Link>().expect("link");
    let e = Engine::new(
        Arc::new(reg),
        Services::new(),
        Arc::new(DerivedCache::disabled()),
    )
    .expect("engine");
    Session::new(e)
}

fn run(s: &mut Session, op: &dyn sm_op_api::Op) {
    let q = QuietControl::new();
    s.apply(op, Meta::new("test", "", ""), &q.control())
        .expect("apply");
}

fn smsh_of(s: &Session, id: EntityId) -> Vec<u8> {
    let mut out = Vec::new();
    export(
        s.state(),
        &ExportRequest::new(ExportFormat::Smsh).mesh(id),
        &mut out,
    )
    .expect("export");
    out
}

#[test]
fn a_linked_mesh_exports_its_level_map_from_either_side() {
    let mut s = session();
    run(&mut s, &Pair {});
    let unlinked = (smsh_of(&s, mesh_id(1)), smsh_of(&s, mesh_id(2)));
    for b in [&unlinked.0, &unlinked.1] {
        let d = SmshFile::parse(b).expect("parse").decode().expect("decode");
        assert!(d.level_maps.is_empty());
    }
    run(&mut s, &Link {});

    // Fine side: the map reads back and restricts the cell volumes analytically.
    let bytes = smsh_of(&s, mesh_id(1));
    let d = SmshFile::parse(&bytes).expect("parse").decode().expect("decode");
    assert_eq!(d.level_maps.len(), 1);
    let m = &d.level_maps[0];
    assert_eq!((m.fine, m.coarse), (mesh_id(1), mesh_id(2)));
    assert_eq!(m.coarse_of.len(), 16);
    let mut per_coarse = [0.0f64; 2];
    for &c in &m.coarse_of {
        per_coarse[c as usize] += 0.125;
    }
    assert_eq!(per_coarse, [1.0, 1.0]);
    assert_eq!(m.coarse_of.iter().filter(|&&c| c == 0).count(), 8);
    // The stored map equals the layer's own.
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let live = &layer.maps_from(mesh_id(1))[0].1;
    for (i, &c) in m.coarse_of.iter().enumerate() {
        assert_eq!(live.coarse_of(CellIx::new(i)).get() as u32, c);
    }

    // Coarse side: the same link is in the file of the coarse mesh.
    let bytes = smsh_of(&s, mesh_id(2));
    let d = SmshFile::parse(&bytes).expect("parse").decode().expect("decode");
    assert_eq!(d.level_maps.len(), 1);
    assert_eq!(
        (d.level_maps[0].fine, d.level_maps[0].coarse),
        (mesh_id(1), mesh_id(2))
    );
    assert_ne!(bytes, unlinked.1);
}

#[test]
fn export_is_deterministic_with_a_link() {
    let build = || {
        let mut s = session();
        run(&mut s, &Pair {});
        run(&mut s, &Link {});
        (smsh_of(&s, mesh_id(1)), smsh_of(&s, mesh_id(2)))
    };
    assert_eq!(build(), build());
}

#[test]
fn a_dimension_only_select_keeps_the_level_map_and_the_plain_bytes() {
    use sm_meshio::select::{Dimension, Select};
    let mut s = session();
    run(&mut s, &Pair {});
    run(&mut s, &Link {});
    let sel = Select { dimension: Dimension::Three, ..Default::default() };
    for n in [1, 2] {
        let plain = smsh_of(&s, mesh_id(n));
        let mut out = Vec::new();
        export(
            s.state(),
            &ExportRequest::new(ExportFormat::Smsh).mesh(mesh_id(n)).select(sel.clone()),
            &mut out,
        )
        .expect("export");
        assert_eq!(out, plain);
        let d = SmshFile::parse(&out).expect("parse").decode().expect("decode");
        assert_eq!(d.level_maps.len(), 1);
    }
    // A wrong dimension is still refused: the check runs.
    let bad = Select { dimension: Dimension::Two, ..Default::default() };
    let r = export(
        s.state(),
        &ExportRequest::new(ExportFormat::Smsh).mesh(mesh_id(1)).select(bad),
        &mut Vec::new(),
    );
    assert!(r.is_err());
}
