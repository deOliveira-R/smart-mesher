//! ACR-LEVELMAP: `MeshLayer` as a `MeshCatalog` / `CatalogEdit` holding fine-to-coarse
//! `LevelMap` links (acceptance 1-5 of `docs/design/w4/drafts/meshops.md` MOD-0).
//!
//! The fine mesh is a 4x2x2 block of 0.5 cubes and the coarse one a 2x1x1 block of unit cubes
//! over the same 2 x 1 x 1 box, so every coarse cell has 8 fine children and the restricted
//! volumes are analytic (1 per coarse cell).

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{
    CoreLayers, MeshEntry, MeshLayer, ModelLayer, TopoHost, insert_mesh, retire_mesh,
};
use sm_mesh_api::{
    CatalogEdit, CellIx, LevelMap, MeshCatalog, MeshError, MeshRead, PointIx, SerialAlloc,
};
use sm_op_api::{
    ApplyCtx, DerivedCache, Engine, Hash256, Layer, LayerType, Meta, Op, OpError, QuietControl,
    Registry, Rev, Services, Session, TypedOp,
};
use sm_topo_api::CellComplex;

fn id(kind: EntityKind, serial: u64) -> EntityId {
    EntityId::new(kind, serial).expect("id")
}

/// Ids of the two meshes the first `Blocks` op inserts.
fn fine_id() -> EntityId {
    id(EntityKind::Mesh, 1)
}
fn coarse_id() -> EntityId {
    id(EntityKind::Mesh, 2)
}

fn block(n: [u32; 3], h: f64, alloc: &mut SerialAlloc) -> sm_mesh::Mesh {
    sm_mesh::generate::hex_block_bulk(n, [h; 3], alloc)
        .expect("hex")
        .mesh
}

fn centroid_x(m: &dyn MeshRead, c: CellIx) -> f64 {
    let v = m.cell_corners(c).expect("hex corners");
    v.iter().map(|p| m.point(p.get()).x).sum::<f64>() / v.len() as f64
}

/// Coarse parent of every fine cell by x centroid (coarse cells are unit wide).
fn parents(fine: &dyn MeshRead) -> Vec<CellIx> {
    (0..fine.cell_count())
        .map(|c| CellIx::new(centroid_x(fine, CellIx::new(c)).floor() as usize))
        .collect()
}

fn quanta() -> sm_mesh_api::HashQuanta {
    ModelLayer::default().hash_quanta()
}

fn entry(m: sm_mesh::Mesh) -> MeshEntry {
    MeshEntry::new(Box::new(m), vec![], None, quanta())
}

/// A layer holding the fine and the coarse block (ids Mesh 1 and Mesh 2).
fn layer_with_blocks() -> MeshLayer {
    let mut alloc = SerialAlloc::new();
    let fine = block([4, 2, 2], 0.5, &mut alloc);
    let after_fine = alloc.clone();
    let coarse = block([2, 1, 1], 1.0, &mut alloc);
    let mut l = MeshLayer::default();
    l.insert(fine.mesh_id(), entry(fine), after_fine)
        .expect("fine");
    l.insert(coarse.mesh_id(), entry(coarse), alloc)
        .expect("coarse");
    l
}

fn map_of(l: &MeshLayer) -> LevelMap {
    let (f, c) = (
        l.mesh(fine_id()).expect("fine"),
        l.mesh(coarse_id()).expect("coarse"),
    );
    LevelMap::new(f, c, parents(f), None).expect("map")
}

fn content(l: &MeshLayer) -> Hash256 {
    Layer::content_hash(l)
}

#[test]
fn link_stores_queries_and_restricts_analytically() {
    let mut l = layer_with_blocks();
    assert_eq!(l.mesh_ids(), &[fine_id(), coarse_id()]);
    assert!(l.maps_from(fine_id()).is_empty() && l.maps_to(coarse_id()).is_empty());
    let map = map_of(&l);
    l.link(map).expect("link");
    let from = l.maps_from(fine_id());
    assert_eq!(from.len(), 1);
    assert_eq!(from[0].0, coarse_id());
    assert_eq!(l.maps_to(coarse_id())[0].0, fine_id());
    assert!(l.maps_from(coarse_id()).is_empty() && l.maps_to(fine_id()).is_empty());
    // 16 fine cells of volume 0.125 sum to exactly 1 in each of the 2 coarse cells.
    let ones = vec![0.125; 16];
    assert_eq!(from[0].1.restrict_sum(&ones).expect("sum"), vec![1.0, 1.0]);
    for c in 0..2 {
        assert_eq!(from[0].1.fines_of(CellIx::new(c)).len(), 8);
    }
    assert!(l.views().is_empty());
}

#[test]
fn link_refusals_and_unsupported_entries() {
    let mut l = layer_with_blocks();
    let map = map_of(&l);
    l.link(map.clone()).expect("link");
    assert!(matches!(l.link(map), Err(MeshError::Hierarchy(m)) if m.contains("already")));
    // Unknown mesh: a map for a pair that is not in the layer (ids Mesh 1..3 from a fresh alloc).
    let mut alloc = SerialAlloc::new();
    let a = block([1, 1, 1], 1.0, &mut alloc);
    let _b = block([1, 1, 1], 1.0, &mut alloc);
    let c = block([1, 1, 1], 1.0, &mut alloc);
    let ghost = LevelMap::new(&c, &a, vec![CellIx::new(0)], None).expect("ghost map");
    assert_eq!(ghost.fine_id(), id(EntityKind::Mesh, 3));
    assert!(matches!(l.link(ghost), Err(MeshError::UnknownMesh(_))));
    // Unsupported entry points, and the default extrusion link.
    assert!(matches!(
        l.insert_mesh(Box::new(a)),
        Err(MeshError::Unsupported(_))
    ));
    assert!(matches!(
        l.insert_view(sm_mesh_api::InstancedView::new(
            id(EntityKind::MeshInstance, 1),
            vec![]
        )),
        Err(MeshError::Unsupported(_))
    ));
    assert_eq!(l.mesh_ids().len(), 2, "nothing was inserted");
}

/// Acceptance 5: a map whose mesh was replaced with different connectivity is refused.
#[test]
fn link_refuses_a_stale_pair() {
    let mut l = layer_with_blocks();
    let map = map_of(&l);
    // Replace the fine mesh (same id) with a 3x2x2 block: different connectivity.
    let mut alloc = SerialAlloc::new();
    let other = block([3, 2, 2], 0.5, &mut alloc);
    assert_eq!(other.mesh_id(), fine_id());
    let after = l.alloc().clone();
    l.replace_mesh(fine_id(), Arc::new(other), after)
        .expect("replace");
    match l.link(map) {
        Err(MeshError::Hierarchy(m)) => assert!(m.contains("stale"), "{m}"),
        other => panic!("expected stale refusal, got {other:?}"),
    }
    assert!(l.maps_from(fine_id()).is_empty());
}

#[test]
fn replace_keeps_a_link_for_equal_connectivity_and_drops_it_otherwise() {
    let mut l = layer_with_blocks();
    let map = map_of(&l);
    l.link(map).expect("link");
    let linked = content(&l);
    // A smooth-like change (one vertex moved) keeps connectivity: the link stays.
    let mut moved = l.get(fine_id()).expect("fine").mesh.clone_host();
    moved
        .set_point(PointIx(0), sm_core::Point3::new(-0.01, 0.0, 0.0))
        .expect("move");
    let after = l.alloc().clone();
    l.replace_mesh(fine_id(), Arc::from(moved), after)
        .expect("replace");
    assert_eq!(l.maps_from(fine_id()).len(), 1, "connectivity unchanged");
    assert_ne!(content(&l), linked, "the moved vertex is hashed");
    // Different connectivity drops it.
    let mut alloc = SerialAlloc::new();
    let other = block([3, 2, 2], 0.5, &mut alloc);
    let after = l.alloc().clone();
    l.replace_mesh(fine_id(), Arc::new(other), after)
        .expect("replace");
    assert!(l.maps_from(fine_id()).is_empty() && l.maps_to(coarse_id()).is_empty());
}

#[test]
fn digest_section_appears_only_with_links_and_sees_the_map_content() {
    let mut l = layer_with_blocks();
    let none = content(&l);
    let exact_none = Layer::exact_hash(&l);
    let map = map_of(&l);
    l.link(map.clone()).expect("link");
    let linked = content(&l);
    assert_ne!(none, linked);
    assert_ne!(exact_none, Layer::exact_hash(&l));
    // Same links, same digest.
    let mut l2 = layer_with_blocks();
    l2.link(map).expect("link");
    assert_eq!(content(&l2), linked);
    // A different map content (one fine cell re-parented) hashes differently.
    let mut l3 = layer_with_blocks();
    let alt = {
        let (f, c) = (
            l3.mesh(fine_id()).expect("f"),
            l3.mesh(coarse_id()).expect("c"),
        );
        let mut p = parents(f);
        p[0] = CellIx::new(1);
        LevelMap::new(f, c, p, None).expect("alt")
    };
    l3.link(alt).expect("link");
    assert_ne!(content(&l3), linked);
    // Dropping the link (coarse mesh retired) gives the digest of a layer that never had one.
    LayerType::on_retired(&mut l, &[coarse_id()]);
    assert!(l.maps_from(fine_id()).is_empty());
    assert!(l.get(coarse_id()).is_none() && l.mesh_ids() == [fine_id()]);
    let mut only_fine = MeshLayer::default();
    let mut alloc = SerialAlloc::new();
    let fine = block([4, 2, 2], 0.5, &mut alloc);
    only_fine
        .insert(fine.mesh_id(), entry(fine), l.alloc().clone())
        .expect("fine");
    assert_eq!(content(&l), content(&only_fine), "no link, no section");
}

/// Acceptance 4: a link is dropped when either mesh is retired.
#[test]
fn retiring_either_mesh_drops_the_link() {
    for retired in [fine_id(), coarse_id()] {
        let mut l = layer_with_blocks();
        let map = map_of(&l);
        l.link(map).expect("link");
        assert!(LayerType::references_any(&l, &[retired]));
        LayerType::on_retired(&mut l, &[retired]);
        assert!(l.maps_from(fine_id()).is_empty() && l.maps_to(coarse_id()).is_empty());
        assert_eq!(l.mesh_ids().len(), 1);
        // `remove` does the same.
        let mut l = layer_with_blocks();
        let map = map_of(&l);
        l.link(map).expect("link");
        l.remove(retired).expect("remove");
        assert!(l.maps_from(fine_id()).is_empty() && l.maps_to(coarse_id()).is_empty());
    }
}

// ------------------------------------------------------------- through the session

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Blocks {
    /// When non-zero: insert one unrelated `extra x 1 x 1` block instead of the fine/coarse pair.
    extra: u32,
}

impl TypedOp for Blocks {
    const NAME: &'static str = "test.blocks";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let specs: Vec<([u32; 3], f64)> = if self.extra == 0 {
            vec![([4, 2, 2], 0.5), ([2, 1, 1], 1.0)]
        } else {
            vec![([self.extra, 1, 1], 1.0)]
        };
        for (n, h) in specs {
            let before = ctx.layer::<MeshLayer>()?.alloc().clone();
            let mut alloc = before.clone();
            let m = block(n, h, &mut alloc);
            insert_mesh(ctx, entry(m), &before, alloc, "mesh")?;
        }
        Ok(())
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Link {}

impl TypedOp for Link {
    const NAME: &'static str = "test.link";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let map = {
            let l = ctx.layer::<MeshLayer>()?;
            let f = l.mesh(fine_id()).ok_or(OpError::UnknownEntity(fine_id()))?;
            let c = l
                .mesh(coarse_id())
                .ok_or(OpError::UnknownEntity(coarse_id()))?;
            LevelMap::new(f, c, parents(f), None).map_err(|e| OpError::Failed(e.to_string()))?
        };
        ctx.layer_mut::<MeshLayer>()?
            .link(map)
            .map_err(|e| OpError::Failed(e.to_string()))
    }
}

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
struct Delete {
    mesh: EntityId,
}

impl TypedOp for Delete {
    const NAME: &'static str = "test.mesh_delete";
    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        retire_mesh(ctx, self.mesh)
    }
}

fn topo() -> Box<dyn TopoHost> {
    Box::new(CellComplex::new())
}

fn session() -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(topo as fn() -> Box<dyn TopoHost>))
        .expect("core");
    reg.register::<Blocks>().expect("blocks");
    reg.register::<Link>().expect("link");
    reg.register::<Delete>().expect("delete");
    let e = Engine::new(
        Arc::new(reg),
        Services::new(),
        Arc::new(DerivedCache::disabled()),
    )
    .expect("engine");
    Session::new(e)
}

fn run(s: &mut Session, op: &dyn Op) -> Hash256 {
    let q = QuietControl::new();
    s.apply(op, Meta::new("test", "", ""), &q.control())
        .expect("apply")
        .0
}

fn links_of(s: &Session) -> usize {
    s.state()
        .layer::<MeshLayer>()
        .expect("layer")
        .maps_from(fine_id())
        .len()
}

/// Acceptance 2: a link survives a fork and replay with equal hashes.
///
/// Replay here is the cold replay of the recorded ops from the root (`state_at_from_root`,
/// which verifies every node hash). The ops that will create links (`W4-MOD-REFINE-U`,
/// `W4-POLY-AGG`) do not exist yet; they must call `CatalogEdit::link` on
/// `ctx.layer_mut::<MeshLayer>()` in the op that inserts the second mesh (or a following op),
/// as `Link` does here. Their replay is then covered by the same mechanism.
#[test]
fn links_survive_fork_and_cold_replay_with_equal_hashes() {
    let mut s = session();
    let n_blocks = run(&mut s, &Blocks { extra: 0 });
    let h_unlinked = s.state().hash();
    let h_link = run(&mut s, &Link {});
    assert_ne!(s.state().hash(), h_unlinked, "the link is hashed state");
    assert_eq!(links_of(&s), 1);
    let linked_state = s.state().hash();

    let q = QuietControl::new();
    let cold = s
        .state_at_from_root(h_link, &q.control())
        .expect("replay verifies node hashes");
    assert_eq!(cold.hash(), linked_state);
    assert_eq!(
        cold.layer::<MeshLayer>()
            .expect("layer")
            .maps_from(fine_id())
            .len(),
        1
    );

    // Fork at the linked node and add an unrelated mesh on the fork.
    s.branch("fork").expect("branch");
    run(&mut s, &Blocks { extra: 3 });
    assert_eq!(links_of(&s), 1, "the link is carried into the fork");
    let fork_head = s.state().hash();
    assert_ne!(fork_head, linked_state);
    let tip = *s.timeline().refs().get("fork").expect("ref");
    let cold_fork = s
        .state_at_from_root(tip, &q.control())
        .expect("replay fork");
    assert_eq!(cold_fork.hash(), fork_head);
    assert_eq!(
        cold_fork
            .layer::<MeshLayer>()
            .expect("layer")
            .maps_from(fine_id())
            .len(),
        1
    );

    // Back at the original node: the hash from before the fork.
    s.checkout(Rev::Node(h_link), &q.control())
        .expect("checkout");
    assert_eq!(s.state().hash(), linked_state);
    // The node before the link has no link; back at the linked node it is there again.
    s.checkout(Rev::Node(n_blocks), &q.control())
        .expect("checkout");
    assert_eq!(links_of(&s), 0);
    assert_eq!(s.state().hash(), h_unlinked);
    s.checkout(Rev::Node(h_link), &q.control())
        .expect("checkout");
    assert_eq!(links_of(&s), 1);
    assert_eq!(s.state().hash(), linked_state);
}

#[test]
fn retire_mesh_drops_the_link_and_restores_the_link_free_hash() {
    let mut s = session();
    run(&mut s, &Blocks { extra: 0 });
    run(&mut s, &Link {});
    run(&mut s, &Delete { mesh: coarse_id() });
    assert_eq!(links_of(&s), 0);
    let mut t = session();
    run(&mut t, &Blocks { extra: 0 });
    run(&mut t, &Delete { mesh: coarse_id() });
    assert_eq!(
        s.state().hash(),
        t.state().hash(),
        "no link left, no link bytes"
    );
}

#[test]
fn a_refused_link_op_leaves_the_state_alone() {
    let mut s = session();
    run(&mut s, &Blocks { extra: 0 });
    run(&mut s, &Link {});
    let before = s.state().hash();
    let q = QuietControl::new();
    let r = s.apply(&Link {}, Meta::new("test", "", ""), &q.control());
    assert!(r.is_err(), "a second link for the pair is refused");
    assert_eq!(s.state().hash(), before);
}

#[test]
fn determinism_two_runs_equal() {
    let build = || {
        let mut s = session();
        run(&mut s, &Blocks { extra: 0 });
        run(&mut s, &Link {});
        s.state().hash()
    };
    assert_eq!(build(), build());
}
