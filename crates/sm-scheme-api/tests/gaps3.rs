//! W0f gaps3: `Caps::BLOCK_STRUCTURED` and `Patch::strip_from` (an L-shaped submapped face
//! with no grid), and the stub sweep walking submapped (grid-less) linking faces.

mod stub_schemes;

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, Param, SerialAlloc, cell_volume, validate};
use sm_scheme_api::{
    Caps, Driver, ElemShape, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, NodeGrid,
    NodeKey, Patch, PatchBuilder, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice,
    Sequential, Target, WholeFaces,
};
use stub_schemes::{brick_setup, registry};

fn face() -> EntityId {
    EntityId::new(EntityKind::TopoFace, 1).unwrap()
}

/// The L-shaped domain `[0,2]^2 \ (1,2]^2` meshed as a submap does it: two blocks on a
/// half-unit lattice, `(i, j)` for `0 <= i, j <= 4` minus `i > 2 && j > 2`, no grid. Node
/// `(i, j)` has key index `5 j + i`.
fn l_patch() -> Patch {
    let f = face();
    let inside = |i: u32, j: u32| i <= 4 && j <= 4 && !(i > 2 && j > 2);
    let mut pb = PatchBuilder::new(Target::Surface(f));
    let mut ix = [[u32::MAX; 5]; 5];
    for j in 0..5u32 {
        for i in 0..5u32 {
            if inside(i, j) {
                let p = Point3::new(0.5 * f64::from(i), 0.5 * f64::from(j), 0.0);
                ix[j as usize][i as usize] = pb.node(NodeKey::new(f, 5 * j + i), p, Param::None);
            }
        }
    }
    for j in 0..4u32 {
        for i in 0..4u32 {
            if i >= 2 && j >= 2 {
                continue;
            }
            let (a, b) = (i as usize, j as usize);
            pb.push(ElemShape::Quad, &[ix[b][a], ix[b][a + 1], ix[b + 1][a + 1], ix[b + 1][a]]);
        }
    }
    pb.finish()
}

fn k(i: u32, j: u32) -> NodeKey {
    NodeKey::new(face(), 5 * j + i)
}

#[test]
fn block_structured_is_its_own_bit_and_documents_the_scheme_split() {
    assert_eq!(Caps::BLOCK_STRUCTURED, Caps(1 << 10));
    let map = Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED;
    let submap = Caps::QUAD | Caps::BLOCK_STRUCTURED;
    assert!(map.contains(Caps::QUAD | Caps::BLOCK_STRUCTURED) && submap.contains(Caps::QUAD | Caps::BLOCK_STRUCTURED));
    assert!(map.contains(Caps::STRUCTURED) && !submap.contains(Caps::STRUCTURED));
}

#[test]
fn strips_of_an_l_shaped_submapped_face_walk_the_element_table() {
    let p = l_patch();
    assert!(p.grid.is_none());
    assert_eq!(p.elements.len(), 12);
    // Up the long leg and across the short one.
    assert_eq!(p.strip_from(k(1, 0)), Some((0..5).map(|j| k(1, j)).collect()));
    assert_eq!(p.strip_from(k(3, 0)), Some((0..3).map(|j| k(3, j)).collect()), "ends on the re-entrant edge");
    assert_eq!(p.strip_from(k(0, 1)), Some((0..5).map(|i| k(i, 1)).collect()));
    assert_eq!(p.strip_from(k(0, 3)), Some((0..3).map(|i| k(i, 3)).collect()));
    // Walking back reverses the strip.
    assert_eq!(p.strip_from(k(1, 4)), Some((0..5).rev().map(|j| k(1, j)).collect()));
    assert_eq!(p.strip_from(k(4, 1)), Some((0..5).rev().map(|i| k(i, 1)).collect()));
    // The re-entrant corner has two interior edges: the smaller neighbour key, (2, 1), wins.
    assert_eq!(p.strip_from(k(2, 2)), Some(vec![k(2, 2), k(2, 1), k(2, 0)]));
    // Convex corners, interior nodes and unknown keys start no strip.
    for c in [k(0, 0), k(4, 0), k(4, 2), k(2, 4), k(0, 4)] {
        assert_eq!(p.strip_from(c), None, "{c:?}");
    }
    assert_eq!(p.strip_from(k(1, 1)), None);
    assert_eq!(p.strip_from(NodeKey::new(face(), 999)), None);
    // Deterministic.
    assert_eq!(l_patch().strip_from(k(2, 2)), p.strip_from(k(2, 2)));
}

#[test]
fn strip_from_uses_the_grid_when_there_is_one_and_refuses_non_quads() {
    // A 3 x 3 node map patch with its grid: the fast path equals `line_from`.
    let f = face();
    let mut pb = PatchBuilder::new(Target::Surface(f));
    let mut keys = Vec::new();
    let mut ix = Vec::new();
    for j in 0..3u32 {
        for i in 0..3u32 {
            let key = NodeKey::new(f, 3 * j + i);
            keys.push(key);
            ix.push(pb.node(key, Point3::new(f64::from(i), f64::from(j), 0.0), Param::None));
        }
    }
    for j in 0..2usize {
        for i in 0..2usize {
            pb.push(ElemShape::Quad, &[ix[3 * j + i], ix[3 * j + i + 1], ix[3 * (j + 1) + i + 1], ix[3 * (j + 1) + i]]);
        }
    }
    let grid = NodeGrid::new(3, 3, keys.clone());
    pb.grid(grid.clone());
    let p = pb.finish();
    for key in &keys {
        assert_eq!(p.strip_from(*key), grid.line_from(*key), "{key:?}");
    }
    let mut no_grid = p.clone();
    no_grid.grid = None;
    for key in &keys {
        assert_eq!(no_grid.strip_from(*key), grid.line_from(*key), "walk agrees with the grid at {key:?}");
    }
    // A triangle anywhere: not block-structured.
    let mut tri = no_grid.clone();
    tri.elements.push(ElemShape::Tri, &[0, 1, 4]);
    assert_eq!(tri.strip_from(keys[1]), None);
}

#[test]
fn sweep_walks_submapped_linking_faces_without_grids() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for (e, n) in [(0, 2u32), (4, 3), (8, 4)] {
        intent.set_intervals(ids.edges[e], Firm::hard(IntervalRequest::Count(n)));
    }
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard),
    );
    // Hard submap on every linking face: it satisfies QUAD | BLOCK_STRUCTURED, not STRUCTURED.
    for f in &ids.faces[..4] {
        intent.set_scheme(*f, SchemeChoice::new("submap", Firmness::Hard));
    }
    let mut alloc = SerialAlloc::new();
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc).expect("mesh");
    for f in &ids.faces[..4] {
        let tp = out.plan.target(Target::Surface(*f)).expect("linking plan");
        assert_eq!(tp.scheme.as_str(), "submap");
        assert!(tp.caps.contains(Caps::QUAD | Caps::BLOCK_STRUCTURED) && !tp.caps.contains(Caps::STRUCTURED));
    }
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let n = m.count(sm_mesh_api::MeshEntityKind::Cell);
    assert_eq!(n, 2 * 3 * 4);
    let total: f64 = (0..n as u32).map(|c| cell_volume(m, sm_mesh_api::CellIx(c))).sum();
    assert!((total - 3.0).abs() < 1e-9, "{total}");
}
