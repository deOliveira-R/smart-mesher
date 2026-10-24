//! Differential for `flatten`, `unfold` and the catalog: decorated meshes (sets, side sets,
//! labels, cell and face tags, reclassified faces) go through the reference and `sm-mesh`;
//! the results, errors and `FlattenMap`s must agree exactly.

mod common;
use common::replay::assert_same;
use common::{mirror_x, mirror_y};
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind, Point3, Transform};
use sm_mesh::{Catalog, MeshBuilder};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceCatalog};
use sm_mesh_api::*;
use sm_testkit::strategies::config;

fn cls(k: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(k, n).unwrap())
}

/// A decorated `n[0] x n[1] x n[2]` hex block; `seed` varies sets, tags and face classes.
fn decorated<B: MeshBuild>(mut b: B, n: [usize; 3], seed: u64, alloc: &mut SerialAlloc) -> Result<B::Output> {
    let at = |i: usize, j: usize, k: usize| i + (n[0] + 1) * (j + (n[1] + 1) * k);
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let v = b.add_vertex(Point3::new(i as f64, j as f64 * 0.5, k as f64 * 2.0))?;
                b.set_class(MeshEntityRef::Vertex(v), cls(EntityKind::TopoVertex, 1 + (seed + v.get() as u64) % 2))?;
            }
        }
    }
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let c = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]
                    .map(|(a, b, c)| VertexIx::new(at(i + a, j + b, k + c)));
                let a = b.add_cell_from_vertices(CellShape::Hex, &c)?;
                b.set_class(MeshEntityRef::Cell(a.cell), cls(EntityKind::TopoVolume, 1))?;
            }
        }
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        if b.face_degree(fi) == 1 {
            b.set_class(MeshEntityRef::Face(fi), cls(EntityKind::TopoFace, 1 + (seed + f as u64) % 3))?;
        }
    }
    let (nv, nf, nc) = (b.vertex_count(), b.face_count(), b.cell_count());
    let pick = |m: usize, s: u64| -> Vec<usize> { (0..m).filter(|i| !(*i as u64 * 7 + s).is_multiple_of(3)).collect() };
    b.add_set("verts", SetMembers::Vertices(pick(nv, seed).into_iter().map(VertexIx::new).collect()))?;
    b.add_set("side", SetMembers::Faces(pick(nf, seed + 1).into_iter().map(FaceIx::new).collect()))?;
    b.add_set("block", SetMembers::Cells(pick(nc, seed + 2).into_iter().map(CellIx::new).collect()))?;
    b.set_label("side", Some(seed as i64 % 5))?;
    b.set_label("block", Some(-1))?;
    b.add_tag("ct", MeshEntityKind::Cell, TagValues::I64((0..nc as i64).map(|i| i * 3 + seed as i64).collect()))?;
    b.add_tag("ft", MeshEntityKind::Face, TagValues::F64((0..nf).map(|i| i as f64 * 0.25).collect()))?;
    Ok(b.finish(alloc)?.mesh)
}

fn deck(k: u8) -> Vec<Transform> {
    match k % 4 {
        0 => vec![],
        1 => vec![mirror_x(0.0)],
        2 => vec![mirror_x(0.0), mirror_y(0.0)],
        _ => vec![mirror_y(0.0)],
    }
}

fn options<'a>(weld: bool, on_merge: MergeRule<'a>) -> FlattenOptions<'a> {
    if weld { FlattenOptions::new().with_weld_tol(1e-9).with_on_merge(on_merge) } else { FlattenOptions::new() }
}

proptest! {
    #![proptest_config(config(40))]

    #[test]
    fn unfold_matches_the_reference(
        nx in 1usize..4, ny in 1usize..3, nz in 1usize..3, seed in 0u64..50, d in 0u8..4, weld in any::<bool>(),
    ) {
        let n = [nx, ny, nz];
        let (mut ar, mut am) = (SerialAlloc::new(), SerialAlloc::new());
        let r = decorated(ReferenceBuilder::new(), n, seed, &mut ar).unwrap();
        let m = decorated(MeshBuilder::new(), n, seed, &mut am).unwrap();
        assert_same(&r, &m, "source");
        let merge = |c: &MergeCtx| Ok(Classification::on(c.cells[0].entity));
        let o = options(weld, &merge);
        let want = unfold(&r, &deck(d), ReferenceBuilder::new(), &o, &mut ar);
        let got = unfold(&m, &deck(d), MeshBuilder::new(), &o, &mut am);
        match (want, got) {
            (Ok((w, wm)), Ok((g, gm))) => {
                assert_same(&w, &g, "unfold");
                prop_assert_eq!(wm, gm);
            }
            (Err(w), Err(g)) => prop_assert_eq!(w, g),
            (w, g) => prop_assert!(false, "{:?} vs {:?}", w.map(|_| ()), g.map(|_| ())),
        }
        for k in [EntityKind::Mesh, EntityKind::MeshVertex, EntityKind::MeshFace, EntityKind::MeshCell, EntityKind::MeshInstance] {
            prop_assert_eq!(ar.next_serial(k), am.next_serial(k));
        }
    }

    #[test]
    fn catalog_flatten_matches_the_reference(
        nx in 1usize..3, ny in 1usize..3, seed in 0u64..50, weld in any::<bool>(), shift in 0u8..3,
    ) {
        let n = [nx, ny, 1];
        let (mut ar, mut am) = (SerialAlloc::new(), SerialAlloc::new());
        let r = decorated(ReferenceBuilder::new(), n, seed, &mut ar).unwrap();
        let m = decorated(MeshBuilder::new(), n, seed, &mut am).unwrap();
        let pid = r.mesh_id();
        prop_assert_eq!(pid, m.mesh_id());
        let vid = ar.one(EntityKind::MeshInstance).unwrap();
        prop_assert_eq!(vid, am.one(EntityKind::MeshInstance).unwrap());
        // Neighbour abutting along x, mirrored across the shared plane, or apart.
        let ts = [common::translate(nx as f64, 0.0, 0.0), mirror_x(nx as f64), common::translate(nx as f64 + 5.0, 0.0, 0.0)];
        let t = ts[shift as usize % 3];
        let mut insts = Vec::new();
        for tr in [Transform::IDENTITY, t] {
            let (a, b) = (ar.one(EntityKind::MeshInstance).unwrap(), am.one(EntityKind::MeshInstance).unwrap());
            prop_assert_eq!(a, b);
            insts.push(Instance::new(a, pid, tr));
        }
        let view = InstancedView::new(vid, insts);
        let mut cr = ReferenceCatalog::new();
        cr.insert_mesh(Box::new(r)).unwrap();
        cr.insert_view(view.clone()).unwrap();
        let mut cm = Catalog::new();
        cm.insert_mesh(Box::new(m)).unwrap();
        cm.insert_view(view).unwrap();
        prop_assert_eq!(cr.views(), cm.views());
        let merge = |c: &MergeCtx| Ok(Classification::on(c.cells[0].entity));
        let o = options(weld, &merge);
        let want = flatten(&cr, vid, ReferenceBuilder::new(), &o, &mut ar);
        let got = flatten(&cm, vid, MeshBuilder::new(), &o, &mut am);
        match (want, got) {
            (Ok((w, wm)), Ok((g, gm))) => {
                assert_same(&w, &g, "flatten");
                prop_assert_eq!(wm, gm);
            }
            (Err(w), Err(g)) => prop_assert_eq!(w, g),
            (w, g) => prop_assert!(false, "{:?} vs {:?}", w.map(|_| ()), g.map(|_| ())),
        }
    }
}
