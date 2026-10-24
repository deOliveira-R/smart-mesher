//! Edge table (parallel derivation) against the contract's `EdgeTable::derive`, and the
//! adjacency tables against brute force.

mod common;
use common::replay::{Mode, assert_same, replay};
use sm_mesh::generate::hex_block;
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::reference::{ReferenceMesh, hex_block as ref_hex_block, hex_block_with_edges};
use sm_mesh_api::*;
use sm_testkit::meshes::all_mesh_fixtures;
use std::collections::BTreeSet;

fn mine(src: &ReferenceMesh) -> Mesh {
    replay(src, MeshBuilder::new(), Mode::Mesher, &mut SerialAlloc::new()).unwrap().0.mesh
}

#[test]
fn edge_derivation_matches_the_contract_on_fixtures_and_blocks() {
    let mut cases: Vec<(String, ReferenceMesh)> =
        all_mesh_fixtures().unwrap().into_iter().map(|f| (f.name.to_string(), f.mesh)).collect();
    for n in [[1, 1, 1], [2, 1, 3], [3, 3, 3]] {
        cases.push((format!("block{n:?}"), ref_hex_block(n, [1.0; 3], &mut SerialAlloc::new()).unwrap()));
    }
    for (name, r) in cases {
        let mut m = mine(&r);
        let (mut a1, mut a2) = (SerialAlloc::new(), SerialAlloc::new());
        // Burn some ids first so the serial bases are not trivially 1.
        a1.take(sm_core::EntityKind::MeshEdge, 7).unwrap();
        a2.take(sm_core::EntityKind::MeshEdge, 7).unwrap();
        let mut rr = r.clone();
        let got = m.ensure_edges(&mut a1, &[]);
        let want = rr.ensure_edges(&mut a2, &[]);
        assert_eq!(got, want, "{name}: result");
        assert_eq!(
            a1.next_serial(sm_core::EntityKind::MeshEdge),
            a2.next_serial(sm_core::EntityKind::MeshEdge),
            "{name}: allocator"
        );
        if got.is_ok() {
            assert_same(&rr, &m, &name);
        }
        // With explicit classes (from the reference table of a block) the table must agree.
        if let Some(t) = r.edges() {
            let explicit: Vec<_> = t.vertices.iter().zip(&t.class).map(|(v, c)| (v[0], v[1], *c)).collect();
            let mut m2 = mine(&r);
            let mut r2 = r.clone();
            r2.drop_edges().unwrap();
            m2.drop_edges().ok();
            let (mut b1, mut b2) = (SerialAlloc::new(), SerialAlloc::new());
            m2.ensure_edges(&mut b1, &explicit).unwrap();
            r2.ensure_edges(&mut b2, &explicit).unwrap();
            assert_eq!(m2.edges(), r2.edges(), "{name}: explicit classes");
        }
    }
    // A fully classified block carries a table; the replay keeps it.
    let r = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let m = hex_block(MeshBuilder::new(), [2, 2, 2], [1.0; 3], true, &mut SerialAlloc::new()).unwrap().mesh;
    assert_same(&r, &m, "block with edges");
    // An unknown explicit pair is an error and draws no ids.
    let mut m = mine(&ref_hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap());
    let mut a = SerialAlloc::new();
    let bad = (VertexIx(0), VertexIx(6), Classification::on(sm_core::EntityId::new(sm_core::EntityKind::TopoFace, 1).unwrap()));
    let before = a.next_serial(sm_core::EntityKind::MeshEdge);
    assert!(m.ensure_edges(&mut a, &[bad]).is_err());
    assert_eq!(a.next_serial(sm_core::EntityKind::MeshEdge), before);
    assert!(m.edges().is_none());
}

fn brute_vertex_cells(m: &Mesh) -> Vec<BTreeSet<u32>> {
    let mut out = vec![BTreeSet::new(); m.vertex_count()];
    for c in 0..m.cell_count() {
        for sf in m.cell_faces(CellIx::new(c)) {
            for v in m.face_vertices(sf.face()) {
                out[v.get()].insert(c as u32);
            }
        }
    }
    out
}

#[test]
fn adjacency_tables_match_brute_force() {
    let mut meshes: Vec<(String, Mesh)> =
        all_mesh_fixtures().unwrap().into_iter().map(|f| (f.name.to_string(), mine(&f.mesh))).collect();
    meshes.push((
        "block".into(),
        hex_block(MeshBuilder::new(), [3, 4, 2], [1.0; 3], true, &mut SerialAlloc::new()).unwrap().mesh,
    ));
    for (name, m) in &meshes {
        let vc = m.vertex_cells();
        let want = brute_vertex_cells(m);
        assert_eq!(vc.len(), m.vertex_count(), "{name}");
        for (v, w) in want.iter().enumerate() {
            assert_eq!(vc.row(v), w.iter().copied().collect::<Vec<_>>().as_slice(), "{name} v{v}");
        }
        let vf = m.vertex_faces();
        for v in 0..m.vertex_count() {
            let w: BTreeSet<u32> = (0..m.face_count() as u32)
                .filter(|f| m.face_vertices(FaceIx(*f)).contains(&VertexIx::new(v)))
                .collect();
            assert_eq!(vf.row(v), w.iter().copied().collect::<Vec<_>>().as_slice(), "{name} vf{v}");
        }
        let nb = m.cell_neighbors();
        for c in 0..m.cell_count() {
            let w: BTreeSet<u32> = m
                .cell_faces(CellIx::new(c))
                .iter()
                .filter_map(|sf| m.across(*sf).map(|x| x.0))
                .collect();
            assert_eq!(nb.row(c), w.iter().copied().collect::<Vec<_>>().as_slice(), "{name} nb{c}");
            // Symmetric.
            for &o in nb.row(c) {
                assert!(nb.row(o as usize).contains(&(c as u32)));
            }
        }
        match m.edges() {
            None => assert!(m.edge_faces().is_none() && m.vertex_edges().is_none()),
            Some(t) => {
                let ef = m.edge_faces().unwrap();
                for e in 0..t.len() {
                    let w: BTreeSet<u32> = (0..m.face_count())
                        .filter(|f| t.of_face(FaceIx::new(*f)).iter().any(|fe| fe.edge.get() == e))
                        .map(|f| f as u32)
                        .collect();
                    assert_eq!(ef.row(e), w.iter().copied().collect::<Vec<_>>().as_slice(), "{name} ef{e}");
                }
                let ve = m.vertex_edges().unwrap();
                for v in 0..m.vertex_count() {
                    let w: BTreeSet<u32> = (0..t.len())
                        .filter(|e| t.vertices[*e].contains(&VertexIx::new(v)))
                        .map(|e| e as u32)
                        .collect();
                    assert_eq!(ve.row(v), w.iter().copied().collect::<Vec<_>>().as_slice(), "{name} ve{v}");
                }
            }
        }
    }
    // Euler on the structured block through the tables: V - E + F - C = 1 for a ball.
    let (name, m) = meshes.last().unwrap();
    assert!(m.edges().is_some(), "{name}: the classified block carries its edge table");
    let t = m.edges().unwrap();
    assert_eq!(m.vertex_count() as i64 - t.len() as i64 + m.face_count() as i64 - m.cell_count() as i64, 1);
}

#[test]
fn adjacency_caches_survive_edits_and_reset_with_the_edge_table() {
    let mut m = hex_block(MeshBuilder::new(), [2, 2, 2], [1.0; 3], true, &mut SerialAlloc::new()).unwrap().mesh;
    let ef = m.edge_faces().unwrap().clone();
    m.set_point(PointIx(0), sm_core::Point3::new(-0.1, 0.0, 0.0)).unwrap();
    assert_eq!(m.edge_faces().unwrap(), &ef);
    m.drop_edges().unwrap();
    assert!(m.edge_faces().is_none());
    // A clone starts without caches but answers the same.
    let m2 = m.clone();
    assert_eq!(m.vertex_cells(), m2.vertex_cells());
    assert_eq!(m, m2);
}
