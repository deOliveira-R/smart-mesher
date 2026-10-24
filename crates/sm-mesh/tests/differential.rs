//! Differential tests: every sm-testkit mesh fixture, replayed into `sm-mesh`, must be identical
//! to the reference implementation (all accessors, ids, hashes, validation).

mod common;
use common::replay::{Mode, assert_same, copy_edge_tags, replay};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;
use sm_testkit::meshes::all_mesh_fixtures;

fn both_modes(src: &ReferenceMesh, name: &str) -> Vec<Mesh> {
    let mut out = Vec::new();
    for mode in [Mode::Explicit, Mode::Mesher] {
        // The harness itself: reference -> reference must be the identity.
        let (mut r, _) = replay(src, ReferenceBuilder::new_like(src), mode, &mut SerialAlloc::new()).unwrap();
        copy_edge_tags(src, &mut r.mesh);
        assert_same(src, &r.mesh, &format!("{name} harness {mode:?}"));
        // The subject.
        let mut alloc = SerialAlloc::new();
        let (mut m, prov) = replay(src, MeshBuilder::with_dim(src.dim()), mode, &mut alloc).unwrap();
        copy_edge_tags(src, &mut m.mesh);
        assert_same(src, &m.mesh, &format!("{name} {mode:?}"));
        // face_remap maps provisional handles to the (already canonical) source faces.
        for (f, p) in prov.iter().enumerate() {
            assert_eq!(m.face_remap[p.get()], FaceIx::new(f), "{name} {mode:?} remap");
        }
        out.push(m.mesh);
    }
    out
}

trait NewLike {
    fn new_like(src: &dyn MeshRead) -> Self;
}
impl NewLike for ReferenceBuilder {
    fn new_like(src: &dyn MeshRead) -> Self {
        ReferenceBuilder::with_dim(src.dim())
    }
}

#[test]
fn every_testkit_fixture_matches_the_reference() {
    for fx in all_mesh_fixtures().unwrap() {
        let ms = both_modes(&fx.mesh, fx.name);
        // The two feeding modes agree with each other too.
        assert_same(&ms[0], &ms[1], fx.name);
        // Known analytic volumes still hold through the new store.
        for (c, v) in fx.cell_volumes.iter().enumerate() {
            let got = cell_volume(&ms[1], CellIx::new(c));
            assert!((got - v).abs() < 1e-9 * (1.0 + v.abs()), "{} cell {c}: {got} vs {v}", fx.name);
        }
        let gc = compute_geometry_cache(&ms[1]);
        assert_eq!(gc.cell_volume.len(), fx.mesh.cell_count());
    }
}
