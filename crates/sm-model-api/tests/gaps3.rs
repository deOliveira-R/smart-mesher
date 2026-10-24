//! W0f gaps3: groups map members through `live_cover` (overlay-aware), and
//! `Model::canonical_visit` (Q2 identity words vs raw exact words).

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::ModelTolerance;
use sm_model_api::*;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, Dim, HashWord, OpId, Overlay, TopologyView};
use std::sync::Arc;

#[test]
fn group_of_a_hidden_top_face_yields_live_overlay_faces_and_chord_vertices() {
    // Brick [0,2]x[0,1]x[0,1] split at x = 1 through the overlay.
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut m = Model::new();
    let mat = m.add_color(1, "mat", Domain::Volume).unwrap();
    m.color_volumes(mat, &[b.volume]).unwrap();
    let top = b.faces[5];
    let g = m.add_group("top", &[top]).unwrap();
    let gv = m.add_group("vol", &[b.volume]).unwrap();
    // On the host the group is the face itself.
    assert_eq!(m.group_faces(g, host.as_ref()).unwrap(), vec![top]);
    let mut ov = Overlay::new(host.clone());
    let [n, p] = m.split_realized(PieceRef::Whole(b.volume), Plane::new(Vector3::new(1.0, 0.0, 0.0), 1.0).unwrap(), &mut ov).unwrap();
    assert!(!ov.is_alive(top));
    let faces = m.group_faces(g, &ov).unwrap();
    assert_eq!(faces, ov.live_cover(top));
    assert_eq!(faces.len(), 2);
    assert!(faces.iter().all(|f| ov.is_alive(*f) && f.kind() == EntityKind::PartitionFace));
    assert_eq!(m.group_entities(g, &ov, Dim::Face).unwrap(), faces);
    // The Vertex expansion includes the virtual vertices at the ends of the cut chord.
    let verts = m.group_entities(g, &ov, Dim::Vertex).unwrap();
    let pv: Vec<EntityId> = verts.iter().copied().filter(|v| v.kind() == EntityKind::PartitionVertex).collect();
    assert_eq!(pv.len(), 2, "{verts:?}");
    assert_eq!(verts.len(), 6, "4 host corners + 2 chord ends");
    assert!(verts.iter().all(|v| ov.is_alive(*v)));
    // A partitioned volume yields its live pieces.
    assert_eq!(m.group_volumes(gv, &ov).unwrap(), vec![n, p]);
    let vf = m.group_faces(gv, &ov).unwrap();
    assert!(vf.iter().all(|f| ov.is_alive(*f)) && vf.len() == 11, "{vf:?}");
}

fn lattice_model(pitch: f64, offset: f64) -> Model {
    let mut m = Model::new();
    m.add_lattice(Lattice::new(Layout::Hex { rings: 2, pitch, orient: HexOrient::XRows, nz: 1, dz: 1.0 }, sm_core::Transform::translation(Vector3::new(offset, 0.0, 0.0)), Fill::Void, Default::default()))
    .unwrap();
    m.add_boundary("mirror", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), offset).unwrap() })).unwrap();
    m.add_group("named", &[]).unwrap();
    m
}

fn identity_words(m: &Model, tol: &ModelTolerance) -> Vec<u64> {
    let mut v = Vec::new();
    m.canonical_visit(tol, &mut |w| {
        v.push(match w {
            HashWord::Int(x) => x,
            HashWord::Float { q, .. } => q as u64,
        })
    });
    v
}

fn exact_words(m: &Model, tol: &ModelTolerance) -> Vec<u64> {
    let mut v = Vec::new();
    m.canonical_visit(tol, &mut |w| {
        v.push(match w {
            HashWord::Int(x) => x,
            HashWord::Float { raw, .. } => raw.to_bits(),
        })
    });
    v
}

#[test]
fn sub_quantum_changes_keep_the_identity_words_and_change_the_exact_words() {
    let tol = ModelTolerance::DEFAULT; // quantum ~ 1e-9
    let a = lattice_model(1.26, 0.5);
    let b = lattice_model(1.26 + 1e-13, 0.5 + 1e-13);
    assert_eq!(identity_words(&a, &tol), identity_words(&b, &tol));
    assert_ne!(exact_words(&a, &tol), exact_words(&b, &tol));
    // A change of many quanta changes the identity words too.
    let c = lattice_model(1.26 + 1e-6, 0.5);
    assert_ne!(identity_words(&a, &tol), identity_words(&c, &tol));
    // Deterministic: the same model visits the same words.
    assert_eq!(exact_words(&a, &tol), exact_words(&a.clone(), &tol));
    // Every table contributes: a group rename changes the words.
    let mut d = a.clone();
    let g = d.group_by_name("named").unwrap();
    d.rename_group(g, "other").unwrap();
    assert_ne!(identity_words(&a, &tol), identity_words(&d, &tol));
}
