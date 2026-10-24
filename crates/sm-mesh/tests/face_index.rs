//! Regression: the lazy face index must keep coincident faces in creation order across table
//! growth (a wrapped probe chain used to be rehashed tail-first).

use sm_core::Point3;
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::*;

fn triangles(nv: u32, want: usize) -> Vec<[VertexIx; 3]> {
    let mut out = Vec::new();
    'o: for a in 0..nv {
        for b in a + 1..nv {
            for c in b + 1..nv {
                out.push([VertexIx(a), VertexIx(b), VertexIx(c)]);
                if out.len() == want {
                    break 'o;
                }
            }
        }
    }
    out
}

fn fill<B: MeshBuild>(b: &mut B, nv: u32) {
    for i in 0..nv {
        b.add_vertex(Point3::new(f64::from(i), f64::from(i * i % 17), 0.0)).unwrap();
    }
}

fn compare(r: &ReferenceBuilder, m: &MeshBuilder, tris: &[[VertexIx; 3]], what: &str) {
    for t in tris {
        let mut rev = *t;
        rev.reverse();
        for l in [t, &rev] {
            assert_eq!(r.find_face(l), m.find_face(l), "{what}: find_face {l:?}");
            // Accept only the second-or-later coincident face, and only a given one.
            for skip in [1usize, 2] {
                let (mut kr, mut km) = (0, 0);
                let a = r.find_face_where(l, &mut |_, _| {
                    kr += 1;
                    kr > skip
                });
                let b = m.find_face_where(l, &mut |_, _| {
                    km += 1;
                    km > skip
                });
                assert_eq!(a, b, "{what}: find_face_where {l:?} skip {skip}");
            }
        }
    }
}

fn run(nv: u32, ntri: usize) {
    let tris = triangles(nv, ntri);
    assert!(tris.len() > 100);
    let (mut r, mut m) = (ReferenceBuilder::new(), MeshBuilder::new());
    fill(&mut r, nv);
    fill(&mut m, nv);
    // Each triangle twice in a row (a baffle pair, the second with the opposite orientation).
    for t in &tris {
        let mut rev = *t;
        rev.reverse();
        for l in [*t, rev] {
            assert_eq!(r.add_face(&l).unwrap(), m.add_face(&l).unwrap());
        }
    }
    compare(&r, &m, &tris, "after grows");
    // Roll back, re-add and compare again.
    let keep = tris.len() / 2;
    r.truncate_faces(keep).unwrap();
    m.truncate_faces(keep).unwrap();
    compare(&r, &m, &tris, "after truncate");
    for t in &tris {
        assert_eq!(r.add_face(t).unwrap(), m.add_face(t).unwrap());
    }
    compare(&r, &m, &tris, "after re-add");
    // A polyhedron onto a baffle pair: a tetrahedron whose faces all exist twice.
    let (a, b, c, d) = (tris[0][0], tris[0][1], tris[0][2], VertexIx(nv - 1));
    let tet: [&[VertexIx]; 4] = [&[a, c, b], &[a, b, d], &[b, c, d], &[c, a, d]];
    for t in tet {
        r.add_face(t).unwrap();
        m.add_face(t).unwrap();
    }
    let (x, y) = (r.add_polyhedron(&tet), m.add_polyhedron(&tet));
    assert_eq!(x, y);
    assert_eq!(r.face_count(), m.face_count());
    for f in 0..r.face_count() {
        assert_eq!(r.face_degree(FaceIx::new(f)), m.face_degree(FaceIx::new(f)), "degree of face {f}");
    }
}

#[test]
fn coincident_faces_keep_creation_order_across_growth() {
    run(97, 1450);
}

#[test]
fn coincident_faces_keep_creation_order_with_many_vertices() {
    run(1009, 6000);
}
