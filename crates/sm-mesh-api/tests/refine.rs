//! Refine-one-hex: lineage columns and non-conformal face hooks.

mod common;
use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;

/// Hex A = [0,1]^3 (inactive, refined into 8 children) next to hex B = [1,2]x[0,1]^2.
/// A and B share their conforming face; the four small child faces on x = 1 are non-conformal
/// one-cell faces. `nc_kind` controls how those faces are marked, `a_active` the parent flag.
fn refined(nc_kind: Option<FaceKind>, a_active: bool) -> Result<(ReferenceMesh, [CellIx; 2])> {
    let mut b = ReferenceBuilder::new();
    let lat = |i: usize, j: usize, k: usize| VertexIx::new((k * 3 + j) * 3 + i);
    let mut n = 0u64;
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                let v =
                    b.add_vertex(Point3::new(0.5 * i as f64, 0.5 * j as f64, 0.5 * k as f64))?;
                n += 1;
                b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, n))?;
            }
        }
    }
    let bx: Vec<VertexIx> = [
        (2.0, 0.0, 0.0),
        (2.0, 1.0, 0.0),
        (2.0, 1.0, 1.0),
        (2.0, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| {
        n += 1;
        let v = b.add_vertex(Point3::new(x, y, z)).unwrap();
        b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, n))
            .unwrap();
        v
    })
    .collect();
    // Cell 0: A, cell 1: B, cells 2..10: children of A.
    let a = [
        lat(0, 0, 0),
        lat(2, 0, 0),
        lat(2, 2, 0),
        lat(0, 2, 0),
        lat(0, 0, 2),
        lat(2, 0, 2),
        lat(2, 2, 2),
        lat(0, 2, 2),
    ];
    let ca = b.add_cell_from_vertices(CellShape::Hex, &a)?;
    let bv = [a[1], bx[0], bx[1], a[2], a[5], bx[3], bx[2], a[6]];
    let cb = b.add_cell_from_vertices(CellShape::Hex, &bv)?;
    let mut boundary = vec![];
    let mut ncs = vec![];
    let plane = |sf: &SignedFace, b: &ReferenceBuilder| -> Option<u64> {
        // Which model face plane does a (builder) face lie on, if any? Same numbering as hex_block.
        let l = b.face_loop(sf.face());
        let same = |g: &dyn Fn(Point3) -> f64, t: f64| {
            l.iter().all(|&v| (g(b.vertex_point(v)) - t).abs() < 1e-12)
        };
        if same(&|p| p.x, 0.0) {
            Some(1)
        } else if same(&|p| p.x, 2.0) {
            Some(2)
        } else if same(&|p| p.y, 0.0) {
            Some(3)
        } else if same(&|p| p.y, 1.0) {
            Some(4)
        } else if same(&|p| p.z, 0.0) {
            Some(5)
        } else if same(&|p| p.z, 1.0) {
            Some(6)
        } else {
            None
        }
    };
    let mut children = Vec::new();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..2 {
                let vs = [
                    lat(i, j, k),
                    lat(i + 1, j, k),
                    lat(i + 1, j + 1, k),
                    lat(i, j + 1, k),
                    lat(i, j, k + 1),
                    lat(i + 1, j, k + 1),
                    lat(i + 1, j + 1, k + 1),
                    lat(i, j + 1, k + 1),
                ];
                let ch = b.add_cell_from_vertices(CellShape::Hex, &vs)?;
                children.push(ch.cell);
                for sf in &ch.faces {
                    // A child face on x = 1 faces B: non-conformal. Interior child-child faces have two cells.
                    let l = b.face_loop(sf.face());
                    if l.iter().all(|&v| (b.vertex_point(v).x - 1.0).abs() < 1e-12) {
                        ncs.push(sf.face());
                    } else if let Some(p) = plane(sf, &b) {
                        boundary.push((sf.face(), p));
                    }
                }
            }
        }
    }
    for ch in [&ca, &cb] {
        for sf in &ch.faces {
            if let Some(p) = plane(sf, &b) {
                boundary.push((sf.face(), p));
            }
        }
    }
    for (f, p) in boundary {
        b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoFace, p))?;
    }
    for c in [ca.cell, cb.cell]
        .into_iter()
        .chain(children.iter().copied())
    {
        b.set_class(MeshEntityRef::Cell(c), vol())?;
    }
    for &f in &ncs {
        match nc_kind {
            Some(k) => b.set_face_kind(f, k)?,
            // Negative control: leave the faces as model boundary, classified on the volume.
            None => b.set_class(MeshEntityRef::Face(f), vol())?,
        }
    }
    for &c in &children {
        b.set_cell_parent(c, ca.cell)?;
    }
    b.set_cell_active(ca.cell, a_active)?;
    let fin = b.finish(&mut SerialAlloc::new())?;
    Ok((fin.mesh, [ca.cell, cb.cell]))
}

#[test]
fn refined_hex_has_lineage_and_nonconformal_faces() {
    let (m, [a, bcell]) = refined(Some(FaceKind::NonConformal), false).unwrap();
    assert_valid(&m);
    assert_eq!(m.cell_count(), 10);
    // Lineage.
    assert!(!m.cell_active(a) && m.cell_active(bcell));
    assert_eq!(m.cell_parent(a), None);
    let kids: Vec<CellIx> = (0..10)
        .map(CellIx::new)
        .filter(|&c| m.cell_parent(c) == Some(a))
        .collect();
    assert_eq!(kids.len(), 8);
    assert!(kids.iter().all(|&c| m.cell_active(c) && c > a));
    // Non-conformal faces: four one-cell faces, not boundary, classified on the volume.
    let nc: Vec<FaceIx> = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.face_kind(f) == FaceKind::NonConformal)
        .collect();
    assert_eq!(nc.len(), 4);
    for &f in &nc {
        assert!(m.face_right(f).is_none() && !m.is_boundary_face(f));
        assert_eq!(m.face_class(f).entity, vol().entity);
        assert!(kids.contains(&m.face_left(f)));
        assert_eq!(m.across(SignedFace::new(f, Side::Left)), None);
    }
    // The sub-face areas add up to the coarse face they replace (the hook for interface records).
    let sub: f64 = nc.iter().map(|&f| face_area_vector(&m, f).norm()).sum();
    let shared = interior_faces(&m)
        .into_iter()
        .find(|&f| m.face_cells(f) == (a, Some(bcell)))
        .unwrap();
    assert!((sub - face_area_vector(&m, shared).norm()).abs() < 1e-14);
    // Active volume is conserved; summing over all cells double-counts the refined parent.
    let active: f64 = (0..10)
        .map(CellIx::new)
        .filter(|&c| m.cell_active(c))
        .map(|c| cell_volume(&m, c))
        .sum();
    assert!((active - 2.0).abs() < 1e-13);
    assert!((total_volume(&m) - 3.0).abs() < 1e-13);
}

#[test]
fn nonconformal_marking_is_checked() {
    // Unmarked, the hanging faces are model boundary faces on a volume: validate rejects it.
    let err = refined(None, false).unwrap_err();
    match err {
        MeshError::Invalid(v) => assert!(
            v.iter().any(|x| x.code == ViolationCode::ClassKind),
            "{v:?}"
        ),
        e => panic!("{e:?}"),
    }
    // An active parent with active children breaks the lineage rule.
    match refined(Some(FaceKind::NonConformal), true).unwrap_err() {
        MeshError::Invalid(v) => {
            assert!(v.iter().any(|x| x.code == ViolationCode::Lineage), "{v:?}")
        }
        e => panic!("{e:?}"),
    }
    // PartitionCut is accepted the same way.
    let (m, _) = refined(Some(FaceKind::PartitionCut), false).unwrap();
    assert_valid(&m);
    assert_eq!(
        (0..m.face_count())
            .filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::PartitionCut)
            .count(),
        4
    );
    // MeshEdit can flip a one-cell face between Boundary/NonConformal/PartitionCut but never to Interior.
    let mut m = m;
    let f = (0..m.face_count())
        .map(FaceIx::new)
        .find(|&f| m.face_kind(f) == FaceKind::PartitionCut)
        .unwrap();
    MeshEdit::set_face_kind(&mut m, f, FaceKind::NonConformal).unwrap();
    assert!(MeshEdit::set_face_kind(&mut m, f, FaceKind::Interior).is_err());
    assert!(
        MeshEdit::set_face_kind(&mut m, f, FaceKind::Boundary).is_err(),
        "classified on a volume, not a TopoFace"
    );
    let inner = interior_faces(&m)[0];
    assert!(MeshEdit::set_face_kind(&mut m, inner, FaceKind::NonConformal).is_err());
}
