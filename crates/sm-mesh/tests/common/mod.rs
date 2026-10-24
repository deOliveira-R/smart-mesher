//! Shared fixtures for the integration tests.
#![allow(dead_code)]

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
pub mod replay;
pub mod shim;
use shim::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;

pub fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

pub fn on(kind: EntityKind, n: u64) -> Classification {
    Classification::on(id(kind, n))
}

pub fn vol() -> Classification {
    on(EntityKind::TopoVolume, 1)
}

/// Reference-shape points (template order) for each template shape.
pub fn template_points(shape: CellShape) -> Vec<[f64; 3]> {
    match shape {
        CellShape::Tet => vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        CellShape::Wedge => vec![
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1.],
            [1., 0., 1.],
            [0., 1., 1.],
        ],
        CellShape::Pyramid => vec![
            [0., 0., 0.],
            [1., 0., 0.],
            [1., 1., 0.],
            [0., 1., 0.],
            [0.5, 0.5, 1.],
        ],
        CellShape::Hex => vec![
            [0., 0., 0.],
            [1., 0., 0.],
            [1., 1., 0.],
            [0., 1., 0.],
            [0., 0., 1.],
            [1., 0., 1.],
            [1., 1., 1.],
            [0., 1., 1.],
        ],
        _ => {
            unreachable!()
        }
    }
}

pub fn analytic_volume(shape: CellShape) -> f64 {
    match shape {
        CellShape::Tet => 1.0 / 6.0,
        CellShape::Wedge => 0.5,
        CellShape::Pyramid => 1.0 / 3.0,
        CellShape::Hex => 1.0,
        _ => {
            unreachable!()
        }
    }
}

/// One template cell with every vertex on its own `TopoVertex` and every face on its own `TopoFace`.
pub fn single_cell(shape: CellShape, alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(shape)
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(
                MeshEntityRef::Vertex(v),
                on(EntityKind::TopoVertex, 1 + i as u64),
            )
            .unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(shape, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    b.finish(alloc).unwrap().mesh
}

pub fn translate(dx: f64, dy: f64, dz: f64) -> Transform {
    Transform {
        t: Vector3::new(dx, dy, dz),
        ..Transform::IDENTITY
    }
}

/// Mirror across the plane `x = x0`.
pub fn mirror_x(x0: f64) -> Transform {
    mirror_transform(Point3::new(x0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0))
}

pub fn mirror_y(y0: f64) -> Transform {
    mirror_transform(Point3::new(0.0, y0, 0.0), Vector3::new(0.0, 1.0, 0.0))
}

pub fn centre(m: &dyn MeshRead, c: CellIx) -> Point3 {
    cell_volume_centroid(m, c).1
}

pub fn total_volume(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count())
        .map(|c| cell_volume(m, CellIx::new(c)))
        .sum()
}

pub fn assert_valid(m: &dyn MeshRead) {
    let v = validate(m);
    assert!(v.is_empty(), "validate: {v:?}");
    let g = validate_geometry(m, &GeomTol::default());
    assert!(g.is_empty(), "validate_geometry: {g:?}");
}

pub fn interior_faces(m: &dyn MeshRead) -> Vec<FaceIx> {
    (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.face_kind(f) == FaceKind::Interior)
        .collect()
}

/// Classify every template edge of a cell on its own `TopoEdge` (a brick's edges lie on two
/// model faces, so the edge table needs them explicitly; high-order meshes carry one).
#[allow(dead_code)]
pub fn classify_template_edges(b: &mut ReferenceBuilder, shape: CellShape, vs: &[VertexIx]) {
    for (i, [a, c]) in shape.local_edges().iter().enumerate() {
        b.set_class(MeshEntityRef::Edge(vs[*a as usize], vs[*c as usize]), on(EntityKind::TopoEdge, 1 + i as u64)).unwrap();
    }
}
