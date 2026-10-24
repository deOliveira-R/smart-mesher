//! Welded vertices that end up interior are reclassified on the volume.

mod common;
use common::*;
use sm_mesh_api::reference::{ReferenceBuilder, hex_block};
use sm_mesh_api::*;

#[test]
fn welded_vertices_that_become_interior_are_reclassified_on_the_volume() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([1, 1, 2], [1.0; 3], &mut alloc).unwrap();
    let opts = FlattenOptions::new().with_weld_tol(1e-9);
    let (u, _) = unfold(
        &m,
        &[mirror_x(0.0), mirror_y(0.0)],
        ReferenceBuilder::new(),
        &opts,
        &mut alloc,
    )
    .unwrap();
    assert_valid(&u);
    // The column-axis vertex at mid height lies on a model edge in the prototype but is interior now.
    let find = |z: f64| {
        (0..u.vertex_count())
            .find(|&v| u.point(v).x == 0.0 && u.point(v).y == 0.0 && u.point(v).z == z)
            .unwrap()
    };
    assert_eq!(
        u.vertex_class(VertexIx::new(find(1.0))).entity,
        vol().entity
    );
    // The axis vertices at the caps are still on the boundary: they keep their (identical) classes.
    assert_ne!(
        u.vertex_class(VertexIx::new(find(0.0))).entity,
        vol().entity
    );
    assert_eq!(u.vertex_count(), 3 * 3 * 3);
}
