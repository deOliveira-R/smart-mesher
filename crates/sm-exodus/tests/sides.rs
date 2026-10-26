//! Acceptance 3 and 4: side numbering verified by geometry, against hand-typed Exodus tables.

mod common;

use common::*;
use proptest::prelude::*;
use sm_exodus::read::read_exodus;
use sm_exodus::{ExodusOptions, exodus_bytes};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::*;
use sm_testkit::meshes;

type P = [f64; 3];

fn close(a: P, b: P) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-12)
}

/// Write the single cell with a side set holding all its faces; return (side, centroid of the
/// mesh face) pairs as the writer assigned them.
fn assigned_sides(fx: &meshes::MeshFixture) -> Vec<(i64, P)> {
    let mut m = fx.mesh.clone();
    let all = all_sides_of(&m, 0..1);
    add_set(&mut m, &mut SerialAlloc::new(), "all", Some(1), sides(&all));
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    assert_eq!(d.sidesets.len(), 1);
    let ss = &d.sidesets[0];
    // Entries are in set order (ascending raw word = ascending face).
    assert_eq!(ss.elems.len(), all.len());
    all.iter()
        .zip(&ss.sides)
        .map(|(sf, side)| {
            let pts: Vec<P> = m.face_vertices(sf.face()).iter().map(|v| {
                let c = m.coords();
                [c.x[v.get()], c.y[v.get()], c.z[v.get()]]
            }).collect();
            (*side, mean(&pts))
        })
        .collect()
}

fn check_centroids(fx: &meshes::MeshFixture, expected: &[P]) {
    let got = assigned_sides(fx);
    assert_eq!(got.len(), expected.len());
    let mut seen = vec![false; expected.len()];
    for (side, centroid) in got {
        let s = side as usize;
        assert!((1..=expected.len()).contains(&s), "side {s}");
        assert!(!seen[s - 1], "side {s} twice");
        seen[s - 1] = true;
        assert!(close(centroid, expected[s - 1]), "{}: side {s} centroid {centroid:?} != {:?}", fx.name, expected[s - 1]);
    }
    assert!(seen.iter().all(|x| *x));
}

#[test]
fn acc3_hex_sides_by_centroid() {
    check_centroids(
        &meshes::single_hex().unwrap(),
        &[[0.5, 0.0, 0.5], [1.0, 0.5, 0.5], [0.5, 1.0, 0.5], [0.0, 0.5, 0.5], [0.5, 0.5, 0.0], [0.5, 0.5, 1.0]],
    );
}

#[test]
fn acc3_tet_sides_by_centroid() {
    let t = 1.0 / 3.0;
    // s1 y=0, s2 slanted x+y+z=1, s3 x=0, s4 z=0.
    check_centroids(&meshes::single_tet().unwrap(), &[[t, 0.0, t], [t, t, t], [0.0, t, t], [t, t, 0.0]]);
}

#[test]
fn acc3_wedge_sides_by_centroid() {
    let t = 1.0 / 3.0;
    // s1 y=0, s2 slanted x+y=1, s3 x=0, s4 z=0, s5 z=1.
    check_centroids(
        &meshes::single_wedge().unwrap(),
        &[[0.5, 0.0, 0.5], [0.5, 0.5, 0.5], [0.0, 0.5, 0.5], [t, t, 0.0], [t, t, 1.0]],
    );
}

#[test]
fn acc3_pyramid_sides_by_centroid() {
    let t = 1.0 / 3.0;
    // s1..s4 triangles around the apex (.5,.5,1), s5 the base.
    check_centroids(
        &meshes::single_pyramid().unwrap(),
        &[
            [0.5, 0.5 / 3.0, t], // nodes (0,0,0) (1,0,0) (.5,.5,1): only the base edge lies on y=0
            [(1.0 + 1.0 + 0.5) / 3.0, 0.5, t],
            [0.5, (1.0 + 1.0 + 0.5) / 3.0, t],
            [0.5 / 3.0, 0.5, t],
            [0.5, 0.5, 0.0],
        ],
    );
}

#[test]
fn acc3_template_corner_order_is_the_exodus_order() {
    // Unit elements as drawn in the Exodus manual: connect = 1..n with these coordinates.
    let cases: [(&str, meshes::MeshFixture, Vec<P>); 4] = [
        (
            "HEX8",
            meshes::single_hex().unwrap(),
            vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]],
        ),
        ("TET4", meshes::single_tet().unwrap(), vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
        (
            "WEDGE6",
            meshes::single_wedge().unwrap(),
            vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]],
        ),
        (
            "PYRAMID5",
            meshes::single_pyramid().unwrap(),
            vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.]],
        ),
    ];
    for (ty, fx, want) in cases {
        let d = read_exodus(&exodus_bytes(&fx.mesh, &ExodusOptions::default()).unwrap()).unwrap();
        assert_eq!(d.blocks[0].elem_type, ty);
        let conn: Vec<i64> = (1..=want.len() as i64).collect();
        assert_eq!(d.blocks[0].connectivity, conn, "{ty}");
        for (i, p) in want.iter().enumerate() {
            assert_eq!(node_xyz(&d, i), *p, "{ty} node {}", i + 1);
        }
    }
}

/// Acceptance 4 for one mesh: every (cell, face) of the template cells, as Left and Right entries.
fn check_side_loops(mesh: &ReferenceMesh) {
    let cells: Vec<usize> = (0..mesh.cell_count()).filter(|c| !mesh.cell_shape(CellIx::new(*c)).is_general()).collect();
    if cells.is_empty() {
        return;
    }
    let all = all_sides_of(mesh, cells.iter().copied());
    let mut m = mesh.clone();
    add_set(&mut m, &mut SerialAlloc::new(), "all", Some(1), sides(&all));
    let d = read_exodus(&exodus_bytes(&m, &ExodusOptions::default()).unwrap()).unwrap();
    let ss = &d.sidesets[0];
    assert_eq!(ss.elems.len(), all.len());
    for (i, sf) in all.iter().enumerate() {
        let e = ss.elems[i] as usize - 1;
        let cell = d.elem_num_map[e] as usize - 1;
        // Locate the element's connectivity row in its block.
        let (blk, row) = {
            let mut start = 0;
            let mut found = None;
            for b in &d.blocks {
                if e < start + b.num_elem {
                    found = Some((b, e - start));
                    break;
                }
                start += b.num_elem;
            }
            found.expect("block")
        };
        let npe = blk.connectivity.len() / blk.num_elem;
        let conn = &blk.connectivity[row * npe..(row + 1) * npe];
        let nodes = manual_side_nodes(&blk.elem_type, ss.sides[i] as usize);
        let loop0: Vec<u32> = nodes.iter().map(|n| (conn[n - 1] - 1) as u32).collect();
        // Same cycle as face_vertices, same orientation for Left, reversed for Right.
        let fv = m.face_vertices(sf.face());
        let want = if sf.side() == Side::Left { LoopRelation::Same } else { LoopRelation::Reversed };
        let loop_v: Vec<VertexIx> = loop0.iter().map(|v| VertexIx(*v)).collect();
        assert_eq!(loop_relation(fv, &loop_v), Some(want), "cell {cell} face {:?}", sf.face());
        // Outward: the Exodus side loop's normal points away from the cell centroid.
        if loop0.len() >= 3 {
            let pts: Vec<P> = loop0.iter().map(|v| node_xyz(&d, *v as usize)).collect();
            let corners: Vec<P> = conn.iter().map(|v| node_xyz(&d, (*v - 1) as usize)).collect();
            let out = dot(newell(&pts), sub(mean(&pts), mean(&corners)));
            assert!(out > 0.0, "cell {cell} side {} not outward ({out})", ss.sides[i]);
        }
    }
}

#[test]
fn acc4_all_fixtures() {
    for fx in meshes::all_mesh_fixtures().unwrap() {
        check_side_loops(&fx.mesh);
    }
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(16))]
    #[test]
    fn acc4_random_hex_blocks(spec in sm_testkit::strategies::hex_block_spec()) {
        check_side_loops(&spec.build().unwrap());
    }
}
