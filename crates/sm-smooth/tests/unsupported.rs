//! Unsupported inputs and frozen nodes: high-order records, polyhedral cells, curve ranges.

#![allow(clippy::needless_range_loop)]

mod common;

use sm_core::Point3;
use sm_model_api::Model;
use sm_mesh_api::{Basis, HighOrderCell, MeshEdit, MeshRead, PointIx, VertexIx};
use sm_smooth::{Method, NodeSelect, SmoothError, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice;
use sm_topo_api::CellComplex;

#[test]
fn a_high_order_record_is_unsupported() {
    // A 9-node (Lagrange, degree 2) quad on the first cell of the 2x2 sheet: dimension 2 needs no edge table.
    let f = lattice::quad([2, 2], [1.0; 2]).expect("quad");
    let mut mesh = f.mesh.clone();
    let corners: Vec<PointIx> = mesh.cell_corners(sm_mesh_api::CellIx::new(0)).expect("corners").iter().map(|v| (*v).into()).collect();
    let mut nodes = corners;
    for k in 0..5 {
        nodes.push(mesh.add_extra_node(Point3::new(0.1 * k as f64, 0.1, 0.0)).expect("extra"));
    }
    mesh.set_high_order(sm_mesh_api::CellIx::new(0), Some(HighOrderCell::new(2, Basis::LagrangeEquispaced, nodes))).expect("record");
    let input = SmoothInput { mesh: &mesh, topo: &f.topo, model: &f.model };
    let r = SmoothRequest { fix_boundary: false, ..SmoothRequest::default() };
    assert!(matches!(smooth(&input, &r), Err(SmoothError::Unsupported(_))));
}

/// A 3x3x3 block of unit cubes in which the cells (0,0,0) and (1,0,0) are polyhedra: their
/// shared face is split into two triangles, so each has 7 faces.
fn block_with_two_polyhedra() -> sm_mesh_api::reference::ReferenceMesh {
    use sm_testkit::meshes::{CellSpec, build_mesh};
    let pt = |i: usize, j: usize, k: usize| i + 4 * j + 16 * k;
    let mut pts = Vec::new();
    for k in 0..4 {
        for j in 0..4 {
            for i in 0..4 {
                // the interior nodes are nudged so smoothing has something to do
                let w = if (1..=2).contains(&i) && (1..=2).contains(&j) && (1..=2).contains(&k) { 0.1 } else { 0.0 };
                pts.push([i as f64 + w * (j as f64 - 1.5), j as f64 + w * (k as f64 - 1.5), k as f64 + w * (i as f64 - 1.5)]);
            }
        }
    }
    let corner = |i: usize, j: usize, k: usize, a: usize, b: usize, c: usize| pt(i + a, j + b, k + c);
    let mut cells = Vec::new();
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                let c = |a, b, d| corner(i, j, k, a, b, d);
                let hex = vec![c(0, 0, 0), c(1, 0, 0), c(1, 1, 0), c(0, 1, 0), c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)];
                if (j, k) == (0, 0) && i < 2 {
                    // x = i + 1 face split along (1,0,0)-(1,1,1) for the cell on its left, reversed for the right one
                    let left = i == 0;
                    let mut faces = vec![
                        vec![c(0, 0, 0), c(0, 0, 1), c(0, 1, 1), c(0, 1, 0)],
                        vec![c(0, 0, 0), c(1, 0, 0), c(1, 0, 1), c(0, 0, 1)],
                        vec![c(0, 1, 0), c(0, 1, 1), c(1, 1, 1), c(1, 1, 0)],
                        vec![c(0, 0, 0), c(0, 1, 0), c(1, 1, 0), c(1, 0, 0)],
                        vec![c(0, 0, 1), c(1, 0, 1), c(1, 1, 1), c(0, 1, 1)],
                    ];
                    if left {
                        faces[0] = vec![c(0, 0, 0), c(0, 0, 1), c(0, 1, 1), c(0, 1, 0)];
                        faces.push(vec![c(1, 0, 0), c(1, 1, 0), c(1, 1, 1)]);
                        faces.push(vec![c(1, 0, 0), c(1, 1, 1), c(1, 0, 1)]);
                    } else {
                        faces[0] = vec![c(0, 0, 0), c(0, 0, 1), c(0, 1, 1)];
                        faces.push(vec![c(0, 0, 0), c(0, 1, 1), c(0, 1, 0)]);
                        faces.push(vec![c(1, 0, 0), c(1, 1, 0), c(1, 1, 1), c(1, 0, 1)]);
                    }
                    cells.push(CellSpec::Poly(faces));
                } else {
                    cells.push(CellSpec::Template(sm_mesh_api::CellShape::Hex, hex));
                }
            }
        }
    }
    build_mesh(&pts, &cells, &mut sm_mesh_api::SerialAlloc::new()).expect("block with two polyhedra")
}

#[test]
fn a_polyhedral_cell_freezes_its_nodes() {
    let mesh = block_with_two_polyhedra();
    assert_eq!(mesh.cell_shape(sm_mesh_api::CellIx::new(0)), sm_mesh_api::CellShape::Polyhedron);
    let (topo, model) = (CellComplex::new(), Model::new());
    let input = SmoothInput { mesh: &mesh, topo: &topo, model: &model };
    let r = SmoothRequest { method: Method::Laplacian { relax: 1.0 }, iterations: 20, tolerance: 0.0, ..SmoothRequest::default() };
    let o = smooth(&input, &r).expect("smooth");
    // The interior nodes (1..=2)^3 are volume nodes; (1,1,1) and (2,1,1) belong to the polyhedra.
    assert_eq!(o.report.pinned.get("polyhedral"), Some(&2), "{:?}", o.report.pinned);
    assert_eq!(o.report.movable, 6);
    assert!(o.report.accepted > 0);
    for (i, j) in [(1usize, 1usize), (2, 1)] {
        let v = i + 4 * j + 16;
        let p = mesh.point(v);
        for (k, b) in [p.x, p.y, p.z].iter().enumerate() {
            assert_eq!(o.points[v][k].to_bits(), b.to_bits(), "polyhedral node {v} moved");
        }
    }
}

#[test]
fn a_move_beyond_the_end_of_an_edge_is_rejected() {
    // Edge node (2,0) of a 4x4 sheet; its two edge neighbours are placed at x = 10, so the
    // projected target leaves the bottom edge's parameter range.
    let f = lattice::quad([4, 4], [1.0; 2]).expect("quad");
    let mut mesh = f.mesh.clone();
    let node = VertexIx::new(2);
    for i in [1usize, 3] {
        mesh.set_point(PointIx::new(i), Point3::new(10.0, 0.0, 0.0)).expect("set");
    }
    let input = SmoothInput { mesh: &mesh, topo: &f.topo, model: &f.model };
    let r = SmoothRequest { nodes: NodeSelect::Vertices(vec![node]), iterations: 1, tolerance: 0.0, ..SmoothRequest::default() };
    let o = smooth(&input, &r).expect("smooth");
    assert_eq!((o.report.rejected_geometry, o.report.accepted), (1, 0));
    assert_eq!(o.points[2][0].to_bits(), mesh.point(2).x.to_bits());
}

#[test]
fn a_listed_face_of_a_surface_mesh_moves_its_edges_with_fix_boundary_false() {
    // lattice::quad meshes one face of a brick; the side faces above its edge nodes are unmeshed,
    // so the edges are in the closure of the listed face and move with it.
    let f = lattice::quad([4, 4], [1.0; 2]).expect("quad");
    let p = f.perturbed(2, 0.2).expect("perturb");
    let face = p.mesh.classification(sm_mesh_api::MeshEntityKind::Cell, 0).entity;
    let input = SmoothInput { mesh: &p.mesh, topo: &p.topo, model: &p.model };
    let run = |fix_boundary: bool| {
        let r = SmoothRequest { nodes: NodeSelect::Entities(vec![face]), fix_boundary, iterations: 30, tolerance: 0.0, ..SmoothRequest::default() };
        smooth(&input, &r).expect("smooth")
    };
    let on_edge = |i: usize| p.mesh.classification(sm_mesh_api::MeshEntityKind::Vertex, i as u32).entity.kind() == sm_core::EntityKind::TopoEdge;
    let (fixed, free) = (run(true), run(false));
    let moved = |o: &sm_smooth::SmoothOutcome| (0..o.points.len()).filter(|&i| on_edge(i) && o.points[i] != [p.mesh.point(i).x, p.mesh.point(i).y, p.mesh.point(i).z]).count();
    assert_eq!(moved(&fixed), 0, "fix_boundary keeps the edge nodes");
    assert!(moved(&free) > 0, "edge nodes slide along their edges");
}
