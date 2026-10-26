//! Acceptance 5: two volumes sharing an interface.

mod common;
use common::*;
use sm_mesh_api::CellShape;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::meshes::{CellSpec, build_mesh};
use sm_testkit::surf3;

#[test]
fn two_cubes_shared_interface_is_conforming() {
    let f = surf3::two_cubes_shared(4);
    let a = tetrahedralize(&input_of(&f, 0), &TetOptions::default()).unwrap();
    let b = tetrahedralize(&input_of(&f, 1), &TetOptions::default()).unwrap();
    let total = total_volume(&a) + total_volume(&b);
    assert!((total - 2.0).abs() < 1e-12 * 2.0, "total {total}");
    check_boundary(&a, &input_of(&f, 0).tris).unwrap();
    check_boundary(&b, &input_of(&f, 1).tris).unwrap();
    // Shared interface triangles: exactly one tet on each side with identical point ids/coordinates.
    let ia: std::collections::BTreeSet<u32> = f.volumes[0].tris.iter().map(|x| x.0).collect();
    let shared: Vec<u32> = f.volumes[1].tris.iter().map(|x| x.0).filter(|t| ia.contains(t)).collect();
    assert_eq!(shared.len(), 2 * 4 * 4);
    for t in &shared {
        for &i in &f.tris[*t as usize] {
            let pa = a.points[i as usize];
            let pb = b.points[i as usize];
            assert_eq!(pa.map(f64::to_bits), pb.map(f64::to_bits));
        }
    }
    // Union: the two meshes share the input points; Steiner points are renumbered after them.
    let n = f.points.len();
    let mut points = a.points.clone();
    let shift = (points.len() - n) as u32;
    points.extend_from_slice(&b.points[n..]);
    let mut cells: Vec<CellSpec> = a.tets.iter().map(|t| CellSpec::Template(CellShape::Tet, t.iter().map(|&i| i as usize).collect())).collect();
    for t in &b.tets {
        cells.push(CellSpec::Template(CellShape::Tet, t.iter().map(|&i| if (i as usize) < n { i as usize } else { (i + shift) as usize }).collect()));
    }
    let mesh = build_mesh(&points, &cells, &mut sm_mesh_api::SerialAlloc::new()).unwrap();
    sm_testkit::check::watertight(&mesh).unwrap();
    sm_testkit::check::mesh_valid(&mesh).unwrap();
}
