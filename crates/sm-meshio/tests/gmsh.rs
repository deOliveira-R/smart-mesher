//! Gmsh MSH 4.1 export: round trip through `parse_msh4` (ASCII and binary), element orientation
//! and volume by Jacobians, entity splitting by cell sets, polyhedra policies, determinism,
//! property tests and corrupted input.

mod common;

use proptest::prelude::*;
use sm_core::EntityKind;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::*;
use sm_meshio::gmsh::*;
use sm_testkit::meshes::{CellSpec, MeshFixture, SetSpec, build_mesh, dual_patch, hex_block_2x2x2, hex_block_two_volumes, hexagonal_prism, mixed_transition, single_hex, single_pyramid, single_tet, single_wedge, split_face_cube, tet_block_two_volumes, truncated_octahedron, with_sets};
use sm_testkit::strategies;
use std::collections::BTreeMap;

type V3 = [f64; 3];

fn write(mesh: &dyn MeshRead, o: &GmshOptions) -> (Vec<u8>, GmshReport) {
    let mut out = Vec::new();
    let r = write_msh4(mesh, o, &mut out).unwrap();
    (out, r)
}

fn opts(binary: bool) -> GmshOptions {
    GmshOptions::default().with_binary(binary)
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn det(a: V3, b: V3, c: V3) -> f64 {
    let n = cross(b, c);
    a[0] * n[0] + a[1] * n[1] + a[2] * n[2]
}

/// Signed volume of one element from its nodes, with every Jacobian determinant that fixes its
/// orientation: `(volume, smallest determinant)`. Tets use `det / 6`; hexahedra the 2x2x2 Gauss
/// rule over the trilinear map (reference cube `[-1,1]^3`, Gmsh node order); wedges the affine
/// Jacobian at the centroid and at the six corners; pyramids two tets over the base diagonal.
fn element_volume(ty: u8, p: &[V3]) -> (f64, f64) {
    match ty {
        4 => {
            let d = det(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[3], p[0]));
            (d / 6.0, d)
        }
        5 => {
            const R: [[f64; 3]; 8] = [[-1., -1., -1.], [1., -1., -1.], [1., 1., -1.], [-1., 1., -1.], [-1., -1., 1.], [1., -1., 1.], [1., 1., 1.], [-1., 1., 1.]];
            let jac = |x: [f64; 3]| -> f64 {
                let mut j = [[0.0; 3]; 3];
                for (i, r) in R.iter().enumerate() {
                    let f = [1.0 + x[0] * r[0], 1.0 + x[1] * r[1], 1.0 + x[2] * r[2]];
                    let d = [r[0] * f[1] * f[2] / 8.0, f[0] * r[1] * f[2] / 8.0, f[0] * f[1] * r[2] / 8.0];
                    for a in 0..3 {
                        for b in 0..3 {
                            j[b][a] += d[b] * p[i][a];
                        }
                    }
                }
                det(j[0], j[1], j[2])
            };
            let g = 1.0 / 3.0_f64.sqrt();
            let (mut v, mut m) = (0.0, f64::INFINITY);
            for sx in [-g, g] {
                for sy in [-g, g] {
                    for sz in [-g, g] {
                        let d = jac([sx, sy, sz]);
                        v += d;
                        m = m.min(d);
                    }
                }
            }
            for r in R {
                m = m.min(jac(r));
            }
            (v, m)
        }
        6 => {
            // x(xi, eta, zeta) = sum_k L_k(xi, eta) ((1 - zeta)/2 b_k + (1 + zeta)/2 t_k)
            let jac = |xi: f64, eta: f64, ze: f64| -> f64 {
                let l = [1.0 - xi - eta, xi, eta];
                let dl = [[-1.0, -1.0], [1.0, 0.0], [0.0, 1.0]];
                let mut j = [[0.0; 3]; 3];
                for k in 0..3 {
                    for a in 0..3 {
                        let (b, t) = (p[k][a], p[k + 3][a]);
                        let mid = (1.0 - ze) / 2.0 * b + (1.0 + ze) / 2.0 * t;
                        j[0][a] += dl[k][0] * mid;
                        j[1][a] += dl[k][1] * mid;
                        j[2][a] += l[k] * (t - b) / 2.0;
                    }
                }
                det(j[0], j[1], j[2])
            };
            let mut m = f64::INFINITY;
            for (xi, eta) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)] {
                for ze in [-1.0, 1.0] {
                    m = m.min(jac(xi, eta, ze));
                }
            }
            // Affine prism: volume = detJ(centroid) * (1/2) * 2.
            (jac(1.0 / 3.0, 1.0 / 3.0, 0.0), m)
        }
        7 => {
            let d1 = det(sub(p[1], p[0]), sub(p[2], p[0]), sub(p[4], p[0]));
            let d2 = det(sub(p[2], p[0]), sub(p[3], p[0]), sub(p[4], p[0]));
            ((d1 + d2) / 6.0, d1.min(d2))
        }
        _ => (0.0, f64::INFINITY),
    }
}

/// Total volume and the smallest determinant over the dimension-3 elements of a parsed file.
fn jacobian_volume(d: &Msh4Data) -> (f64, f64) {
    let coords: BTreeMap<u64, V3> = d.node_blocks.iter().flat_map(|b| b.tags.iter().copied().zip(b.coords.iter().copied())).collect();
    let (mut total, mut min) = (0.0, f64::INFINITY);
    for b in d.element_blocks.iter().filter(|b| b.dim == 3) {
        for i in 0..b.tags.len() {
            let p: Vec<V3> = b.element(i).iter().map(|t| coords[t]).collect();
            let (v, m) = element_volume(b.etype, &p);
            total += v;
            min = min.min(m);
        }
    }
    (total, min)
}

fn round_trip(mesh: &dyn MeshRead, o: &GmshOptions) -> (Msh4Data, GmshReport) {
    let a = o.clone().with_binary(false);
    let b = o.clone().with_binary(true);
    let (ta, ra) = write(mesh, &a);
    let (tb, rb) = write(mesh, &b);
    let (pa, pb) = (parse_msh4(&ta).unwrap(), parse_msh4(&tb).unwrap());
    assert!(!pa.binary && pb.binary);
    assert_eq!(Msh4Data { binary: false, ..pb.clone() }, pa, "ASCII and binary carry identical content");
    assert_eq!((ra.nodes, &ra.elements_by_type, ra.entities, ra.physical_groups), (rb.nodes, &rb.elements_by_type, rb.entities, rb.physical_groups));
    assert_eq!(pa.node_count() as u64, ra.nodes);
    assert_eq!(pa.elements_by_type(), ra.elements_by_type);
    assert_eq!(ra.bytes as usize, ta.len());
    for (dim, n) in ra.entities.iter().enumerate() {
        assert_eq!(pa.entities[dim].len() as u64, *n);
    }
    (pa, ra)
}

#[test]
fn each_shape_round_trips_with_positive_jacobians_and_analytic_volume() {
    let fixtures: Vec<MeshFixture> = vec![single_hex().unwrap(), single_tet().unwrap(), single_wedge().unwrap(), single_pyramid().unwrap(), mixed_transition().unwrap(), hex_block_2x2x2().unwrap(), tet_block_two_volumes([3, 2, 2], [0.3, 0.2, 0.7], 1).unwrap()];
    for f in &fixtures {
        let (d, r) = round_trip(&f.mesh, &GmshOptions::default());
        let (vol, min) = jacobian_volume(&d);
        assert!(min > 0.0, "{}: smallest Jacobian determinant {min}", f.name);
        assert!((vol - f.volume).abs() <= 1e-12 * f.volume, "{}: {vol} vs {}", f.name, f.volume);
        let cells = r.elements_by_type.iter().filter(|(t, _)| [4u8, 5, 6, 7].contains(t)).map(|(_, n)| n).sum::<u64>();
        assert_eq!(cells as usize, f.mesh.cell_count(), "{}", f.name);
        assert_eq!(r.nodes as usize, f.mesh.vertex_count());
    }
    let (d, r) = round_trip(&mixed_transition().unwrap().mesh, &GmshOptions::default());
    for t in [4u8, 5, 6, 7] {
        assert_eq!(r.elements_by_type[&t], 1, "one cell of type {t}");
    }
    assert_eq!(r.elements_by_type[&2] + r.elements_by_type[&3], 14, "boundary faces: 8 triangles and 6 quads");
    let (_, min) = jacobian_volume(&d);
    assert!(min > 0.0);
}

#[test]
fn entities_physical_groups_and_boundaries() {
    let f = hex_block_two_volumes([2, 2, 2], [1.0; 3], 1).unwrap();
    let (d, r) = round_trip(&f.mesh, &GmshOptions::default());
    assert_eq!(d.physical_names, vec![(3, 1, "volume_1".to_string()), (3, 2, "volume_2".to_string())]);
    assert_eq!(r.entities[2], 11);
    assert_eq!(r.entities[3], 2);
    assert_eq!(d.entities[3][0].physical, vec![1]);
    assert_eq!(d.entities[3][1].physical, vec![2]);
    // Every volume lists its surfaces; the interface appears with opposite signs.
    let (b1, b2) = (&d.entities[3][0].boundary, &d.entities[3][1].boundary);
    assert_eq!((b1.len(), b2.len()), (6, 6));
    let common: Vec<i32> = b1.iter().filter(|t| b2.contains(&-**t)).copied().collect();
    assert_eq!(common.len(), 1);
    // Each surface entity holds the quads of one TopoFace: 24 skin quads + 4 interface quads = 28.
    let quads: usize = d.element_blocks.iter().filter(|b| b.etype == 3).map(|b| b.tags.len()).sum();
    assert_eq!(quads, 28);
    // Node bounding boxes cover the entity's nodes: the volume boxes are the half blocks.
    assert_eq!(d.entities[3][0].min, [0.0, 0.0, 0.0]);
    assert_eq!(d.entities[3][0].max, [1.0, 2.0, 2.0]);
    assert_eq!(d.entities[3][1].max, [2.0, 2.0, 2.0]);
    // Element tags are 1..n in file order, node tags 1..n.
    let tags: Vec<u64> = d.element_blocks.iter().flat_map(|b| b.tags.clone()).collect();
    assert_eq!(tags, (1..=tags.len() as u64).collect::<Vec<_>>());
    let nodes: Vec<u64> = d.node_blocks.iter().flat_map(|b| b.tags.clone()).collect();
    let mut sorted = nodes.clone();
    sorted.sort_unstable();
    assert_eq!(sorted, (1..=27).collect::<Vec<_>>());
    // Without boundary elements and points only the cells remain.
    let mut o = GmshOptions::default();
    o.boundary_elements = false;
    o.lines_and_points = false;
    let (d, r) = round_trip(&f.mesh, &o);
    assert_eq!(r.elements_by_type.keys().copied().collect::<Vec<_>>(), vec![5]);
    assert_eq!(d.entities[3].len(), 2);
}

fn cells_of(mesh: &dyn MeshRead, volume: u64) -> Vec<CellIx> {
    (0..mesh.cell_count()).map(CellIx::new).filter(|c| mesh.cell_class(*c).entity.serial() == volume).collect()
}

#[test]
fn a_cell_set_that_cuts_a_volume_splits_the_entity() {
    let base = tet_block_two_volumes([4, 2, 2], [0.5; 3], 2).unwrap();
    let c1 = cells_of(&base.mesh, 1);
    let cut: Vec<CellIx> = c1.iter().copied().step_by(2).collect();
    let f = with_sets(tet_block_two_volumes([4, 2, 2], [0.5; 3], 2).unwrap(), &[SetSpec::new("cut", SetMembers::Cells(cut.clone())).labeled(7)]).unwrap();
    let (d, r) = round_trip(&f.mesh, &GmshOptions::default());
    assert_eq!(d.physical_names, vec![(3, 7, "cut".to_string())]);
    // Volume 1 -> (no set) and (set 7); volume 2 stays one entity: tags in (model id, signature) order.
    assert_eq!(r.entities[3], 3);
    assert_eq!(d.entities[3].iter().map(|e| e.physical.clone()).collect::<Vec<_>>(), vec![vec![], vec![7], vec![]]);
    assert_eq!(r.loss.get("msh.entity_split").unwrap().count, 1);
    let counts: Vec<usize> = (1..=3).map(|t| d.element_blocks.iter().filter(|b| b.dim == 3 && b.entity == t).map(|b| b.tags.len()).sum()).collect();
    assert_eq!(counts, vec![c1.len() - cut.len(), cut.len(), cells_of(&base.mesh, 2).len()]);
    // The total volume is unchanged by the split.
    let (vol, min) = jacobian_volume(&d);
    assert!(min > 0.0 && (vol - base.volume).abs() <= 1e-12 * base.volume);
}

#[test]
fn bad_labels_and_names_are_typed_errors() {
    let mk = |specs: &[SetSpec]| with_sets(hex_block_two_volumes([2, 1, 1], [1.0; 3], 1).unwrap(), specs).unwrap();
    let c = |v| SetMembers::Cells(cells_of(&hex_block_two_volumes([2, 1, 1], [1.0; 3], 1).unwrap().mesh, v));
    // The mesh API keeps labels unique per entity kind, so a repeat cannot reach the writer; labels
    // that are not positive 32-bit integers can.
    for bad in [0i64, -3, 1 << 40] {
        let f = mk(&[SetSpec::new("a", c(1)).labeled(bad)]);
        assert!(matches!(write_msh4(&f.mesh, &GmshOptions::default(), &mut Vec::new()), Err(GmshError::BadLabel { label, .. }) if label == bad));
    }
    let f = mk(&[SetSpec::new("a\"b", c(1))]);
    assert!(matches!(write_msh4(&f.mesh, &GmshOptions::default(), &mut Vec::new()), Err(GmshError::BadName(_))));
    // Unlabelled sets take the next free tag per dimension, labels are kept.
    let f = mk(&[SetSpec::new("x", c(1)).labeled(1)]);
    let (d, _) = round_trip(&f.mesh, &GmshOptions::default());
    let by: BTreeMap<String, u32> = d.physical_names.iter().map(|(_, t, n)| (n.clone(), *t)).collect();
    assert_eq!((by["x"], by["volume_1"], by["volume_2"]), (1, 2, 3));
    // Face and side sets become dimension 2 groups; sides lose their orientation.
    let skin: Vec<FaceIx> = (0..f.mesh.face_count()).map(FaceIx::new).filter(|i| f.mesh.face_right(*i).is_none()).collect();
    let g = with_sets(hex_block_two_volumes([2, 1, 1], [1.0; 3], 1).unwrap(), &[SetSpec::new("wall", SetMembers::Faces(skin.clone())), SetSpec::new("sides", SetMembers::Sides(skin.iter().map(|f| SignedFace::new(*f, Side::Left)).collect()))]).unwrap();
    let (d, r) = round_trip(&g.mesh, &GmshOptions::default());
    assert!(d.physical_names.contains(&(2, 1, "wall".to_string())) && d.physical_names.contains(&(2, 2, "sides".to_string())));
    assert!(r.loss.get("msh.side_orientation_dropped").is_some());
    let phys: usize = d.entities[2].iter().filter(|e| e.physical == vec![1, 2]).count();
    assert!(phys > 0, "skin surfaces carry both groups");
}

fn hybrid() -> ReferenceMesh {
    // The split-face cube (polyhedron) next to a hexahedron on its +x face.
    let mut p: Vec<[f64; 3]> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    p.extend([[2., 0., 0.], [2., 1., 0.], [2., 0., 1.], [2., 1., 1.]]);
    let poly = vec![vec![0, 2, 1], vec![0, 3, 2], vec![4, 5, 6, 7], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]];
    build_mesh(&p, &[CellSpec::Poly(poly), CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6])], &mut SerialAlloc::new()).unwrap()
}

#[test]
fn polyhedra_policies() {
    let dp = dual_patch().unwrap();
    let err = write_msh4(&dp.mesh, &GmshOptions::default(), &mut Vec::new()).unwrap_err();
    match err {
        GmshError::PolyhedraUnsupported { count, hint } => {
            assert_eq!(count, 7);
            assert_eq!(hint, "use openfoam, vtu or exodus");
        }
        e => panic!("{e:?}"),
    }
    // Decompose conserves the volume of planar-face polyhedra and every tet is positive.
    let o = GmshOptions::default().with_polyhedra(PolyPolicy::Decompose);
    for f in [dual_patch().unwrap(), hexagonal_prism(1.0, 2.0).unwrap(), split_face_cube().unwrap(), truncated_octahedron().unwrap()] {
        let (d, r) = round_trip(&f.mesh, &o);
        let (vol, min) = jacobian_volume(&d);
        assert!(min > 0.0, "{}: {min}", f.name);
        assert!((vol - f.volume).abs() <= 1e-12 * f.volume, "{}: {vol} vs {}", f.name, f.volume);
        assert_eq!(r.loss.get("msh.poly_decomposed").unwrap().count as usize, f.mesh.cell_count());
        assert_eq!(r.nodes as usize, f.mesh.vertex_count() + r.loss.get("msh.nodes_added").unwrap().count as usize);
        // Boundary elements are triangles on the same face centroids (no polygon is left).
        assert!(r.elements_by_type.keys().all(|t| [2u8, 3, 4, 15].contains(t)), "{}: {:?}", f.name, r.elements_by_type);
    }
    // A decomposed polyhedron next to a hexahedron across a quad is refused, naming the face.
    let h = hybrid();
    assert!(matches!(write_msh4(&h, &o, &mut Vec::new()), Err(GmshError::NonconformingDecomposition { .. })));
    // Drop reports the exact count and keeps the hexahedron.
    let o = GmshOptions::default().with_polyhedra(PolyPolicy::Drop);
    let (d, r) = round_trip(&h, &o);
    assert_eq!(r.loss.get("msh.poly_dropped").unwrap().count, 1);
    assert_eq!(r.elements_by_type[&5], 1);
    let (vol, min) = jacobian_volume(&d);
    assert!(min > 0.0 && (vol - 1.0).abs() <= 1e-12);
    let (_, r) = round_trip(&dual_patch().unwrap().mesh, &o);
    assert_eq!(r.loss.get("msh.poly_dropped").unwrap().count, 7);
    assert!(!r.elements_by_type.contains_key(&5));
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn bytes_are_identical_across_runs_and_thread_counts() {
    let f = hex_block_two_volumes([3, 3, 3], [0.5; 3], 2).unwrap();
    let m = mixed_transition().unwrap();
    let dp = dual_patch().unwrap();
    for binary in [false, true] {
        let o = opts(binary);
        sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&f.mesh, &o).1.hash)).unwrap();
        sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&m.mesh, &o).1.hash)).unwrap();
        let o = o.with_polyhedra(PolyPolicy::Decompose);
        sm_testkit::determinism::check(2, 4, |t| in_pool(t, || write(&dp.mesh, &o).1.hash)).unwrap();
        assert_eq!(write(&f.mesh, &o).0, write(&f.mesh, &o).0);
    }
}

#[test]
fn unsupported_inputs_are_typed_errors() {
    let f = hex_block_two_volumes([2, 1, 1], [1.0; 3], 1).unwrap();
    let w = common::wrap::Wrap::new(&f.mesh).dim2();
    assert!(matches!(write_msh4(&w, &GmshOptions::default(), &mut Vec::new()), Err(GmshError::Unsupported(_))));
    let w = common::wrap::Wrap::new(&f.mesh).inactive(&[0]);
    assert!(matches!(write_msh4(&w, &GmshOptions::default(), &mut Vec::new()), Err(GmshError::Unsupported(_))));
    let _ = EntityKind::TopoVolume;
}

proptest! {
    #![proptest_config(strategies::config(16))]

    #[test]
    fn random_blocks_round_trip(n0 in 2u32..5, n1 in 1u32..4, n2 in 1u32..4, split in 1u32..4, h in 0.1f64..2.0) {
        let split = 1 + split % (n0 - 1);
        let hs = [h, h * 0.75, h * 1.5];
        for (fixture, per_cell) in [(tet_block_two_volumes([n0, n1, n2], hs, split).unwrap(), 6usize), (hex_block_two_volumes([n0, n1, n2], hs, split).unwrap(), 1)] {
            let (d, r) = round_trip(&fixture.mesh, &GmshOptions::default());
            let cells = (n0 * n1 * n2) as usize * per_cell;
            prop_assert_eq!(r.elements_by_type.iter().filter(|(t, _)| **t == 4 || **t == 5).map(|(_, n)| *n).sum::<u64>() as usize, cells);
            prop_assert_eq!(r.entities[3], 2);
            let (vol, min) = jacobian_volume(&d);
            prop_assert!(min > 0.0);
            prop_assert!((vol - fixture.volume).abs() <= 1e-12 * fixture.volume);
        }
    }
}

#[test]
fn corrupted_input_never_panics() {
    let f = hex_block_two_volumes([2, 2, 1], [1.0; 3], 1).unwrap();
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for binary in [false, true] {
        let good = write(&f.mesh, &opts(binary)).0;
        assert!(parse_msh4(&good).is_ok());
        for len in (0..good.len()).step_by(7) {
            let _ = parse_msh4(&good[..len]);
        }
        for _ in 0..3000 {
            let mut b = good.clone();
            for _ in 0..1 + next() % 4 {
                let i = (next() % b.len() as u64) as usize;
                b[i] = next() as u8;
            }
            let _ = parse_msh4(&b);
        }
    }
    assert!(matches!(parse_msh4(b""), Err(GmshError::Malformed(_))));
    assert!(matches!(parse_msh4(b"$MeshFormat\n2.2 0 8\n$EndMeshFormat\n"), Err(GmshError::Malformed(_))));
}
