//! Property tests (pinned seeds): random structured blocks, random mixed hex/wedge meshes,
//! random surface meshes and injected classification faults, all against the reference.

mod common;
use common::replay::{Mode, assert_same, replay};
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh::{Mesh, MeshBuilder, Validation};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;
use sm_testkit::check::{
    box_carriers, check_mesh_euler, classification_complete, closed_cells, conformity, mesh_euler, mesh_valid,
    orientation_rule, positive_volumes, watertight,
};
use sm_testkit::meshes::{CellSpec, build_mesh};
use sm_testkit::strategies::{config, hex_block_spec};

fn model(kind: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(kind, n).unwrap())
}

proptest! {
    #![proptest_config(config(32))]

    #[test]
    fn random_hex_blocks_match_the_reference_and_pass_every_checker(spec in hex_block_spec()) {
        let want = spec.build().unwrap();
        let inc = hex_block(MeshBuilder::new(), spec.n, spec.h, false, &mut SerialAlloc::new()).unwrap();
        let blk = hex_block_bulk(spec.n, spec.h, &mut SerialAlloc::new()).unwrap();
        assert_same(&want, &inc.mesh, "incremental");
        assert_same(&want, &blk.mesh, "bulk");
        prop_assert_eq!(&inc.face_remap, &blk.face_remap);
        let m = &blk.mesh;
        prop_assert!(mesh_valid(m).is_ok());
        prop_assert!(orientation_rule(m).is_ok());
        prop_assert!(closed_cells(m).is_ok());
        prop_assert!(positive_volumes(m).is_ok());
        prop_assert!(watertight(m).is_ok());
        prop_assert!(classification_complete(m).is_ok());
        prop_assert!(check_mesh_euler(m, mesh_euler(m)).is_ok());
        prop_assert!(conformity(m, &box_carriers(spec.n, spec.h), 1e-9).is_ok());
        // Analytic total volume.
        let vol: f64 = (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum();
        prop_assert!((vol - spec.volume()).abs() <= 1e-9 * spec.volume());
        // Boundary area is exact too.
        let area: f64 = (0..m.face_count())
            .filter(|&f| m.is_boundary_face(FaceIx::new(f)))
            .map(|f| face_area_vector(m, FaceIx::new(f)).norm())
            .sum();
        let [a, b, c] = [spec.n[0] as f64 * spec.h[0], spec.n[1] as f64 * spec.h[1], spec.n[2] as f64 * spec.h[2]];
        let want_area = 2.0 * (a * b + b * c + a * c);
        prop_assert!((area - want_area).abs() <= 1e-9 * want_area);
    }
}

/// A grid of unit columns, each either a hex per layer or two wedges per layer (split along one
/// of the two diagonals, chosen per column so the stack conforms).
fn mixed_grid(n: [usize; 3], split: &[u8]) -> (Vec<[f64; 3]>, Vec<CellSpec>) {
    let m = [n[0] + 1, n[1] + 1, n[2] + 1];
    let ix = |i: usize, j: usize, k: usize| (k * m[1] + j) * m[0] + i;
    let mut pts = Vec::new();
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                pts.push([i as f64, j as f64, k as f64]);
            }
        }
    }
    let mut cells = Vec::new();
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let v = [
                    ix(i, j, k),
                    ix(i + 1, j, k),
                    ix(i + 1, j + 1, k),
                    ix(i, j + 1, k),
                    ix(i, j, k + 1),
                    ix(i + 1, j, k + 1),
                    ix(i + 1, j + 1, k + 1),
                    ix(i, j + 1, k + 1),
                ];
                match split[j * n[0] + i] % 3 {
                    0 => cells.push(CellSpec::Template(CellShape::Hex, v.to_vec())),
                    1 => {
                        cells.push(CellSpec::Template(CellShape::Wedge, vec![v[0], v[1], v[2], v[4], v[5], v[6]]));
                        cells.push(CellSpec::Template(CellShape::Wedge, vec![v[0], v[2], v[3], v[4], v[6], v[7]]));
                    }
                    _ => {
                        cells.push(CellSpec::Template(CellShape::Wedge, vec![v[0], v[1], v[3], v[4], v[5], v[7]]));
                        cells.push(CellSpec::Template(CellShape::Wedge, vec![v[1], v[2], v[3], v[5], v[6], v[7]]));
                    }
                }
            }
        }
    }
    (pts, cells)
}

/// Rebuild the template cells of `src` in bulk, classified like `src`.
fn bulk_rebuild(src: &dyn MeshRead) -> Mesh {
    let mut b = MeshBuilder::new();
    let co = src.coords();
    for i in 0..src.vertex_count() {
        let v = b.add_vertex(co.get(i)).unwrap();
        b.set_class(MeshEntityRef::Vertex(v), src.vertex_class(v)).unwrap();
    }
    let (mut shapes, mut offsets, mut verts) = (Vec::new(), vec![0u64], Vec::new());
    for c in 0..src.cell_count() {
        let c = CellIx::new(c);
        shapes.push(src.cell_shape(c));
        verts.extend_from_slice(src.cell_corners(c).unwrap());
        offsets.push(verts.len() as u64);
    }
    b.add_cells_bulk(&shapes, &offsets, &verts).unwrap();
    for c in 0..src.cell_count() {
        b.set_class(MeshEntityRef::Cell(CellIx::new(c)), src.cell_class(CellIx::new(c))).unwrap();
    }
    for f in 0..src.face_count() {
        let (p, _) = b.find_face(src.face_vertices(FaceIx::new(f))).unwrap();
        b.set_class(MeshEntityRef::Face(p), src.face_class(FaceIx::new(f))).unwrap();
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

proptest! {
    #![proptest_config(config(24))]

    #[test]
    fn random_mixed_hex_wedge_meshes_match_the_reference(
        n in [1usize..=3, 1usize..=3, 1usize..=2],
        split in prop::collection::vec(0u8..3, 9),
    ) {
        let (pts, cells) = mixed_grid(n, &split);
        let want: ReferenceMesh = build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap();
        for mode in [Mode::Explicit, Mode::Mesher] {
            let (f, _) = replay(&want, MeshBuilder::new(), mode, &mut SerialAlloc::new()).unwrap();
            assert_same(&want, &f.mesh, &format!("{mode:?}"));
        }
        let bulk = bulk_rebuild(&want);
        assert_same(&want, &bulk, "bulk");
        prop_assert!(mesh_valid(&bulk).is_ok());
        let vol: f64 = (0..bulk.cell_count()).map(|c| cell_volume(&bulk, CellIx::new(c))).sum();
        prop_assert!((vol - (n[0] * n[1] * n[2]) as f64).abs() < 1e-9);
    }
}

/// An `nx x ny` quad grid, every quad randomly kept or split along a random diagonal.
fn surface_grid<B: MeshBuild>(mut b: B, n: [usize; 2], split: &[u8], alloc: &mut SerialAlloc) -> Result<Finished<B::Output>> {
    let m = [n[0] + 1, n[1] + 1];
    let ix = |i: usize, j: usize| VertexIx::new(j * m[0] + i);
    for j in 0..m[1] {
        for i in 0..m[0] {
            let v = b.add_vertex(Point3::new(i as f64, j as f64, 0.0))?;
            let boundary = i == 0 || j == 0 || i == n[0] || j == n[1];
            b.set_class(
                MeshEntityRef::Vertex(v),
                if boundary { model(EntityKind::TopoVertex, 1) } else { model(EntityKind::TopoFace, 1) },
            )?;
        }
    }
    for j in 0..n[1] {
        for i in 0..n[0] {
            let q = [ix(i, j), ix(i + 1, j), ix(i + 1, j + 1), ix(i, j + 1)];
            let pieces: Vec<Vec<VertexIx>> = match split[j * n[0] + i] % 3 {
                0 => vec![q.to_vec()],
                1 => vec![vec![q[0], q[1], q[2]], vec![q[0], q[2], q[3]]],
                _ => vec![vec![q[0], q[1], q[3]], vec![q[1], q[2], q[3]]],
            };
            for p in pieces {
                let shape = if p.len() == 3 { CellShape::Tri } else { CellShape::Quad };
                let added = b.add_cell_from_vertices(shape, &p)?;
                b.set_class(MeshEntityRef::Cell(added.cell), model(EntityKind::TopoFace, 1))?;
            }
        }
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        if b.face_degree(fi) == 1 {
            b.set_class(MeshEntityRef::Face(fi), model(EntityKind::TopoEdge, 1))?;
        }
    }
    b.finish(alloc)
}

proptest! {
    #![proptest_config(config(24))]

    #[test]
    fn random_surface_meshes_match_the_reference(
        n in [1usize..=4, 1usize..=4],
        split in prop::collection::vec(0u8..3, 16),
    ) {
        let want = surface_grid(ReferenceBuilder::surface(), n, &split, &mut SerialAlloc::new()).unwrap().mesh;
        let got = surface_grid(MeshBuilder::surface(), n, &split, &mut SerialAlloc::new()).unwrap().mesh;
        assert_same(&want, &got, "surface incremental");
        for mode in [Mode::Explicit, Mode::Mesher] {
            let (f, _) = replay(&want, MeshBuilder::surface(), mode, &mut SerialAlloc::new()).unwrap();
            assert_same(&want, &f.mesh, &format!("surface {mode:?}"));
        }
        // Bulk in dimension 2 (Tri and Quad cells).
        let mut b = MeshBuilder::surface();
        let co = want.coords();
        for i in 0..want.vertex_count() {
            let v = b.add_vertex(co.get(i)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), want.vertex_class(v)).unwrap();
        }
        let (mut shapes, mut offsets, mut verts) = (Vec::new(), vec![0u64], Vec::new());
        for c in 0..want.cell_count() {
            let c = CellIx::new(c);
            shapes.push(want.cell_shape(c));
            verts.extend_from_slice(want.cell_corners(c).unwrap());
            offsets.push(verts.len() as u64);
        }
        b.add_cells_bulk(&shapes, &offsets, &verts).unwrap();
        for c in 0..want.cell_count() {
            b.set_class(MeshEntityRef::Cell(CellIx::new(c)), want.cell_class(CellIx::new(c))).unwrap();
        }
        for f in 0..want.face_count() {
            let (p, _) = b.find_face(want.face_vertices(FaceIx::new(f))).unwrap();
            b.set_class(MeshEntityRef::Face(p), want.face_class(FaceIx::new(f))).unwrap();
        }
        let bulk = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
        assert_same(&want, &bulk, "surface bulk");
        // Analytic: the grid covers n0 x n1 unit squares.
        let area: f64 = (0..bulk.cell_count()).map(|c| cell_volume(&bulk, CellIx::new(c))).sum();
        prop_assert!((area - (n[0] * n[1]) as f64).abs() < 1e-9);
    }
}

// ---------------------------------------------------------------------------- injected faults

#[derive(Clone, Copy, Debug)]
enum Fault {
    None,
    BoundaryFaceOnVolume(usize),
    BoundaryFaceOnEdge(usize),
    VertexOnVolume(usize),
    UnclassifiedVertex(usize),
    UnclassifiedCell(usize),
    ActiveChildOfActiveParent(usize),
    ConflictingCells,
    NonConformalMark(usize),
    OpenCell,
    EdgeAboveItsFaces,
}

/// A 2x2x2 block (edges fully classified) with one injected classification fault.
fn faulty<B: MeshBuild>(mut b: B, fault: Fault, alloc: &mut SerialAlloc) -> Result<Finished<B::Output>> {
    let n = [2usize, 2, 2];
    let m = [3usize, 3, 3];
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m[1] + j) * m[0] + i);
    let vol = model(EntityKind::TopoVolume, 1);
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                b.add_vertex(Point3::new(i as f64, j as f64, k as f64))?;
            }
        }
    }
    // Vertices: on the volume if interior, else on the single TopoFace#1 (a valid, if odd, class).
    for v in 0..b.vertex_count() {
        let (i, j, k) = (v % 3, (v / 3) % 3, v / 9);
        let boundary = [i, j, k].iter().any(|&q| q == 0 || q == 2);
        let c = if boundary { model(EntityKind::TopoFace, 1) } else { vol };
        let skip = matches!(fault, Fault::UnclassifiedVertex(x) if x == v);
        if matches!(fault, Fault::VertexOnVolume(x) if x == v) {
            b.set_class(MeshEntityRef::Vertex(VertexIx::new(v)), vol)?;
        } else if !skip {
            b.set_class(MeshEntityRef::Vertex(VertexIx::new(v)), c)?;
        }
    }
    let mut cell_no = 0;
    let mut first_cells = Vec::new();
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                let vs = [
                    ix(i, j, k),
                    ix(i + 1, j, k),
                    ix(i + 1, j + 1, k),
                    ix(i, j + 1, k),
                    ix(i, j, k + 1),
                    ix(i + 1, j, k + 1),
                    ix(i + 1, j + 1, k + 1),
                    ix(i, j + 1, k + 1),
                ];
                let added = if matches!(fault, Fault::OpenCell) && cell_no == 0 {
                    // The same hex as a polyhedron missing its top face.
                    let loops: Vec<Vec<VertexIx>> = CellShape::Hex
                        .local_faces()
                        .iter()
                        .enumerate()
                        .filter(|(q, _)| *q != 1)
                        .map(|(_, lf)| lf.iter().map(|&x| vs[x as usize]).collect())
                        .collect();
                    let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
                    b.add_polyhedron(&refs)?
                } else {
                    b.add_cell_from_vertices(CellShape::Hex, &vs)?
                };
                first_cells.push(added.cell);
                let unclassified = matches!(fault, Fault::UnclassifiedCell(x) if x == cell_no);
                let c = if matches!(fault, Fault::ConflictingCells) && cell_no == 1 {
                    model(EntityKind::TopoVolume, 2)
                } else {
                    vol
                };
                if !unclassified {
                    b.set_class(MeshEntityRef::Cell(added.cell), c)?;
                }
                cell_no += 1;
            }
        }
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        if b.face_degree(fi) == 1 {
            let c = match fault {
                Fault::BoundaryFaceOnVolume(x) if x == f => vol,
                Fault::BoundaryFaceOnEdge(x) if x == f => model(EntityKind::TopoEdge, 1),
                _ => model(EntityKind::TopoFace, 1),
            };
            b.set_class(MeshEntityRef::Face(fi), c)?;
            if matches!(fault, Fault::NonConformalMark(x) if x == f) {
                b.set_face_kind(fi, FaceKind::NonConformal)?;
            }
        }
    }
    if let Fault::ActiveChildOfActiveParent(c) = fault {
        let c = c.max(1).min(first_cells.len() - 1);
        b.set_cell_parent(first_cells[c], first_cells[0])?;
    }
    if matches!(fault, Fault::EdgeAboveItsFaces) {
        // Classify every edge (so the table derives), one of them above its faces.
        let pairs = [(ix(0, 0, 0), ix(1, 0, 0), model(EntityKind::TopoVolume, 1))];
        for (a, c, cl) in pairs {
            b.set_class(MeshEntityRef::Edge(a, c), cl)?;
        }
        // Every other edge of the block gets the face class.
        for k in 0..m[2] {
            for j in 0..m[1] {
                for i in 0..m[0] {
                    for (di, dj, dk) in [(1, 0, 0), (0, 1, 0), (0, 0, 1)] {
                        let (i2, j2, k2) = (i + di, j + dj, k + dk);
                        if i2 < m[0] && j2 < m[1] && k2 < m[2] && !(i == 0 && j == 0 && k == 0 && di == 1) {
                            b.set_class(
                                MeshEntityRef::Edge(ix(i, j, k), ix(i2, j2, k2)),
                                model(EntityKind::TopoFace, 1),
                            )?;
                        }
                    }
                }
            }
        }
    }
    b.finish(alloc)
}

fn fault_strategy() -> impl Strategy<Value = Fault> {
    prop_oneof![
        Just(Fault::None),
        (0usize..24).prop_map(Fault::BoundaryFaceOnVolume),
        (0usize..24).prop_map(Fault::BoundaryFaceOnEdge),
        (0usize..27).prop_map(Fault::VertexOnVolume),
        (0usize..27).prop_map(Fault::UnclassifiedVertex),
        (0usize..8).prop_map(Fault::UnclassifiedCell),
        (1usize..8).prop_map(Fault::ActiveChildOfActiveParent),
        Just(Fault::ConflictingCells),
        (0usize..24).prop_map(Fault::NonConformalMark),
        Just(Fault::OpenCell),
        Just(Fault::EdgeAboveItsFaces),
    ]
}

proptest! {
    #![proptest_config(config(96))]

    /// For every injected fault, the fast validation, the full validation and the reference
    /// builder agree: same `Ok`, or the same error (including the full violation list), and the
    /// same allocator state afterwards.
    #[test]
    fn fast_validation_never_accepts_what_the_reference_rejects(fault in fault_strategy()) {
        let (mut ar, mut af, mut au) = (SerialAlloc::new(), SerialAlloc::new(), SerialAlloc::new());
        let want = faulty(ReferenceBuilder::new(), fault, &mut ar);
        let fast = faulty(MeshBuilder::new().with_validation(Validation::Fast), fault, &mut af);
        let full = faulty(MeshBuilder::new().with_validation(Validation::Full), fault, &mut au);
        match (&want, &fast, &full) {
            (Ok(w), Ok(f), Ok(u)) => {
                assert_same(&w.mesh, &f.mesh, &format!("{fault:?} fast"));
                assert_same(&w.mesh, &u.mesh, &format!("{fault:?} full"));
            }
            (Err(w), Err(f), Err(u)) => {
                prop_assert_eq!(w, f, "{:?}", fault);
                prop_assert_eq!(w, u, "{:?}", fault);
            }
            _ => prop_assert!(false, "{:?}: reference {:?}, fast {:?}, full {:?}", fault,
                want.as_ref().map(|_| ()), fast.as_ref().map(|_| ()), full.as_ref().map(|_| ())),
        }
        for k in [EntityKind::Mesh, EntityKind::MeshVertex, EntityKind::MeshFace, EntityKind::MeshCell, EntityKind::MeshEdge] {
            prop_assert_eq!(ar.next_serial(k), af.next_serial(k));
            prop_assert_eq!(ar.next_serial(k), au.next_serial(k));
        }
    }
}

#[test]
fn the_fault_scripts_really_exercise_failures() {
    // Guard against a vacuous property: the specific faults must fail (or pass) as designed.
    let run = |f: Fault| faulty(ReferenceBuilder::new(), f, &mut SerialAlloc::new()).map(|_| ());
    assert!(run(Fault::None).is_ok());
    assert!(run(Fault::NonConformalMark(0)).is_ok());
    for f in [
        Fault::BoundaryFaceOnVolume(0),
        Fault::BoundaryFaceOnEdge(0),
        Fault::VertexOnVolume(0),
        Fault::UnclassifiedVertex(0),
        Fault::UnclassifiedCell(0),
        Fault::ActiveChildOfActiveParent(1),
        Fault::ConflictingCells,
        Fault::OpenCell,
        Fault::EdgeAboveItsFaces,
    ] {
        assert!(run(f).is_err(), "{f:?} should fail");
    }
    // The failures reported are the intended ones.
    assert!(matches!(run(Fault::OpenCell), Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::Closedness)));
    assert!(matches!(run(Fault::VertexOnVolume(0)), Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::ClassDimOrder)));
    assert!(matches!(run(Fault::ActiveChildOfActiveParent(1)), Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::Lineage)));
    assert!(matches!(run(Fault::EdgeAboveItsFaces), Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::ClassDimOrder)));
    assert!(matches!(run(Fault::BoundaryFaceOnVolume(0)), Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::ClassKind)));
}
