//! Topology scenarios: mesher writes, consumer reads (DG-SN sweep, MC walking), exporters.

mod common;
use common::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block};
use sm_mesh_api::*;
use std::collections::BTreeMap;

/// A sweep mesher: extrude an `nx x ny` quad grid through `nz` layers, written through the
/// builder trait only. Caps are `TopoFace#1/#2`, the lateral wall `TopoFace#3`.
fn sweep<B: MeshBuild>(
    mut b: B,
    nx: usize,
    ny: usize,
    nz: usize,
    alloc: &mut SerialAlloc,
) -> (B::Output, Vec<(FaceIx, [VertexIx; 4])>) {
    let at = |i: usize, j: usize, k: usize| VertexIx::new((k * (ny + 1) + j) * (nx + 1) + i);
    for k in 0..=nz {
        for j in 0..=ny {
            for i in 0..=nx {
                let v = b
                    .add_vertex(Point3::new(i as f64, 0.5 * j as f64, 2.0 * k as f64))
                    .unwrap();
                let cap = k == 0 || k == nz;
                let wall = i == 0 || i == nx || j == 0 || j == ny;
                let c = if k == 0 {
                    on(EntityKind::TopoFace, 1)
                } else if k == nz {
                    on(EntityKind::TopoFace, 2)
                } else if wall {
                    on(EntityKind::TopoFace, 3)
                } else {
                    vol()
                };
                let _ = cap;
                b.set_class(MeshEntityRef::Vertex(v), c).unwrap();
            }
        }
    }
    let mut cap_faces = Vec::new();
    let mut mat = Vec::new();
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let vs = [
                    at(i, j, k),
                    at(i + 1, j, k),
                    at(i + 1, j + 1, k),
                    at(i, j + 1, k),
                    at(i, j, k + 1),
                    at(i + 1, j, k + 1),
                    at(i + 1, j + 1, k + 1),
                    at(i, j + 1, k + 1),
                ];
                let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
                b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
                mat.push((k % 2) as i64);
                // Template face order: -z, +z, -y, +x, +y, -x.
                let on_wall = [false, false, j == 0, i + 1 == nx, j + 1 == ny, i == 0];
                for (n, sf) in added.faces.iter().enumerate() {
                    let class = match n {
                        0 if k == 0 => Some(1),
                        1 if k + 1 == nz => Some(2),
                        _ if on_wall[n] => Some(3),
                        _ => None,
                    };
                    if let Some(s) = class {
                        b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, s))
                            .unwrap();
                    }
                    if n == 0 && k == 0 {
                        cap_faces.push((sf.face(), [vs[0], vs[1], vs[2], vs[3]]));
                    }
                }
            }
        }
    }
    b.add_tag("material", MeshEntityKind::Cell, TagValues::I64(mat))
        .unwrap();
    let fin = b.finish(alloc).unwrap();
    let remapped = cap_faces
        .into_iter()
        .map(|(f, vs)| (fin.face_remap[f.get()], vs))
        .collect();
    (fin.mesh, remapped)
}

#[test]
fn sweep_mesher_writes_hex_mesh_with_classification() {
    let (m, caps): (ReferenceMesh, _) =
        sweep(ReferenceBuilder::new(), 3, 2, 4, &mut SerialAlloc::new());
    assert_valid(&m);
    assert_eq!((m.vertex_count(), m.cell_count()), (4 * 3 * 5, 24));
    // Faces: boundary = 2 caps (6 each) + wall 2*(3+2)*4 = 40; interior = 3*4*... count by kind.
    let nb = (0..m.face_count())
        .filter(|&f| m.is_boundary_face(FaceIx::new(f)))
        .count();
    assert_eq!(nb, 2 * 6 + 2 * (3 + 2) * 4);
    assert_eq!(
        m.classified_on(id(EntityKind::TopoFace, 1))
            .unwrap()
            .faces
            .len(),
        6
    );
    assert_eq!(
        m.classified_on(id(EntityKind::TopoFace, 3))
            .unwrap()
            .faces
            .len(),
        40
    );
    assert_eq!(m.classified_on(vol().entity).unwrap().cells.len(), 24);
    // Interior faces were auto-classified on the volume.
    for f in interior_faces(&m) {
        assert_eq!(m.face_class(f).entity, vol().entity);
    }
    // Every hex has template corners and an analytic volume 1 * 0.5 * 2.
    for c in 0..24 {
        let c = CellIx::new(c);
        assert_eq!(m.cell_shape(c), CellShape::Hex);
        assert_eq!(m.cell_corners(c).unwrap().len(), 8);
        assert!((cell_volume(&m, c) - 1.0).abs() < 1e-13);
    }
    // Tag column and selector.
    assert_eq!(
        m.tag(MeshEntityKind::Cell, "material")
            .unwrap()
            .values
            .as_i64()
            .unwrap()
            .iter()
            .filter(|&&x| x == 1)
            .count(),
        12
    );
    let sel = select::select(
        &m,
        MeshEntityKind::Cell,
        &select::Selector::TagEq("material".into(), 1),
    );
    assert_eq!(sel.len(), 12);
    // face_remap: the remapped bottom-cap faces have exactly the vertices recorded at creation.
    for (f, vs) in caps {
        let mut got: Vec<_> = m.face_vertices(f).to_vec();
        got.sort_unstable();
        let mut want = vs.to_vec();
        want.sort_unstable();
        assert_eq!(got, want);
        assert_eq!(m.face_class(f).entity, id(EntityKind::TopoFace, 1));
    }
}

#[test]
fn hex_block_orientation_boundary_and_walking() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([2, 2, 2], [0.5; 3], &mut alloc).unwrap();
    assert_eq!(
        (m.vertex_count(), m.face_count(), m.cell_count()),
        (27, 36, 8)
    );
    assert_valid(&m);
    assert_eq!(interior_faces(&m).len(), 12);
    for f in interior_faces(&m) {
        let (l, r) = m.face_cells(f);
        let r = r.unwrap();
        assert!(l < r && m.cell_id(l) < m.cell_id(r));
        let n = face_area_vector(&m, f);
        assert!(
            n.dot(centre(&m, r) - centre(&m, l)) > 0.0,
            "normal points left -> right"
        );
        assert!(m.cell_faces(l).contains(&SignedFace::new(f, Side::Left)));
        assert!(m.cell_faces(r).contains(&SignedFace::new(f, Side::Right)));
        assert_eq!(m.across(SignedFace::new(f, Side::Left)), Some(r));
        assert_eq!(m.across(SignedFace::new(f, Side::Right)), Some(l));
    }
    // Boundary normals are outward; kinds agree.
    let mut nbnd = 0;
    for f in (0..36).map(FaceIx::new).filter(|&f| m.is_boundary_face(f)) {
        nbnd += 1;
        assert!(
            face_area_vector(&m, f).dot(face_centroid(&m, f) - centre(&m, m.face_left(f))) > 0.0
        );
        assert_eq!(m.face_class(f).entity.kind(), EntityKind::TopoFace);
    }
    assert_eq!(nbnd, 24);
    // Neighbours of cell 0 are 1 (x), 2 (y), 4 (z).
    let mut nb: Vec<usize> = m
        .cell_faces(CellIx(0))
        .iter()
        .filter_map(|&sf| m.across(sf))
        .map(|c| c.get())
        .collect();
    nb.sort_unstable();
    assert_eq!(nb, vec![1, 2, 4]);
    // other_cell is checked: a non-incident cell is an error, not a silent wrong answer.
    let f01 = interior_faces(&m)
        .into_iter()
        .find(|&f| m.face_cells(f) == (CellIx(0), Some(CellIx(1))))
        .unwrap();
    assert_eq!(m.other_cell(f01, CellIx(0)), Ok(Some(CellIx(1))));
    assert_eq!(m.other_cell(f01, CellIx(1)), Ok(Some(CellIx(0))));
    assert_eq!(
        m.other_cell(f01, CellIx(7)),
        Err(MeshError::NotIncident(f01, CellIx(7)))
    );
    let bf = (0..36)
        .map(FaceIx::new)
        .find(|&f| m.is_boundary_face(f) && m.face_left(f) == CellIx(0))
        .unwrap();
    assert_eq!(m.other_cell(bf, CellIx(0)), Ok(None));
    // Id lookups round trip.
    for c in 0..8 {
        assert_eq!(m.index_of(m.cell_id(CellIx::new(c))), Some(c as u32));
    }
    assert_eq!(m.index_of(id(EntityKind::MeshCell, 999)), None);
}

#[test]
fn dg_sn_sweep_consumer_reads_columns_and_dyn_api() {
    let m = hex_block([4, 3, 2], [1.0, 0.7, 1.3], &mut SerialAlloc::new()).unwrap();
    let dynm: &dyn MeshRead = &m;
    // Cell closure: sum of outward area vectors vanishes; volume from the divergence theorem.
    for c in 0..dynm.cell_count() {
        let c = CellIx::new(c);
        let mut s = Vector3::default();
        for sf in dynm.cell_faces(c) {
            s = s + face_area_vector(dynm, sf.face()) * sf.sign();
        }
        assert!(s.norm() < 1e-13);
    }
    // Upwind graph for a direction: Kahn's algorithm over the bulk columns, no per-call dispatch.
    let w = Vector3::new(1.0, 0.6, 0.3);
    let cols = dynm.columns();
    let nc = dynm.cell_count();
    let mut indeg = vec![0usize; nc];
    let mut down: Vec<Vec<usize>> = vec![Vec::new(); nc];
    for f in 0..dynm.face_count() {
        let r = cols.face_right[f];
        if r == NO_CELL {
            continue;
        }
        let a = face_area_vector(dynm, FaceIx::new(f));
        let (l, r) = (cols.face_left[f].get(), r as usize);
        if w.dot(a) > 0.0 {
            down[l].push(r);
            indeg[r] += 1;
        } else {
            down[r].push(l);
            indeg[l] += 1;
        }
    }
    let mut order = Vec::new();
    let mut ready: Vec<usize> = (0..nc).filter(|&c| indeg[c] == 0).collect();
    while let Some(c) = ready.pop() {
        order.push(c);
        for &d in &down[c] {
            indeg[d] -= 1;
            if indeg[d] == 0 {
                ready.push(d);
            }
        }
    }
    assert_eq!(
        order.len(),
        nc,
        "the upwind graph of a convex hex block is acyclic"
    );
    // Bulk columns agree with the per-call accessors.
    for c in 0..nc {
        let (a, b) = (
            cols.cell_face_offsets[c] as usize,
            cols.cell_face_offsets[c + 1] as usize,
        );
        assert_eq!(&cols.cell_faces[a..b], dynm.cell_faces(CellIx::new(c)));
    }
    assert_eq!(cols.face_vertex_offsets.len(), dynm.face_count() + 1);
    // Geometry cache equals the free functions and is checked by validate.
    let mut mm = m.clone();
    let cache = compute_geometry_cache(&mm);
    mm.set_geometry_cache(Some(cache.clone())).unwrap();
    assert!(validate(&mm).is_empty());
    assert_eq!(
        mm.geometry_cache().unwrap().cell_volume[0],
        cell_volume(&mm, CellIx(0))
    );
    assert!((cache.cell_volume.iter().sum::<f64>() - 4.0 * 3.0 * 2.0 * 0.7 * 1.3).abs() < 1e-12);
    // Moving a point stales the cache: it is dropped.
    mm.set_point(PointIx(0), Point3::new(-0.01, 0.0, 0.0))
        .unwrap();
    assert!(mm.geometry_cache().is_none());
}

#[test]
fn monte_carlo_walk_is_one_step_per_cell() {
    let n = 12;
    let m = hex_block([n, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let dynm: &dyn MeshRead = &m;
    let mut cell = CellIx(0);
    let mut steps = 0;
    loop {
        // The +x exit face of a hex is the one whose outward area vector points along +x.
        let sf = *dynm
            .cell_faces(cell)
            .iter()
            .max_by(|a, b| {
                let ax = face_area_vector(dynm, a.face()).x * a.sign();
                let bx = face_area_vector(dynm, b.face()).x * b.sign();
                ax.partial_cmp(&bx).unwrap()
            })
            .unwrap();
        steps += 1;
        match dynm.across(sf) {
            Some(next) => {
                assert_eq!(next.get(), cell.get() + 1);
                cell = next;
            }
            None => break,
        }
    }
    assert_eq!(steps, n as usize);
    assert!(
        dynm.is_boundary_face(
            dynm.cell_faces(cell)
                .iter()
                .map(|s| s.face())
                .find(|&f| dynm.face_right(f).is_none() && face_centroid(dynm, f).x > 11.9)
                .unwrap()
        )
    );
}

#[test]
fn openfoam_owner_neighbour_export_needs_no_flips() {
    let m = hex_block([3, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // Exporter step: interior faces ordered by (owner, neighbour), then boundary faces grouped by patch.
    let mut interior: Vec<FaceIx> = interior_faces(&m);
    interior.sort_by_key(|&f| (m.face_left(f), m.face_right(f)));
    let mut patches: BTreeMap<EntityId, Vec<FaceIx>> = BTreeMap::new();
    for f in (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.is_boundary_face(f))
    {
        patches.entry(m.face_class(f).entity).or_default().push(f);
    }
    assert_eq!(patches.len(), 6);
    let order: Vec<FaceIx> = interior
        .iter()
        .copied()
        .chain(patches.values().flatten().copied())
        .collect();
    assert_eq!(order.len(), m.face_count());
    for (n, &f) in order.iter().enumerate() {
        let owner = m.face_left(f);
        let a = face_area_vector(&m, f);
        let fc = face_centroid(&m, f);
        // OpenFOAM requires: owner < neighbour, and the face normal points from owner out of the owner.
        assert!(
            a.dot(fc - centre(&m, owner)) > 0.0,
            "face {n}: loop used as stored is outward from the owner"
        );
        if let Some(nb) = m.face_right(f) {
            assert!(owner < nb);
            assert!(a.dot(centre(&m, nb) - fc) > 0.0);
        }
    }
    // Upper-triangular property holds for the sorted interior faces.
    assert!(interior.windows(2).all(
        |w| (m.face_left(w[0]), m.face_right(w[0])) <= (m.face_left(w[1]), m.face_right(w[1]))
    ));
}

#[test]
fn cgns_ngon_nface_export_needs_no_flips() {
    let m = hex_block([2, 2, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // NGON: face loops as stored. ParentElements = (left, right-or-0). NFACE: signed face ids.
    let mut nface: Vec<i64> = Vec::new();
    for c in 0..m.cell_count() {
        let c = CellIx::new(c);
        let cc = centre(&m, c);
        for sf in m.cell_faces(c) {
            let signed = (i64::from(sf.face().0) + 1) * if sf.sign() > 0.0 { 1 } else { -1 };
            nface.push(signed);
            // The CGNS sign is the NFACE sign: + means the stored loop is outward for this cell.
            let a = face_area_vector(&m, sf.face());
            assert!(a.dot(face_centroid(&m, sf.face()) - cc) * sf.sign() > 0.0);
        }
    }
    assert_eq!(nface.len(), 4 * 6);
    for f in (0..m.face_count()).map(FaceIx::new) {
        let (l, r) = (m.face_left(f), m.face_right(f));
        assert!(m.cell_faces(l).contains(&SignedFace::new(f, Side::Left)));
        if let Some(r) = r {
            assert!(m.cell_faces(r).contains(&SignedFace::new(f, Side::Right)));
        }
    }
}

#[test]
fn single_pentagonal_prism_polyhedron() {
    let mut b = ReferenceBuilder::new();
    let (r, hgt) = (1.0_f64, 2.0_f64);
    let ring = |z: f64, b: &mut ReferenceBuilder| -> Vec<VertexIx> {
        (0..5u64)
            .map(|i| {
                let a = 2.0 * std::f64::consts::PI * i as f64 / 5.0;
                let v = b
                    .add_vertex(Point3::new(r * a.cos(), r * a.sin(), z))
                    .unwrap();
                b.set_class(
                    MeshEntityRef::Vertex(v),
                    on(EntityKind::TopoVertex, 1 + i + if z > 0.0 { 5 } else { 0 }),
                )
                .unwrap();
                v
            })
            .collect()
    };
    let bot = ring(0.0, &mut b);
    let top = ring(hgt, &mut b);
    let mut loops: Vec<Vec<VertexIx>> = vec![bot.iter().rev().copied().collect(), top.clone()];
    for i in 0..5 {
        let j = (i + 1) % 5;
        loops.push(vec![bot[i], bot[j], top[j], top[i]]);
    }
    let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
    let added = b.add_polyhedron(&refs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    let m = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polyhedron);
    assert!(m.cell_corners(CellIx(0)).is_none());
    assert_eq!(m.face_count(), 7);
    assert_valid(&m);
    let area = 2.5 * r * r * (2.0 * std::f64::consts::PI / 5.0).sin();
    let (v, cen) = cell_volume_centroid(&m, CellIx(0));
    assert!((v - area * hgt).abs() < 1e-12, "{v}");
    assert!((cen.z - hgt / 2.0).abs() < 1e-12 && cen.x.abs() < 1e-12);
    let shapes: Vec<FaceShape> = (0..7).map(|f| m.face_shape(FaceIx::new(f))).collect();
    assert_eq!(
        shapes.iter().filter(|&&s| s == FaceShape::Polygon).count(),
        2
    );
    assert_eq!(shapes.iter().filter(|&&s| s == FaceShape::Quad).count(), 5);
}

#[test]
fn template_shapes_have_analytic_volumes_and_derived_corners() {
    for shape in [
        CellShape::Tet,
        CellShape::Hex,
        CellShape::Wedge,
        CellShape::Pyramid,
    ] {
        let m = single_cell(shape, &mut SerialAlloc::new());
        assert_eq!(m.cell_shape(CellIx(0)), shape);
        assert_valid(&m);
        assert!(
            (cell_volume(&m, CellIx(0)) - analytic_volume(shape)).abs() < 1e-14,
            "{shape:?}"
        );
        assert_eq!(
            m.cell_corners(CellIx(0)).unwrap().len(),
            shape.vertex_count().unwrap()
        );
        assert_eq!(m.cell_faces(CellIx(0)).len(), shape.local_faces().len());

        // The same cell built from loops only: corners are derived and the volume agrees.
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
        // Present the faces in a scrambled order and rotation.
        let mut loops: Vec<Vec<VertexIx>> = shape
            .local_faces()
            .iter()
            .map(|lf| lf.iter().map(|&i| vs[i as usize]).collect())
            .collect();
        loops.reverse();
        for l in &mut loops {
            l.rotate_left(1);
        }
        let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
        let added = b.add_polyhedron(&refs).unwrap();
        b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
        for (i, sf) in added.faces.iter().enumerate() {
            b.set_class(
                MeshEntityRef::Face(sf.face()),
                on(EntityKind::TopoFace, 1 + i as u64),
            )
            .unwrap();
        }
        let d = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
        assert_eq!(d.cell_shape(CellIx(0)), shape);
        let corners = d.cell_corners(CellIx(0)).expect("derived corners");
        assert_eq!(corners.len(), shape.vertex_count().unwrap());
        assert_valid(&d);
        assert!((cell_volume(&d, CellIx(0)) - analytic_volume(shape)).abs() < 1e-14);
    }
}

#[test]
fn template_shapes_mirrored_in_place() {
    let opts = FlattenOptions::default();
    for shape in [
        CellShape::Tet,
        CellShape::Hex,
        CellShape::Wedge,
        CellShape::Pyramid,
    ] {
        let m = single_cell(shape, &mut SerialAlloc::new());
        let mut b = ReferenceBuilder::new();
        let am = append_transformed(
            &mut b,
            &m,
            id(EntityKind::Instance, 1),
            &mirror_x(0.0),
            &opts,
        )
        .unwrap();
        let mut alloc = SerialAlloc::new();
        let mm = b.finish(&mut alloc).unwrap().mesh;
        assert_valid(&mm);
        assert!(
            (cell_volume(&mm, CellIx(0)) - analytic_volume(shape)).abs() < 1e-14,
            "{shape:?}"
        );
        assert_eq!(am.cell, vec![CellIx(0)]);
        // Corners are the old corners permuted by reflect_permutation.
        let old = m.cell_corners(CellIx(0)).unwrap();
        let want: Vec<VertexIx> = shape
            .reflect_permutation()
            .iter()
            .map(|&k| old[k as usize])
            .collect();
        assert_eq!(mm.cell_corners(CellIx(0)).unwrap(), want.as_slice());
        // Coordinates are mirrored.
        assert_eq!(mm.point(0).x, -m.point(0).x);
        assert_eq!(mm.point(1).x, -m.point(1).x);
    }
    // Negative control: mirrored points with the unpermuted corner list are inside out.
    let m = single_cell(CellShape::Hex, &mut SerialAlloc::new());
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = (0..8)
        .map(|i| {
            let p = m.point(i);
            let v = b.add_vertex(Point3::new(-p.x, p.y, p.z)).unwrap();
            b.set_class(
                MeshEntityRef::Vertex(v),
                on(EntityKind::TopoVertex, 1 + i as u64),
            )
            .unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    let bad = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
    let g = validate_geometry(&bad, &GeomTol::default());
    assert!(
        g.iter().any(|v| v.code == ViolationCode::GeomVolume),
        "{g:?}"
    );
}

#[test]
fn builder_is_atomic_and_rejects_bad_input() {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet)
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    // A cell with a repeated face: nothing is left behind (no stray faces).
    let f0 = b.add_face(&[vs[0], vs[2], vs[1]]).unwrap();
    let before = (b.face_count(), b.cell_count());
    let e = b
        .add_cell(&[SignedFace::new(f0, Side::Left); 4], None)
        .unwrap_err();
    assert!(matches!(e, MeshError::RepeatedCellFace(_)));
    assert_eq!((b.face_count(), b.cell_count()), before);
    // add_polyhedron rolls back the faces it created when the cell is rejected (here: 3 faces only).
    let loops: [&[VertexIx]; 3] = [
        &[vs[0], vs[1], vs[3]],
        &[vs[1], vs[2], vs[3]],
        &[vs[2], vs[0], vs[3]],
    ];
    let n = b.face_count();
    assert!(b.add_polyhedron(&loops).is_err());
    assert_eq!(b.face_count(), n);
    // Parameter kind mismatch is rejected, not rewritten.
    let bad = Classification {
        entity: id(EntityKind::TopoVertex, 1),
        param: Param::Curve(0.5),
    };
    assert!(matches!(
        b.set_class(MeshEntityRef::Vertex(vs[0]), bad),
        Err(MeshError::BadClassification(_))
    ));
    let ok = Classification {
        entity: id(EntityKind::TopoVolume, 1),
        param: Param::Volume([0.1, 0.2, 0.3]),
    };
    b.set_class(MeshEntityRef::Vertex(vs[0]), ok).unwrap();
    // Classification must be mandatory.
    let added = b.add_cell_from_vertices(CellShape::Tet, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    assert!(matches!(
        b.finish(&mut SerialAlloc::new()),
        Err(MeshError::Unclassified(_))
    ));
    // Non-manifold: a third cell on a face.
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet)
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    b.add_cell_from_vertices(CellShape::Tet, &vs).unwrap();
    let again = b.add_cell_from_vertices(CellShape::Tet, &vs);
    assert!(
        matches!(again, Err(MeshError::InconsistentOrientation(_))),
        "{again:?}"
    );
}
