//! Robustness: perturbed lattices, several volumes, and the typed refusals of unsupported input.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, GeomTol, MeshBuild, MeshEdit, MeshEntityRef, MeshRead, SerialAlloc,
    VertexIx, validate, validate_geometry,
};
use sm_poly::{FaceMode, PolyError, PolyOptions, TopologyResolver, VertexClassResolver};
use sm_testkit::{lattice, meshes};

fn tol_for(mode: FaceMode) -> GeomTol {
    match mode {
        FaceMode::Keep => GeomTol::new().with_planarity(f64::INFINITY),
        _ => GeomTol::new(),
    }
}

#[test]
fn perturbed_lattices_stay_valid_and_conserve_volume() {
    for amp in [0.05, 0.15, 0.25] {
        for seed in 0..3u64 {
            let lf = lattice::tet_kuhn([3, 3, 3], [3.0; 3]).unwrap().perturbed(seed, amp).unwrap();
            let r = TopologyResolver(&lf.topo);
            // the true volume of the tets, independent of the dual
            let src_vol: f64 = (0..lf.mesh.cell_count()).map(|c| sm_mesh_api::cell_volume(&lf.mesh, CellIx::new(c))).sum();
            for mode in [FaceMode::Planar, FaceMode::Keep] {
                let (m, rep) = dual_prod(&lf.mesh, &PolyOptions::new().with_faces(mode), &r).unwrap();
                assert!(validate(&m).is_empty(), "amp {amp} seed {seed} {mode:?}: {:?}", validate(&m));
                let g = validate_geometry(&m, &tol_for(mode));
                assert!(g.is_empty(), "amp {amp} seed {seed} {mode:?}: {:?}", &g[..g.len().min(3)]);
                assert!(rel(rep.total_volume, src_vol) < 1e-12, "amp {amp} seed {seed} {mode:?}");
            }
        }
    }
}

#[test]
fn multi_volume_blocks_with_several_splits() {
    for (n, split) in [([4, 2, 2], 2), ([5, 3, 2], 2), ([5, 3, 2], 3), ([6, 3, 3], 2), ([6, 3, 3], 4)] {
        let f = meshes::tet_block_two_volumes(n, [1.0, 0.8, 1.3], split).unwrap();
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            let (m, rep) = dual_prod(&f.mesh, &PolyOptions::new().with_faces(mode), &VertexClassResolver)
                .unwrap_or_else(|e| panic!("{n:?}/{split} {mode:?}: {e}"));
            let g = validate_geometry(&m, &tol_for(mode));
            assert!(validate(&m).is_empty(), "{n:?}/{split}: {:?}", validate(&m));
            assert!(g.is_empty(), "{n:?}/{split} {mode:?}: {:?}", &g[..g.len().min(3)]);
            assert!(rel(rep.total_volume, f.volume) < 1e-12);
            // cells = vertices + interface vertices
            let iface = (n[1] + 1) * (n[2] + 1);
            assert_eq!(m.cell_count(), ((n[0] + 1) * (n[1] + 1) * (n[2] + 1) + iface) as usize);
        }
    }
}

/// Negative-case construction (no stage-0 fixture is non-manifold): two tets of one region that
/// touch only along the edge (0, 1).
fn bowtie() -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let pts: [[f64; 3]; 6] = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [0., 0., -1.], [1.0, 1.0, -1.0]];
    let vs: Vec<VertexIx> = pts.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap()).collect();
    let vol = Classification::on(EntityId::new(EntityKind::TopoVolume, 1).unwrap());
    for t in [[0, 1, 2, 3], [0, 1, 4, 5]] {
        let c = b.add_cell_from_vertices(CellShape::Tet, &t.map(|i| vs[i])).unwrap();
        b.set_class(MeshEntityRef::Cell(c.cell), vol).unwrap();
    }
    for f in 0..b.face_count() {
        let tf = Classification::on(EntityId::new(EntityKind::TopoFace, f as u64 + 1).unwrap());
        b.set_class(MeshEntityRef::Face(FaceIx::new(f)), tf).unwrap();
    }
    for (i, v) in vs.iter().enumerate() {
        // each vertex on a TopoVertex of its own keeps the dimension order valid
        let tv = Classification::on(EntityId::new(EntityKind::TopoVertex, i as u64 + 1).unwrap());
        b.set_class(MeshEntityRef::Vertex(*v), tv).unwrap();
    }
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

#[test]
fn bowtie_edge_is_an_open_ring_error() {
    // the tets around the shared edge form two chains
    let e = dual_ref(&bowtie(), &PolyOptions::new(), &PairResolver).unwrap_err();
    assert!(matches!(e, PolyError::OpenRing { edge } if edge == [VertexIx(0), VertexIx(1)]), "{e:?}");
}

#[test]
fn interface_not_on_a_model_face_is_refused() {
    // re-classify the interface faces of the two-volume fixture onto a volume
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    let mut m = f.mesh.clone();
    let vol = Classification::on(EntityId::new(EntityKind::TopoVolume, 1).unwrap());
    let iface: Vec<_> = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|f| m.face_right(*f).is_some_and(|r| m.cell_class(m.face_left(*f)).entity != m.cell_class(r).entity))
        .collect();
    assert_eq!(iface.len(), 2 * 2 * 2);
    for fi in &iface {
        MeshEdit::set_class(&mut m, sm_mesh_api::MeshEntity::Face(*fi), vol).unwrap();
    }
    let e = dual_ref(&m, &PolyOptions::new(), &VertexClassResolver).unwrap_err();
    assert!(matches!(e, PolyError::BadClassification { .. }), "{e:?}");
}

#[test]
fn a_vertex_with_one_feature_edge_is_unsupported() {
    // Declare one edge in the middle of a smooth face a TopoEdge: its two end vertices then have
    // exactly one split edge in the region.
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    let m = &f.mesh;
    // find a boundary edge strictly inside a model face: both end vertices classified on that face
    let mut pick = None;
    'outer: for fi in (0..m.face_count()).map(FaceIx::new).filter(|f| m.is_boundary_face(*f)) {
        let l = m.face_vertices(fi);
        for i in 0..3 {
            let (a, b) = (l[i], l[(i + 1) % 3]);
            if m.vertex_class(a).dim() == Some(2) && m.vertex_class(b).dim() == Some(2) {
                pick = Some((a, b, m.face_class(fi).entity));
                break 'outer;
            }
        }
    }
    let (a, b, face) = pick.expect("an interior boundary edge");
    let curve = Classification::on(EntityId::new(EntityKind::TopoEdge, 99).unwrap());
    let _ = face;
    let mut src2 = f.mesh.clone();
    // edge classes of the source: everything derives except the picked edge, which we set
    let mut explicit: Vec<(VertexIx, VertexIx, Classification)> = Vec::new();
    // feature edges need explicit classes in the source; give the model-edge segments theirs
    for fi in (0..m.face_count()).map(FaceIx::new) {
        let l = m.face_vertices(fi);
        for i in 0..l.len() {
            let (x, y) = (l[i], l[(i + 1) % l.len()]);
            let (cx, cy) = (m.vertex_class(x), m.vertex_class(y));
            let key = (x.min(y), x.max(y));
            if (key.0, key.1) == (a.min(b), a.max(b)) {
                explicit.push((key.0, key.1, curve));
            } else if cx.dim() == Some(1) && cy.dim().is_some_and(|d| d <= 1) && (cy.dim() == Some(0) || cx.entity == cy.entity) {
                explicit.push((key.0, key.1, cx));
            } else if cy.dim() == Some(1) && cx.dim() == Some(0) {
                explicit.push((key.0, key.1, cy));
            }
        }
    }
    explicit.sort_by_key(|e| (e.0, e.1));
    explicit.dedup_by_key(|e| (e.0, e.1));
    src2.ensure_edges(&mut SerialAlloc::new(), &explicit).expect("source edge table with one fake feature edge");
    let e = dual_ref(&src2, &PolyOptions::new(), &VertexClassResolver).unwrap_err();
    assert!(matches!(e, PolyError::UnsupportedFeature { .. }), "{e:?}");
}

fn rotate(mesh: &ReferenceMesh, axis: [f64; 3], angle: f64, scale: f64, shift: [f64; 3]) -> ReferenceMesh {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let k = [axis[0] / n, axis[1] / n, axis[2] / n];
    let (s, c) = angle.sin_cos();
    let mut out = mesh.clone();
    for i in 0..mesh.vertex_count() {
        let p = mesh.point(i);
        let v = [p.x, p.y, p.z];
        // Rodrigues
        let kv = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]];
        let kd = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
        let r: Vec<f64> = (0..3).map(|a| (v[a] * c + kv[a] * s + k[a] * kd * (1.0 - c)) * scale + shift[a]).collect();
        out.set_point(sm_mesh_api::PointIx(i as u32), Point3::new(r[0], r[1], r[2])).unwrap();
    }
    out
}

#[test]
fn rotated_scaled_and_shifted_blocks_behave_like_the_aligned_one() {
    for (n, h) in [([3, 3, 3], [1.0; 3]), ([4, 3, 2], [0.5, 1.0, 2.0])] {
        let f = meshes::tet_block(n, h).unwrap();
        let (_, base) = dual_ref(&f.mesh, &PolyOptions::new(), &VertexClassResolver).unwrap();
        for (axis, angle, scale, shift) in [
            ([1.0, 2.0, 3.0], 0.7, 1.0, [0.0; 3]),
            ([0.0, 1.0, 1.0], 2.1, 1e-3, [5e-3, 0.0, 1e-3]),
            ([1.0, -1.0, 0.5], 1.3, 1e3, [1e4, -2e4, 3e3]),
            ([0.3, 0.2, 1.0], std::f64::consts::FRAC_PI_4, 1.0, [100.0, 100.0, 100.0]),
        ] {
            let m0 = rotate(&f.mesh, axis, angle, scale, shift);
            for mode in [FaceMode::Planar, FaceMode::Keep] {
                let (m, rep) = dual_ref(&m0, &PolyOptions::new().with_faces(mode), &VertexClassResolver)
                    .unwrap_or_else(|e| panic!("{axis:?} {angle} {scale}: {e}"));
                assert!(validate(&m).is_empty(), "{:?}", validate(&m));
                let g = validate_geometry(&m, &tol_for(mode));
                assert!(g.is_empty(), "{axis:?} {angle} {scale} {mode:?}: {:?}", &g[..g.len().min(3)]);
                let want = f.volume * scale.powi(3);
                assert!(rel(rep.total_volume, want) < 1e-9, "{} vs {want}", rep.total_volume);
                assert_eq!(rep.cells, base.cells);
                assert_eq!(rep.edge_faces, base.edge_faces);
                // no fan was split because of rounding: boundary polygon count is unchanged
                assert_eq!(rep.boundary_faces, base.boundary_faces, "{axis:?} {angle} {scale} {mode:?}");
            }
        }
    }
}
