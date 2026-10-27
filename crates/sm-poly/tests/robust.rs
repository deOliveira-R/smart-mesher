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
use sm_poly::{EdgeResolver, FaceMode, PolyError, PolyOptions, TopologyResolver, VertexClassResolver};
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

// ---- reflex interface edges (formerly a known gap) -------------------------------------------

type Notch = Box<dyn Fn([f64; 3]) -> bool>;

/// `tet_block([4,4,4])` re-built with `region(centroid)` naming the volume of each tet, classified
/// by plane groups (`build_mesh_volumes`).
fn regionized_block(in_region_2: &dyn Fn([f64; 3]) -> bool) -> ReferenceMesh {
    use sm_testkit::meshes::{CellSpec, build_mesh_volumes};
    let src = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap().mesh;
    let co = src.coords();
    let pts: Vec<[f64; 3]> = (0..src.vertex_count()).map(|v| [co.x[v], co.y[v], co.z[v]]).collect();
    let cells: Vec<(CellSpec, u64)> = (0..src.cell_count())
        .map(|c| {
            let k: Vec<usize> = src.cell_corners(CellIx::new(c)).unwrap().iter().map(|v| v.0 as usize).collect();
            let cen = centroid_of(&pts, &k);
            (CellSpec::Template(CellShape::Tet, k), if in_region_2(cen) { 2 } else { 1 })
        })
        .collect();
    build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new()).unwrap()
}

/// [`regionized_block`] with every free coordinate (not 0, 2 or 4) moved by up to `amp`.
fn regionized_block_perturbed(in_region_2: &dyn Fn([f64; 3]) -> bool, seed: u64, amp: f64) -> ReferenceMesh {
    use sm_testkit::meshes::{CellSpec, build_mesh_volumes};
    let src = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap().mesh;
    let co = src.coords();
    let pts0: Vec<[f64; 3]> = (0..src.vertex_count()).map(|v| [co.x[v], co.y[v], co.z[v]]).collect();
    let mut state = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut rnd = move || {
        // xorshift64*, mapped to [-1, 1)
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        (state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    };
    let pts: Vec<[f64; 3]> = pts0
        .iter()
        .map(|p| p.map(|x| if [0.0, 2.0, 4.0].contains(&x) { x } else { x + amp * rnd() }))
        .collect();
    let cells: Vec<(CellSpec, u64)> = (0..src.cell_count())
        .map(|c| {
            let k: Vec<usize> = src.cell_corners(CellIx::new(c)).unwrap().iter().map(|v| v.0 as usize).collect();
            let cen = centroid_of(&pts0, &k);
            (CellSpec::Template(CellShape::Tet, k), if in_region_2(cen) { 2 } else { 1 })
        })
        .collect();
    build_mesh_volumes(&pts, &cells, &mut SerialAlloc::new()).unwrap()
}

fn centroid_of(pts: &[[f64; 3]], k: &[usize]) -> [f64; 3] {
    [0, 1, 2].map(|a| k.iter().map(|&i| pts[i][a]).sum::<f64>() / 4.0)
}

/// Centroid of the tet of `tet_block([4,4,4])` nearest to `p`.
fn tet_near(p: [f64; 3]) -> [f64; 3] {
    let src = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap().mesh;
    let co = src.coords();
    let pts: Vec<[f64; 3]> = (0..src.vertex_count()).map(|v| [co.x[v], co.y[v], co.z[v]]).collect();
    let d2 = |c: [f64; 3]| (0..3).map(|a| (c[a] - p[a]).powi(2)).sum::<f64>();
    (0..src.cell_count())
        .map(|c| {
            let k: Vec<usize> = src.cell_corners(CellIx::new(c)).unwrap().iter().map(|v| v.0 as usize).collect();
            centroid_of(&pts, &k)
        })
        .min_by(|a, b| d2(*a).total_cmp(&d2(*b)))
        .unwrap()
}

/// Cells with a `GeomSideTet` finding of `validate_geometry(default)`.
fn side_tet_cells(m: &dyn MeshRead) -> Vec<usize> {
    let mut cells: Vec<usize> = validate_geometry(m, &GeomTol::default())
        .iter()
        .filter(|v| v.code == sm_mesh_api::ViolationCode::GeomSideTet)
        .filter_map(|v| match v.entity {
            Some(sm_mesh_api::MeshEntity::Cell(c)) => Some(c.get()),
            _ => None,
        })
        .collect();
    cells.sort_unstable();
    cells.dedup();
    cells
}

/// [`VertexClassResolver`], and for a curve of one interval (both ends on model vertices) an id
/// minted from the two end vertices, so both regions of an interface agree.
struct EndsResolver;

impl EdgeResolver for EndsResolver {
    fn resolve(&self, a: EntityId, b: EntityId, ends: [Classification; 2]) -> Option<EntityId> {
        VertexClassResolver.resolve(a, b, ends).or_else(|| {
            let [x, y] = ends.map(|c| c.entity);
            (x.kind() == EntityKind::TopoVertex && y.kind() == EntityKind::TopoVertex)
                .then(|| EntityId::new(EntityKind::TopoEdge, 1_000_000 + x.serial().min(y.serial()) * 1000 + x.serial().max(y.serial())))
                .flatten()
        })
    }
}

/// Notches, inclusions and concave corners: every dual passes `validate` and
/// `validate_geometry(default)` and conserves the volume. The cells at a reflex interface edge
/// that are not star-shaped about their centroid (formerly `GeomSideTet`, e.g. cells 12 and 96
/// of the `x<2,y>2` notch) are split into wedges; the favourable orientations split nothing.
#[test]
fn reflex_interface_edges_are_split_into_valid_wedges() {
    let one = tet_near([2.3, 2.2, 2.1]);
    let cases: Vec<(&str, Notch, Option<bool>)> = vec![
        ("x>2,y>2", Box::new(|c| c[0] > 2.0 && c[1] > 2.0), Some(false)),
        ("x<2,y<2", Box::new(|c| c[0] < 2.0 && c[1] < 2.0), Some(false)),
        ("x<2,z<2", Box::new(|c| c[0] < 2.0 && c[2] < 2.0), Some(false)),
        ("x>2,z>2", Box::new(|c| c[0] > 2.0 && c[2] > 2.0), Some(false)),
        ("x<2,y>2", Box::new(|c| c[0] < 2.0 && c[1] > 2.0), Some(true)),
        ("x>2,y<2", Box::new(|c| c[0] > 2.0 && c[1] < 2.0), Some(true)),
        ("y<2,z>2", Box::new(|c| c[1] < 2.0 && c[2] > 2.0), Some(true)),
        ("x<2,y<2,z>1", Box::new(|c| c[0] < 2.0 && c[1] < 2.0 && c[2] > 1.0), Some(true)),
        ("single tet", Box::new(move |c| (0..3).all(|a| (c[a] - one[a]).abs() < 1e-9)), Some(true)),
        ("cube [1,3]^3", Box::new(|c| c.iter().all(|x| *x > 1.0 && *x < 3.0)), Some(false)),
    ];
    let mut cases = cases;
    // the eight octants at (2, 2, 2): three reflex edges meet at an interior vertex
    for o in 0..8u32 {
        let name: &'static str = Box::leak(format!("octant {o}").into_boxed_str());
        let s = [o & 1, (o >> 1) & 1, (o >> 2) & 1].map(|b| b == 1);
        cases.push((name, Box::new(move |c| (0..3).all(|a| (c[a] > 2.0) == s[a])), None));
    }
    for (name, f, must_split) in &cases {
        let m = regionized_block(f);
        assert!(validate(&m).is_empty(), "{name}: {:?}", validate(&m));
        let r = &EndsResolver;
        for mode in [FaceMode::Planar, FaceMode::Keep] {
            let opts = PolyOptions::new().with_faces(mode);
            let (d, rep) = dual_prod(&m, &opts, r).unwrap_or_else(|e| panic!("{name} {mode:?}: {e}"));
            assert!(validate(&d).is_empty(), "{name} {mode:?}: {:?}", validate(&d));
            let g = validate_geometry(&d, &tol_for(mode));
            eprintln!(
                "{name} {mode:?}: split {} -> {} wedges, cells {}, side-tet {:?}",
                rep.split_cells,
                rep.wedges,
                rep.cells,
                side_tet_cells(&d)
            );
            assert!(g.is_empty(), "{name} {mode:?}: {:?}", &g[..g.len().min(3)]);
            assert!(rel(rep.total_volume, 64.0) < 1e-12, "{name} {mode:?}: volume {}", rep.total_volume);
            if mode == FaceMode::Planar
                && let Some(want) = must_split
            {
                assert_eq!(rep.split_cells > 0, *want, "{name}: split cells {}", rep.split_cells);
            }
            let (dr, rep_r) = dual_ref(&m, &opts, r).unwrap();
            assert_eq!(rep_r, rep, "{name} {mode:?}: builders agree");
            assert!(validate(&dr).is_empty());
        }
    }
}

/// The same notches on unstructured neighbourhoods: every coordinate that is not on the box
/// faces or on a notch plane (0, 2, 4) is moved by up to `amp` (seeded), so the interface stays
/// planar but the tets around the reflex edge are irregular.
#[test]
fn perturbed_reflex_notches_are_valid() {
    let notches: [(&str, Notch); 4] = [
        ("x<2,y>2", Box::new(|c| c[0] < 2.0 && c[1] > 2.0)),
        ("x>2,y>2", Box::new(|c| c[0] > 2.0 && c[1] > 2.0)),
        ("y<2,z>2", Box::new(|c| c[1] < 2.0 && c[2] > 2.0)),
        ("x<2,y<2,z<2", Box::new(|c| c[0] < 2.0 && c[1] < 2.0 && c[2] < 2.0)),
    ];
    let mut split_total = 0;
    for (name, f) in &notches {
        for seed in 0..4u64 {
            let m = regionized_block_perturbed(f, seed, 0.3);
            assert!(validate(&m).is_empty(), "{name}/{seed}: {:?}", validate(&m));
            let (d, rep) = dual_prod(&m, &PolyOptions::new(), &EndsResolver).unwrap_or_else(|e| panic!("{name}/{seed}: {e}"));
            let src_vol: f64 = (0..m.cell_count()).map(|c| sm_mesh_api::cell_volume(&m, CellIx::new(c))).sum();
            assert!(validate(&d).is_empty(), "{name}/{seed}: {:?}", validate(&d));
            let g = validate_geometry(&d, &GeomTol::default());
            assert!(g.is_empty(), "{name}/{seed}: split {} {:?}", rep.split_cells, &g[..g.len().min(3)]);
            assert!(rel(rep.total_volume, src_vol) < 1e-12, "{name}/{seed}");
            split_total += rep.split_cells;
        }
    }
    eprintln!("perturbed notches: {split_total} split cells");
}

/// The edge-manifold refusal class: two volumes alternating around the edge `x = y = 2`
/// (checkerboard) give each volume two chains of tets there, refused as `OpenRing`.
#[test]
fn checkerboard_edge_is_an_open_ring_error() {
    let m = regionized_block(&|c| (c[0] < 2.0) == (c[1] < 2.0));
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    let e = dual_prod(&m, &PolyOptions::new(), &EndsResolver).unwrap_err();
    assert!(matches!(e, PolyError::OpenRing { .. }), "{e:?}");
}

/// Split decisions and wedges are deterministic: repeated runs with 1 and 4 threads give
/// bit-identical meshes (perturbed notch with split cells).
#[test]
fn split_cells_are_deterministic() {
    let notch = |c: [f64; 3]| c[0] < 2.0 && c[1] > 2.0;
    let m = regionized_block_perturbed(&notch, 1, 0.3);
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let (d, rep) = dual_prod(&m, &PolyOptions::new().with_edges(Some(true)), &EndsResolver).unwrap();
            assert!(rep.split_cells > 0);
            (sm_mesh_api::topology_hash(&d), sm_mesh_api::geometry_hash(&d))
        })
    };
    sm_testkit::determinism::check(2, 4, run).unwrap();
}
