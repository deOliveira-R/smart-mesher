//! W4-POLY-VORO acceptance 2 and 3: interior cells of Delaunay patches are the Voronoi cells
//! (truncated octahedra on a BCC lattice, orthogonal and convex on random points). The boundary
//! of such patches is not orthogonal, so the strict gate refuses them; the `test-hooks` function
//! `with_ungated_circumcentric` lets the interior be inspected.
#![allow(missing_docs)]

mod common;

use common::*;
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{CellIx, CellShape, Classification, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, TagValues, cell_volume};
use sm_poly::{DualPoints, EdgeResolver, GENERATOR_TAGS, PolyError, PolyOptions, face_nonplanarity};
use sm_quality::{RT_PLANARITY_TOL, cell_convexity};
use sm_testkit::meshes::{self, CellSpec};

fn circ() -> PolyOptions {
    PolyOptions::new().with_dual(DualPoints::Circumcentric)
}

struct Pair;

impl EdgeResolver for Pair {
    fn resolve(&self, a: EntityId, b: EntityId, _e: [Classification; 2]) -> Option<EntityId> {
        EntityId::new(EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
    }
}

/// Generator point of every cell, from the tag columns.
fn generators(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    let col = |k: usize| match &m.tag(MeshEntityKind::Cell, GENERATOR_TAGS[k]).expect("generator tag").values {
        TagValues::F64(v) => v.clone(),
        other => panic!("{other:?}"),
    };
    let (x, y, z) = (col(0), col(1), col(2));
    (0..x.len()).map(|i| [x[i], y[i], z[i]]).collect()
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Every face of cell `c` lies in the bisector plane of `c`'s generator and the generator of the
/// cell across it (to `tol` relative to the generator distance). Returns the face count.
fn check_bisectors(m: &dyn MeshRead, gen_: &[[f64; 3]], c: usize, tol: f64) -> usize {
    let co = m.coords();
    let sfs = m.cell_faces(CellIx::new(c));
    for sf in sfs {
        let nb = m.across(*sf).unwrap_or_else(|| panic!("interior cell {c} has a boundary face"));
        let (g1, g2) = (gen_[c], gen_[nb.get()]);
        let h = dist(g1, g2);
        for v in m.face_vertices(sf.face()) {
            let p = [co.x[v.get()], co.y[v.get()], co.z[v.get()]];
            assert!((dist(p, g1) - dist(p, g2)).abs() <= tol * h, "cell {c} face {:?}: {} vs {}", sf.face(), dist(p, g1), dist(p, g2));
        }
    }
    sfs.len()
}

#[test]
fn acc2_bcc_interior_cells_are_truncated_octahedra() {
    let n = 4;
    let (pts, cells, n_corner) = bcc_patch(n);
    let src = meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap();
    // the strict gate refuses the patch: its boundary is not orthogonal
    let e = dual_ref(&src, &circ(), &Pair).expect_err("strict gate refuses the BCC box");
    assert!(matches!(e, PolyError::NotOrthogonalDual { .. }), "{e}");
    let (m, _) = sm_poly::with_ungated_circumcentric(|| dual_ref(&src, &circ(), &Pair)).unwrap();
    let g = generators(&m);
    assert_eq!(g.len(), pts.len());
    let (mut checked, mut vols) = (0, 0.0);
    for (c, p) in pts.iter().enumerate() {
        // interior generators: corners one layer in, centres two layers in (all their tets are
        // honeycomb tets)
        let (lo, hi) = if c < n_corner { (1.0, n as f64 - 1.0) } else { (1.5, n as f64 - 1.5) };
        if !p.iter().all(|&x| x >= lo && x <= hi) {
            continue;
        }
        assert_eq!(g[c], *p, "cell {c}'s generator is its source point");
        assert_eq!(check_bisectors(&m, &g, c, 1e-12), 14, "cell {c}: 14 faces");
        let sizes: Vec<usize> = m.cell_faces(CellIx::new(c)).iter().map(|sf| m.face_vertices(sf.face()).len()).collect();
        assert_eq!(sizes.iter().filter(|&&k| k == 4).count(), 6, "6 squares");
        assert_eq!(sizes.iter().filter(|&&k| k == 6).count(), 8, "8 hexagons");
        let v = cell_volume(&m, CellIx::new(c));
        assert!(rel(v, 0.5) < 1e-12, "cell {c}: volume {v}");
        let cv = cell_convexity(&m, CellIx::new(c), RT_PLANARITY_TOL);
        assert!(cv.eligible && cv.excess <= 1e-12, "cell {c}: {cv:?}");
        vols += v;
        checked += 1;
    }
    assert_eq!(checked, 27 + 8);
    assert!(rel(vols, 0.5 * 35.0) < 1e-12);
    // boundary cells included: the ungated cells still tile the box, so the total is n^3
    let total: f64 = volumes(&m).iter().sum();
    assert!(rel(total, (n * n * n) as f64) < 1e-12, "total volume {total}");
}

/// splitmix64 seeded by the property's input: no clock, no thread state.
fn points(seed: u64, n: usize) -> Vec<[f64; 3]> {
    let mut s = seed;
    let mut next = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };
    (0..n).map(|_| [next(), next(), next()]).collect()
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(6))]

    #[test]
    fn acc3_random_delaunay_interior_cells_are_orthogonal(seed in any::<u64>()) {
        random_delaunay_case(seed);
    }
}

fn random_delaunay_case(seed: u64) {
    let pts = points(seed, 160);
    let dt = sm_scheme_tet::delaunay(&pts).expect("delaunay");
    let cells: Vec<CellSpec> = dt.tets.iter().map(|t| CellSpec::Template(CellShape::Tet, t.iter().map(|&v| v as usize).collect())).collect();
    let src: ReferenceMesh = meshes::build_mesh(&dt.points, &cells, &mut SerialAlloc::new()).unwrap();
    let (m, _) = sm_poly::with_ungated_circumcentric(|| dual_ref(&src, &circ(), &Pair)).unwrap();
    let g = generators(&m);
    // deep vertices: neither the vertex nor any tet-neighbour touches the hull
    let nv = pts.len();
    let mut on_hull = vec![false; nv];
    for f in 0..src.count(MeshEntityKind::Face) {
        if src.is_boundary_face(FaceIx::new(f)) {
            for v in src.face_vertices(FaceIx::new(f)) {
                on_hull[v.get()] = true;
            }
        }
    }
    let mut deep: Vec<bool> = on_hull.iter().map(|h| !h).collect();
    for t in &dt.tets {
        if t.iter().any(|&v| on_hull[v as usize]) {
            for &v in t {
                deep[v as usize] = false;
            }
        }
    }
    let mut checked = 0;
    for c in 0..g.len() {
        let Some(v) = (0..nv).find(|&v| g[c] == pts[v]) else { continue };
        if !deep[v] {
            continue;
        }
        check_bisectors(&m, &g, c, 1e-12);
        for sf in m.cell_faces(CellIx::new(c)) {
            assert!(face_nonplanarity(&m, sf.face()) <= 1e-12);
        }
        let cv = cell_convexity(&m, CellIx::new(c), RT_PLANARITY_TOL);
        assert!(cv.convex, "cell {c}: {cv:?}");
        assert!(cell_volume(&m, CellIx::new(c)) > 0.0);
        checked += 1;
    }
    assert!(checked > 10, "{checked} deep cells");
    // The ungated cells tile the hull: total volume equals the tet volume. 1e-8, not 1e-12: the
    // circumcentres of sliver tets of a random Delaunay set amplify rounding (observed up to 1e-9).
    let (tet_vol, dual_vol): (f64, f64) = (volumes(&src).iter().sum(), volumes(&m).iter().sum());
    assert!(rel(dual_vol, tet_vol) < 1e-8, "dual {dual_vol} vs tets {tet_vol}");
}

/// The strict gate and the star-shape check cannot see every folded or zero-area face (the BCC
/// boundary has some); a final validity check refuses such an output instead of returning it, and
/// leaves the allocator alone.
#[test]
fn circumcentric_output_that_fails_validation_is_refused() {
    let (pts, cells, _) = bcc_patch(4);
    let src = meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap();
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    let r = sm_poly::with_gate_off_circumcentric(|| {
        sm_poly::dualize(&src, &circ(), &Pair, sm_mesh_api::reference::ReferenceBuilder::new(), &mut alloc)
    });
    match r {
        Err(PolyError::InvalidOutput { count, first }) => assert!(count > 0 && !first.is_empty()),
        Err(e) => panic!("{e}"),
        Ok(_) => panic!("an invalid circumcentric output was returned"),
    }
    assert_eq!(alloc.state().collect::<Vec<_>>(), before);
}
