//! Shared helpers: fixture builders and mesh invariants.
#![allow(dead_code)]

use std::collections::BTreeMap;

use sm_core::{EntityId, Point3};
use sm_tess::Tessellation;
use sm_topo_api::fixtures::{FixtureError, with_op};
use sm_topo_api::{CellComplex, Dim, OpId, Side, TopologyView, TopologyViewExt};

/// Build a complex with one operation.
pub fn build<T>(f: impl FnOnce(&mut CellComplex) -> Result<T, FixtureError>) -> (CellComplex, T) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), f).expect("fixture");
    (c, ids)
}

pub fn all_faces(view: &dyn TopologyView) -> Vec<EntityId> {
    view.cells(Dim::Face).to_vec()
}

pub fn p64(t: &Tessellation, i: u32) -> [f64; 3] {
    let p = t.position(i);
    [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Vector area (cross / 2) of a triangle, computed in f64 from the f32 positions.
pub fn tri_vector_area(t: &Tessellation, tri: &[u32]) -> [f64; 3] {
    let (a, b, c) = (p64(t, tri[0]), p64(t, tri[1]), p64(t, tri[2]));
    let n = cross(sub(b, a), sub(c, a));
    [0.5 * n[0], 0.5 * n[1], 0.5 * n[2]]
}

pub fn tri_area(t: &Tessellation, tri: &[u32]) -> f64 {
    norm(tri_vector_area(t, tri))
}

pub fn face_area(t: &Tessellation, face: EntityId) -> f64 {
    t.face_indices(face).expect("face").chunks(3).map(|tri| tri_area(t, tri)).sum()
}

pub fn total_area(t: &Tessellation) -> f64 {
    t.indices.chunks(3).map(|tri| tri_area(t, tri)).sum()
}

/// Circumradius of a triangle.
pub fn circumradius(t: &Tessellation, tri: &[u32]) -> f64 {
    let (a, b, c) = (p64(t, tri[0]), p64(t, tri[1]), p64(t, tri[2]));
    norm(sub(a, b)) * norm(sub(b, c)) * norm(sub(c, a)) / (4.0 * tri_area(t, tri))
}

/// Per-face structural invariants:
/// - no directed edge twice (consistent orientation);
/// - every polyline segment of a face edge occurs in the face's triangles exactly as often as the
///   face uses the edge, in the direction of the coedge (the face lies on the left);
/// - every other undirected edge occurs exactly twice (interior).
pub fn check_face_structure(view: &dyn TopologyView, t: &Tessellation, face: EntityId) {
    let tris: Vec<&[u32]> = t.face_indices(face).expect("face tessellated").chunks(3).collect();
    let mut directed: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    let mut undirected: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for tri in &tris {
        assert!(tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2], "degenerate triangle in {face:?}");
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            *directed.entry((a, b)).or_default() += 1;
            *undirected.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    assert!(directed.values().all(|&c| c == 1), "a directed edge is used twice in {face:?}");
    let mut boundary: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for ce in view.coedges(face) {
        if ce.kind == sm_topo_api::types::CoedgeKind::Pole {
            continue;
        }
        let poly = t.edge_polyline(ce.key.edge).expect("edge polyline");
        for w in poly.windows(2) {
            let (a, b) = if ce.key.side == Side::Left { (w[0], w[1]) } else { (w[1], w[0]) };
            assert!(directed.contains_key(&(a, b)), "boundary segment {a}->{b} of {:?} missing or reversed in {face:?}", ce.key.edge);
            *boundary.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    for (e, &c) in &undirected {
        let expect = boundary.get(e).copied().unwrap_or(2);
        assert_eq!(c, expect, "edge {e:?} of {face:?} used {c} times, expected {expect}");
    }
    for e in boundary.keys() {
        assert!(undirected.contains_key(e));
    }
}

/// Unit normal of a face at a point near `p` (carrier sense applied), from the surface.
pub fn face_normal_at(view: &dyn TopologyView, face: EntityId, p: [f64; 3]) -> Option<[f64; 3]> {
    let sm_topo_api::Carrier::Surface(sc) = view.carrier(face) else { return None };
    let s = view.shapes().surface(sc.surface)?;
    let pr = s.project(Point3::new(p[0], p[1], p[2]), None).ok()?;
    let n = s.normal(pr.uv).ok()?;
    let sign = if sc.sense == sm_topo_api::Sense::Reversed { -1.0 } else { 1.0 };
    Some([sign * n.x, sign * n.y, sign * n.z])
}

/// Every triangle's normal agrees in sign with the face normal at its centroid.
pub fn check_normal_signs(view: &dyn TopologyView, t: &Tessellation, face: EntityId) {
    for tri in t.face_indices(face).expect("face").chunks(3) {
        let (a, b, c) = (p64(t, tri[0]), p64(t, tri[1]), p64(t, tri[2]));
        let cen = [(a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0, (a[2] + b[2] + c[2]) / 3.0];
        let n = face_normal_at(view, face, cen).expect("surface normal");
        let tn = tri_vector_area(t, tri);
        assert!(dot(tn, n) > 0.0, "triangle {tri:?} of {face:?} disagrees with the face normal");
    }
}

/// Stable 64-bit digest of every output buffer, by bit pattern.
pub fn digest(t: &Tessellation) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |x: u64| {
        h ^= x;
        h = h.wrapping_mul(0x0100_0000_01b3);
    };
    t.positions.iter().for_each(|x| eat(u64::from(x.to_bits())));
    t.indices.iter().for_each(|&x| eat(u64::from(x)));
    t.edge_indices.iter().for_each(|&x| eat(u64::from(x)));
    t.faces.iter().for_each(|r| {
        eat(r.face.to_raw());
        eat(u64::from(r.first_index));
        eat(u64::from(r.index_count));
    });
    t.edges.iter().for_each(|r| {
        eat(r.edge.to_raw());
        eat(u64::from(r.first));
        eat(u64::from(r.count));
    });
    h
}

pub fn pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

/// Largest distance from the mesh of `face` to the exact surface, with `dist` giving the
/// unsigned distance of a point to the surface. Samples every triangle at barycentric lattice
/// points `(i, j)/10` with `i + j <= 10` (vertices, edges, interior) plus the exact centroid.
pub fn max_deviation(t: &Tessellation, face: EntityId, dist: impl Fn([f64; 3]) -> f64) -> f64 {
    let mut worst: f64 = 0.0;
    for tri in t.face_indices(face).expect("face").chunks(3) {
        let (a, b, c) = (p64(t, tri[0]), p64(t, tri[1]), p64(t, tri[2]));
        for i in 0..=10 {
            for j in 0..=(10 - i) {
                let (wb, wc) = (f64::from(i) / 10.0, f64::from(j) / 10.0);
                let wa = 1.0 - wb - wc;
                let p = [0, 1, 2].map(|k| wa * a[k] + wb * b[k] + wc * c[k]);
                worst = worst.max(dist(p));
            }
        }
        let cen = [0, 1, 2].map(|k| (a[k] + b[k] + c[k]) / 3.0);
        worst = worst.max(dist(cen));
    }
    worst
}

/// f32 rounding slack for positions of magnitude `scale`.
pub fn f32_slack(scale: f64) -> f64 {
    4.0 * f64::from(f32::EPSILON) * scale
}
