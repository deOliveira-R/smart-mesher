//! A small in-memory [`GeometryView`] for tests and examples: hand-built volumes/faces, plus a
//! builder from axis-aligned boxes that merges coincident faces (like a merged cell complex).
//! It is a reference implementation of the trait contract, not a topology kernel.

use crate::ids::IdAllocator;
use crate::topo::{FaceSides, GeometryView, Moments, TopologyView};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use std::collections::BTreeMap;

/// An axis-aligned box volume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxSpec {
    /// Lower corner.
    pub lo: [f64; 3],
    /// Upper corner.
    pub hi: [f64; 3],
}

impl BoxSpec {
    /// Analytic moments of the box (measure, centroid, `∫ x xᵀ`).
    pub fn moments(&self) -> Moments {
        let d = [self.hi[0] - self.lo[0], self.hi[1] - self.lo[1], self.hi[2] - self.lo[2]];
        let c = [(self.hi[0] + self.lo[0]) / 2.0, (self.hi[1] + self.lo[1]) / 2.0, (self.hi[2] + self.lo[2]) / 2.0];
        let v = d[0] * d[1] * d[2];
        let mut s = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] = v * (c[i] * c[j] + if i == j { d[i] * d[i] / 12.0 } else { 0.0 });
            }
        }
        Moments { measure: v, centroid: Point3::new(c[0], c[1], c[2]), second: s }
    }
}

#[derive(Clone, Copy, Debug)]
struct MockFace {
    sides: FaceSides,
    centroid: Point3,
    area: f64,
    normal: Vector3,
}

/// In-memory topology + geometry measures.
#[derive(Clone, Debug, Default)]
pub struct MockTopology {
    volumes: BTreeMap<EntityId, Moments>,
    faces: BTreeMap<EntityId, MockFace>,
}

impl MockTopology {
    /// An empty mock.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a volume.
    pub fn add_volume(&mut self, id: EntityId, m: Moments) {
        self.volumes.insert(id, m);
    }

    /// Add a face. `normal` points from `sides.fwd` to `sides.rev` (outward on boundary faces).
    pub fn add_face(&mut self, id: EntityId, sides: FaceSides, centroid: Point3, area: f64, normal: Vector3) {
        self.faces.insert(id, MockFace { sides, centroid, area, normal });
    }

    /// Build from boxes: shared rectangles become one interior face. Returns the mock and the volume
    /// ids in input order.
    pub fn from_boxes(ids: &mut dyn IdAllocator, boxes: &[BoxSpec]) -> (Self, Vec<EntityId>) {
        type Key = (usize, i64, i64, i64, i64, i64);
        let q = |x: f64| (x * 1e9).round() as i64;
        let mut topo = Self::new();
        let mut vids = Vec::new();
        let mut users: BTreeMap<Key, Vec<(EntityId, bool)>> = BTreeMap::new();
        let mut geom: BTreeMap<Key, (Point3, f64)> = BTreeMap::new();
        for b in boxes {
            let v = ids.alloc(EntityKind::TopoVolume);
            vids.push(v);
            topo.add_volume(v, b.moments());
            for a in 0..3 {
                let (u, w) = ((a + 1) % 3, (a + 2) % 3);
                for hi in [false, true] {
                    let coord = if hi { b.hi[a] } else { b.lo[a] };
                    let key: Key = (a, q(coord), q(b.lo[u]), q(b.hi[u]), q(b.lo[w]), q(b.hi[w]));
                    let mut c = [0.0; 3];
                    c[a] = coord;
                    c[u] = (b.lo[u] + b.hi[u]) / 2.0;
                    c[w] = (b.lo[w] + b.hi[w]) / 2.0;
                    geom.insert(key, (Point3::new(c[0], c[1], c[2]), (b.hi[u] - b.lo[u]) * (b.hi[w] - b.lo[w])));
                    users.entry(key).or_default().push((v, hi));
                }
            }
        }
        for (key, us) in users {
            let id = ids.alloc(EntityKind::TopoFace);
            let (c, area) = geom[&key];
            let mut axis = [0.0; 3];
            axis[key.0] = 1.0;
            let ax = Vector3::new(axis[0], axis[1], axis[2]);
            let (sides, normal) = match us.as_slice() {
                [(v, hi)] => (FaceSides { fwd: Some(*v), rev: None }, if *hi { ax } else { -ax }),
                [(v1, h1), (v2, _)] => {
                    let (fwd, rev) = if *h1 { (*v1, *v2) } else { (*v2, *v1) };
                    (FaceSides { fwd: Some(fwd), rev: Some(rev) }, ax)
                }
                _ => continue,
            };
            topo.add_face(id, sides, c, area, normal);
        }
        (topo, vids)
    }

    /// Find the face whose centroid is within `tol` of `p`.
    pub fn find_face(&self, p: Point3, tol: f64) -> Option<EntityId> {
        self.faces.iter().find(|(_, f)| (f.centroid - p).norm() <= tol).map(|(&id, _)| id)
    }
}

impl TopologyView for MockTopology {
    fn volumes(&self) -> Vec<EntityId> {
        self.volumes.keys().copied().collect()
    }
    fn faces(&self) -> Vec<EntityId> {
        self.faces.keys().copied().collect()
    }
    fn face_sides(&self, face: EntityId) -> Option<FaceSides> {
        self.faces.get(&face).map(|f| f.sides)
    }
}

impl GeometryView for MockTopology {
    fn face_centroid(&self, face: EntityId) -> Option<Point3> {
        self.faces.get(&face).map(|f| f.centroid)
    }
    fn face_area(&self, face: EntityId) -> Option<f64> {
        self.faces.get(&face).map(|f| f.area)
    }
    fn face_normal(&self, face: EntityId) -> Option<Vector3> {
        self.faces.get(&face).map(|f| f.normal)
    }
    fn volume_moments(&self, volume: EntityId) -> Option<Moments> {
        self.volumes.get(&volume).copied()
    }
}
