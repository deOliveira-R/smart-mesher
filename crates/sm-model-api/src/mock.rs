//! A table-backed [`TopologyView`] for tests and for other crates' contract tests.
//!
//! [`TableMock`] stores volumes and faces explicitly. [`TableMock::from_boxes`] builds the
//! shared-face topology of axis-aligned boxes whose touching faces coincide exactly.

use crate::topology::{FaceProps, FaceSides, MassProps, TopologyView};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use std::collections::BTreeMap;

/// Explicit tables of volumes and faces.
#[derive(Clone, Debug, Default)]
pub struct TableMock {
    volumes: BTreeMap<EntityId, MassProps>,
    faces: BTreeMap<EntityId, (FaceSides, FaceProps)>,
}

impl TableMock {
    /// An empty topology.
    pub fn new() -> Self {
        Self::default()
    }

    /// The id `TopoVolume#n`. Panics only if `n` exceeds the 56-bit serial range.
    pub fn vid(n: u64) -> EntityId {
        EntityId::new(EntityKind::TopoVolume, n).expect("serial in range")
    }

    /// The id `TopoFace#n`. Panics only if `n` exceeds the 56-bit serial range.
    pub fn fid(n: u64) -> EntityId {
        EntityId::new(EntityKind::TopoFace, n).expect("serial in range")
    }

    /// Add a volume.
    pub fn add_volume(&mut self, id: EntityId, props: MassProps) {
        self.volumes.insert(id, props);
    }

    /// Add a face.
    pub fn add_face(&mut self, id: EntityId, sides: FaceSides, props: FaceProps) {
        self.faces.insert(id, (sides, props));
    }

    /// Faces whose centroid lies on `axis = coord` and whose normal is along `axis`, ascending.
    pub fn faces_on_axis_plane(&self, axis: usize, coord: f64) -> Vec<EntityId> {
        let c = |p: Point3| [p.x, p.y, p.z][axis];
        let n = |v: Vector3| [v.x, v.y, v.z][axis];
        self.faces.iter().filter(|(_, (_, p))| (c(p.centroid) - coord).abs() < 1e-9 && n(p.normal).abs() > 0.999_999).map(|(id, _)| *id).collect()
    }

    /// Topology of axis-aligned boxes `(min, max)`. Box `i` becomes `TopoVolume#(i+1)`; faces are
    /// numbered in a canonical geometric order. Touching faces must coincide exactly to be shared.
    pub fn from_boxes(boxes: &[([f64; 3], [f64; 3])]) -> Self {
        let q = |x: f64| (x * 1e9).round() as i64;
        type Key = (usize, i64, i64, i64, i64, i64);
        let mut by_key: BTreeMap<Key, Vec<(usize, f64)>> = BTreeMap::new();
        let mut m = Self::new();
        for (i, (lo, hi)) in boxes.iter().enumerate() {
            let len = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
            let vol = len[0] * len[1] * len[2];
            let mut s = [[0.0; 3]; 3];
            for a in 0..3 {
                s[a][a] = vol * len[a] * len[a] / 12.0;
            }
            let centroid = Point3::new((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0);
            m.add_volume(Self::vid(i as u64 + 1), MassProps { volume: vol, centroid, second_moment: s });
            for a in 0..3 {
                let (b, c) = ((a + 1) % 3, (a + 2) % 3);
                for (coord, sign) in [(lo[a], -1.0), (hi[a], 1.0)] {
                    let key = (a, q(coord), q(lo[b]), q(hi[b]), q(lo[c]), q(hi[c]));
                    by_key.entry(key).or_default().push((i, sign));
                }
            }
        }
        for (n, (key, users)) in by_key.into_iter().enumerate() {
            let (a, coord, lb, hb, lc, hc) = key;
            let f = |x: i64| x as f64 / 1e9;
            let (b, c) = ((a + 1) % 3, (a + 2) % 3);
            let mut p = [0.0; 3];
            p[a] = f(coord);
            p[b] = (f(lb) + f(hb)) / 2.0;
            p[c] = (f(lc) + f(hc)) / 2.0;
            let (left, sign) = users[0];
            let right = users.get(1).map(|u| Self::vid(u.0 as u64 + 1));
            let mut nv = [0.0; 3];
            nv[a] = sign;
            m.add_face(
                Self::fid(n as u64 + 1),
                FaceSides { left: Some(Self::vid(left as u64 + 1)), right },
                FaceProps { centroid: Point3::new(p[0], p[1], p[2]), normal: Vector3::new(nv[0], nv[1], nv[2]), area: (f(hb) - f(lb)) * (f(hc) - f(lc)) },
            );
        }
        m
    }
}

impl TopologyView for TableMock {
    fn volumes(&self) -> Vec<EntityId> {
        self.volumes.keys().copied().collect()
    }
    fn faces(&self) -> Vec<EntityId> {
        self.faces.keys().copied().collect()
    }
    fn volume_faces(&self, volume: EntityId) -> Vec<EntityId> {
        self.faces.iter().filter(|(_, (s, _))| s.left == Some(volume) || s.right == Some(volume)).map(|(id, _)| *id).collect()
    }
    fn face_sides(&self, face: EntityId) -> Option<FaceSides> {
        self.faces.get(&face).map(|f| f.0)
    }
    fn volume_mass_properties(&self, volume: EntityId) -> Option<MassProps> {
        self.volumes.get(&volume).copied()
    }
    fn face_properties(&self, face: EntityId) -> Option<FaceProps> {
        self.faces.get(&face).map(|f| f.1)
    }
}
