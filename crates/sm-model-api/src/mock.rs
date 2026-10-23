//! In-memory [`TopologyView`] for tests and for downstream crates that need a topology stub.

use crate::topology::{FaceSide, Moments, Sense, TopologyView};
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use smallvec::SmallVec;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
struct MockFace {
    outline: Vec<Point3>,
    sides: Vec<FaceSide>,
}

/// A hand-built topology. Faces added through [`MockTopology::add_box`] are shared (merged) with
/// existing coincident faces, so adjacent boxes get an interior face with two sides.
#[derive(Clone, Debug, Default)]
pub struct MockTopology {
    volumes: BTreeMap<EntityId, Option<Moments>>,
    faces: BTreeMap<EntityId, MockFace>,
    nv: u64,
    nf: u64,
}

/// Exact moments of an axis-aligned box.
pub fn box_moments(min: Point3, max: Point3) -> Moments {
    let d = [max.x - min.x, max.y - min.y, max.z - min.z];
    let c = [(max.x + min.x) / 2.0, (max.y + min.y) / 2.0, (max.z + min.z) / 2.0];
    let vol = d[0] * d[1] * d[2];
    let mut second = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            second[i][j] = vol * (c[i] * c[j] + if i == j { d[i] * d[i] / 12.0 } else { 0.0 });
        }
    }
    Moments { volume: vol, first: Vector3::new(vol * c[0], vol * c[1], vol * c[2]), second }
}

fn same_points(a: &[Point3], b: &[Point3]) -> bool {
    a.len() == b.len() && a.iter().all(|p| b.iter().any(|q| (*p - *q).norm() < 1e-12))
}

impl MockTopology {
    /// An empty topology.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a volume (moments optional).
    pub fn add_volume(&mut self, moments: Option<Moments>) -> EntityId {
        self.nv += 1;
        let id = EntityId::new(EntityKind::TopoVolume, self.nv).expect("serial in range");
        self.volumes.insert(id, moments);
        id
    }

    /// Add a face with an explicit outline and sides.
    pub fn add_face(&mut self, outline: Vec<Point3>, sides: &[FaceSide]) -> EntityId {
        self.nf += 1;
        let id = EntityId::new(EntityKind::TopoFace, self.nf).expect("serial in range");
        self.faces.insert(id, MockFace { outline, sides: sides.to_vec() });
        id
    }

    /// Add an axis-aligned box volume with exact moments. Returns the volume and its six faces
    /// in the order `[-x, +x, -y, +y, -z, +z]`; faces coincident with existing ones are shared.
    pub fn add_box(&mut self, min: Point3, max: Point3) -> (EntityId, [EntityId; 6]) {
        let v = self.add_volume(Some(box_moments(min, max)));
        let (x0, y0, z0, x1, y1, z1) = (min.x, min.y, min.z, max.x, max.y, max.z);
        let p = Point3::new;
        let outlines = [
            vec![p(x0, y0, z0), p(x0, y0, z1), p(x0, y1, z1), p(x0, y1, z0)],
            vec![p(x1, y0, z0), p(x1, y1, z0), p(x1, y1, z1), p(x1, y0, z1)],
            vec![p(x0, y0, z0), p(x1, y0, z0), p(x1, y0, z1), p(x0, y0, z1)],
            vec![p(x0, y1, z0), p(x0, y1, z1), p(x1, y1, z1), p(x1, y1, z0)],
            vec![p(x0, y0, z0), p(x0, y1, z0), p(x1, y1, z0), p(x1, y0, z0)],
            vec![p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1)],
        ];
        let mut ids = [v; 6];
        for (k, o) in outlines.into_iter().enumerate() {
            let existing = self.faces.iter().find(|(_, f)| same_points(&f.outline, &o)).map(|(id, _)| *id);
            ids[k] = match existing {
                Some(id) => {
                    // The existing face's normal points out of its first volume, so into this one.
                    let f = self.faces.get_mut(&id).expect("face exists");
                    f.sides.push(FaceSide { volume: v, sense: Sense::Reverse });
                    f.sides.sort();
                    id
                }
                None => self.add_face(o, &[FaceSide { volume: v, sense: Sense::Forward }]),
            };
        }
        (v, ids)
    }
}

impl TopologyView for MockTopology {
    fn volumes(&self) -> Vec<EntityId> {
        self.volumes.keys().copied().collect()
    }

    fn faces(&self) -> Vec<EntityId> {
        self.faces.keys().copied().collect()
    }

    fn face_sides(&self, face: EntityId) -> SmallVec<[FaceSide; 2]> {
        self.faces.get(&face).map(|f| f.sides.iter().copied().collect()).unwrap_or_default()
    }

    fn face_outline(&self, face: EntityId) -> Vec<Point3> {
        self.faces.get(&face).map(|f| f.outline.clone()).unwrap_or_default()
    }

    fn volume_moments(&self, volume: EntityId) -> Option<Moments> {
        self.volumes.get(&volume).copied().flatten()
    }
}
