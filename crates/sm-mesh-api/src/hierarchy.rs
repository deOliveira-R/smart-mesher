//! Several meshes over one model: fine and coarse levels with fine -> coarse maps.

use crate::error::MeshError;
use crate::point::{NONE, Point, Stratum};
use crate::traits::MeshView;
use indexmap::IndexMap;
use sm_core::EntityId;
use std::sync::Arc;

/// Shared, thread-safe handle to a mesh.
pub type MeshHandle = Arc<dyn MeshView + Send + Sync>;

/// Map from a fine mesh to a coarser mesh of the same model (CMFD, multigrid).
#[derive(Clone, Debug, PartialEq)]
pub struct FineToCoarse {
    /// Id of the fine mesh.
    pub fine: EntityId,
    /// Id of the coarse mesh.
    pub coarse: EntityId,
    /// Coarse cell index for every fine cell.
    pub cell_map: Vec<u32>,
    /// Optional: for every fine face, the coarse face it lies on, or [`NONE`] when it is
    /// interior to a coarse cell. When present it is checked against the cell map.
    pub face_map: Option<Vec<u32>>,
}

impl FineToCoarse {
    /// The coarse cell containing a fine cell.
    pub fn coarse_cell(&self, fine_cell: Point) -> Point {
        Point::cell(self.cell_map[fine_cell.index as usize])
    }

    /// Fine cells of a coarse cell, ascending.
    pub fn children(&self, coarse_cell: Point) -> Vec<Point> {
        self.cell_map
            .iter()
            .enumerate()
            .filter(|&(_, &c)| c == coarse_cell.index)
            .map(|(i, _)| Point::cell(i as u32))
            .collect()
    }

    /// Check shape and consistency against the two meshes.
    pub fn validate(&self, fine: &dyn MeshView, coarse: &dyn MeshView) -> Result<(), MeshError> {
        let bad = |s: &str| Err(MeshError::InvalidMap(s.to_string()));
        if self.fine != fine.mesh_id() || self.coarse != coarse.mesh_id() {
            return bad("mesh ids do not match the map");
        }
        if self.cell_map.len() != fine.count(Stratum::Cell) as usize {
            return bad("cell_map length != fine cell count");
        }
        let nc = coarse.count(Stratum::Cell);
        if self.cell_map.iter().any(|&c| c >= nc) {
            return bad("cell_map target out of range");
        }
        let Some(fm) = &self.face_map else { return Ok(()) };
        if fm.len() != fine.count(Stratum::Face) as usize {
            return bad("face_map length != fine face count");
        }
        for (i, &cf) in fm.iter().enumerate() {
            let sides = fine.face_sides(Point::face(i as u32));
            let a = self.coarse_cell(sides.left).index;
            let b = sides.right.map(|r| self.coarse_cell(r).index);
            if cf == NONE {
                if b.is_some_and(|b| b != a) {
                    return bad("face between different coarse cells has no coarse face");
                }
                continue;
            }
            if cf >= coarse.count(Stratum::Face) {
                return bad("face_map target out of range");
            }
            let cs = coarse.face_sides(Point::face(cf));
            let want = match b {
                Some(b) if b != a => {
                    let mut w = [a, b];
                    w.sort_unstable();
                    w.to_vec()
                }
                Some(_) => return bad("face interior to a coarse cell mapped to a coarse face"),
                None => vec![a],
            };
            let mut have = vec![cs.left.index];
            have.extend(cs.right.map(|r| r.index));
            have.sort_unstable();
            if have != want {
                return bad("coarse face does not separate the mapped coarse cells");
            }
        }
        Ok(())
    }
}

/// Meshes of one model plus the maps between them.
#[derive(Default)]
pub struct MeshCollection {
    /// The model these meshes discretize (optional).
    pub model: Option<EntityId>,
    meshes: IndexMap<EntityId, MeshHandle>,
    maps: Vec<FineToCoarse>,
}

impl MeshCollection {
    /// An empty collection.
    pub fn new(model: Option<EntityId>) -> Self {
        Self { model, meshes: IndexMap::new(), maps: Vec::new() }
    }

    /// Add a mesh under its own `mesh_id`. Replaces nothing: duplicates are rejected.
    pub fn add_mesh(&mut self, mesh: MeshHandle) -> Result<EntityId, MeshError> {
        let id = mesh.mesh_id();
        if self.meshes.contains_key(&id) {
            return Err(MeshError::InvalidMap(format!("mesh {id:?} already present")));
        }
        self.meshes.insert(id, mesh);
        Ok(id)
    }

    /// Add a validated fine -> coarse map. A fine mesh has at most one coarser mesh.
    pub fn add_map(&mut self, map: FineToCoarse) -> Result<(), MeshError> {
        let fine = self.mesh(map.fine)?;
        let coarse = self.mesh(map.coarse)?;
        map.validate(fine.as_ref(), coarse.as_ref())?;
        if self.maps.iter().any(|m| m.fine == map.fine) {
            return Err(MeshError::InvalidMap("fine mesh already has a coarse map".into()));
        }
        self.maps.push(map);
        Ok(())
    }

    /// Look up a mesh.
    pub fn mesh(&self, id: EntityId) -> Result<&MeshHandle, MeshError> {
        self.meshes.get(&id).ok_or(MeshError::UnknownMesh(id))
    }

    /// Mesh ids in insertion order.
    pub fn mesh_ids(&self) -> Vec<EntityId> {
        self.meshes.keys().copied().collect()
    }

    /// The map from `fine` to its coarser mesh, if any.
    pub fn coarser_of(&self, fine: EntityId) -> Option<&FineToCoarse> {
        self.maps.iter().find(|m| m.fine == fine)
    }

    /// Chain of mesh ids from `fine` through successively coarser levels (inclusive).
    pub fn levels_from(&self, fine: EntityId) -> Vec<EntityId> {
        let mut out = vec![fine];
        let mut cur = fine;
        while let Some(m) = self.coarser_of(cur) {
            if out.contains(&m.coarse) {
                break;
            }
            out.push(m.coarse);
            cur = m.coarse;
        }
        out
    }
}
