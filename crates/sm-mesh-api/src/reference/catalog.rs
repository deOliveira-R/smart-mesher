//! Reference catalog: meshes of one model, level links and instanced views.

use crate::error::{MeshError, Result};
use crate::hierarchy::{CatalogEdit, CellMap, InstancedView, MeshCatalog};
use crate::index::CellIx;
use crate::read::MeshRead;
use indexmap::IndexMap;
use sm_core::EntityId;

struct Entry {
    mesh: Box<dyn MeshRead>,
    level: u32,
    coarse: Option<(EntityId, CellMap)>,
}

/// In-memory [`MeshCatalog`] / [`CatalogEdit`].
#[derive(Default)]
pub struct ReferenceCatalog {
    meshes: IndexMap<EntityId, Entry>,
    views: Vec<InstancedView>,
}

impl ReferenceCatalog {
    /// An empty catalog.
    pub fn new() -> Self {
        Self::default()
    }
}

impl MeshCatalog for ReferenceCatalog {
    fn mesh_ids(&self) -> Vec<EntityId> {
        let mut v: Vec<EntityId> = self.meshes.keys().copied().collect();
        v.sort_unstable();
        v
    }
    fn mesh(&self, id: EntityId) -> Option<&dyn MeshRead> {
        self.meshes.get(&id).map(|e| e.mesh.as_ref())
    }
    fn level(&self, id: EntityId) -> Option<u32> {
        self.meshes.get(&id).map(|e| e.level)
    }
    fn coarse_of(&self, fine: EntityId) -> Option<(EntityId, &CellMap)> {
        self.meshes.get(&fine)?.coarse.as_ref().map(|(id, m)| (*id, m))
    }
    fn views(&self) -> &[InstancedView] {
        &self.views
    }
}

impl CatalogEdit for ReferenceCatalog {
    fn insert_mesh(&mut self, mesh: Box<dyn MeshRead>, level: u32) -> Result<()> {
        let id = mesh.mesh_id();
        if self.meshes.contains_key(&id) {
            return Err(MeshError::Hierarchy(format!("mesh {id:?} already present")));
        }
        self.meshes.insert(id, Entry { mesh, level, coarse: None });
        Ok(())
    }

    fn link_levels(&mut self, fine: EntityId, coarse: EntityId, map: CellMap) -> Result<()> {
        let f = self.meshes.get(&fine).ok_or(MeshError::UnknownMesh(fine))?;
        let c = self.meshes.get(&coarse).ok_or(MeshError::UnknownMesh(coarse))?;
        if c.level != f.level + 1 {
            return Err(MeshError::Hierarchy("coarse level must be exactly fine level + 1".into()));
        }
        if map.fine_len() != f.mesh.cell_count() || map.coarse_len() != c.mesh.cell_count() {
            return Err(MeshError::Hierarchy("map sizes do not match the meshes".into()));
        }
        for i in 0..map.fine_len() {
            let fc = CellIx::new(i);
            if f.mesh.cell_class(fc).entity != c.mesh.cell_class(map.coarse_of(fc)).entity {
                return Err(MeshError::Hierarchy(format!("fine cell {fc:?} and its coarse cell lie on different model entities")));
            }
        }
        if let Some(e) = self.meshes.get_mut(&fine) {
            e.coarse = Some((coarse, map));
        }
        Ok(())
    }

    fn insert_view(&mut self, view: InstancedView) -> Result<()> {
        if let Some(i) = view.instances.iter().find(|i| !self.meshes.contains_key(&i.prototype)) {
            return Err(MeshError::UnknownMesh(i.prototype));
        }
        self.views.push(view);
        Ok(())
    }
}
