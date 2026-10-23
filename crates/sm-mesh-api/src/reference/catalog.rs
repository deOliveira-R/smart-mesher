//! Reference catalog: meshes of one model, level maps and instanced views.

use crate::error::{MeshError, Result};
use crate::hierarchy::{CatalogEdit, LevelMap, MeshCatalog};
use crate::instance::InstancedView;
use crate::read::MeshRead;
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// In-memory [`MeshCatalog`] / [`CatalogEdit`].
#[derive(Default)]
pub struct ReferenceCatalog {
    ids: Vec<EntityId>,
    meshes: BTreeMap<EntityId, Box<dyn MeshRead>>,
    maps: BTreeMap<(EntityId, EntityId), LevelMap>,
    views: Vec<InstancedView>,
}

impl ReferenceCatalog {
    /// An empty catalog.
    pub fn new() -> Self {
        Self::default()
    }
}

impl MeshCatalog for ReferenceCatalog {
    fn mesh_ids(&self) -> &[EntityId] {
        &self.ids
    }
    fn mesh(&self, id: EntityId) -> Option<&dyn MeshRead> {
        self.meshes.get(&id).map(|m| m.as_ref())
    }
    fn maps_from(&self, fine: EntityId) -> Vec<(EntityId, &LevelMap)> {
        self.maps
            .iter()
            .filter(|((f, _), _)| *f == fine)
            .map(|((_, c), m)| (*c, m))
            .collect()
    }
    fn maps_to(&self, coarse: EntityId) -> Vec<(EntityId, &LevelMap)> {
        let mut v: Vec<(EntityId, &LevelMap)> = self
            .maps
            .iter()
            .filter(|((_, c), _)| *c == coarse)
            .map(|((f, _), m)| (*f, m))
            .collect();
        v.sort_by_key(|(f, _)| *f);
        v
    }
    fn views(&self) -> &[InstancedView] {
        &self.views
    }
}

impl CatalogEdit for ReferenceCatalog {
    fn insert_mesh(&mut self, mesh: Box<dyn MeshRead>) -> Result<()> {
        let id = mesh.mesh_id();
        if id.kind() != EntityKind::Mesh || self.meshes.contains_key(&id) {
            return Err(MeshError::Hierarchy(format!(
                "mesh id {id:?} is not a new Mesh id"
            )));
        }
        let at = self.ids.partition_point(|x| *x < id);
        self.ids.insert(at, id);
        self.meshes.insert(id, mesh);
        Ok(())
    }

    fn link(&mut self, map: LevelMap) -> Result<()> {
        let (f, c) = (map.fine_id(), map.coarse_id());
        if f == c {
            return Err(MeshError::Hierarchy(
                "a mesh cannot be linked to itself".into(),
            ));
        }
        let fm = self.meshes.get(&f).ok_or(MeshError::UnknownMesh(f))?;
        let cm = self.meshes.get(&c).ok_or(MeshError::UnknownMesh(c))?;
        map.check_meshes(fm.as_ref(), cm.as_ref())?;
        if self.maps.contains_key(&(f, c)) {
            return Err(MeshError::Hierarchy(
                "these meshes are already linked".into(),
            ));
        }
        self.maps.insert((f, c), map);
        Ok(())
    }

    fn insert_view(&mut self, view: InstancedView) -> Result<()> {
        if self.views.iter().any(|v| v.id == view.id) {
            return Err(MeshError::Hierarchy(format!(
                "view {:?} already present",
                view.id
            )));
        }
        if view.id.kind() != EntityKind::MeshInstance {
            return Err(MeshError::Hierarchy(
                "view id must be a MeshInstance id".into(),
            ));
        }
        let mut ids: Vec<EntityId> = view.instances.iter().map(|i| i.id).collect();
        ids.sort_unstable();
        if ids.windows(2).any(|w| w[0] == w[1])
            || ids.iter().any(|i| i.kind() != EntityKind::MeshInstance)
        {
            return Err(MeshError::Hierarchy(
                "instance ids must be unique MeshInstance ids".into(),
            ));
        }
        if let Some(i) = view
            .instances
            .iter()
            .find(|i| !self.meshes.contains_key(&i.prototype))
        {
            return Err(MeshError::UnknownMesh(i.prototype));
        }
        let at = self.views.partition_point(|v| v.id < view.id);
        self.views.insert(at, view);
        Ok(())
    }
}
