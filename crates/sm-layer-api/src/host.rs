//! Object-safe hosts: a topology store and a mesh store behind a trait object.
//!
//! The contract crates cannot name an implementation crate, so the `model` layer holds its
//! topology as `Arc<dyn TopoHost>` and each `meshes` entry holds `Arc<dyn MeshHost>`. The
//! production distribution passes `sm_topo::TopoStore` and `sm_mesh::Mesh`; tests pass the
//! reference `CellComplex` and `ReferenceMesh`. Both traits have a blanket impl, so any
//! `Clone` store qualifies without extra code.

use sm_mesh_api::{MeshEdit, MeshRead};
use sm_topo_api::{TopologyEdit, TopologyView};

/// A topology store usable as a layer's content: editable, cloneable behind a trait object.
///
/// Implemented for every `T: TopologyEdit + Clone + Send + Sync + 'static`. A clone must be a
/// deep, independent copy (copy-on-write sharing is the implementor's business).
pub trait TopoHost: TopologyEdit + Send + Sync + 'static {
    /// A deep copy behind a new box (what a `State` clone needs on first write).
    fn clone_host(&self) -> Box<dyn TopoHost>;
    /// The read-only view of this store.
    fn as_view(&self) -> &dyn TopologyView;
}

impl<T: TopologyEdit + Clone + Send + Sync + 'static> TopoHost for T {
    fn clone_host(&self) -> Box<dyn TopoHost> {
        Box::new(self.clone())
    }
    fn as_view(&self) -> &dyn TopologyView {
        self
    }
}

/// A mesh store usable as a `meshes` entry: editable, cloneable behind a trait object.
///
/// Implemented for every `T: MeshEdit + Clone + Send + Sync + 'static`.
pub trait MeshHost: MeshEdit + Send + Sync + 'static {
    /// A deep copy behind a new box.
    fn clone_host(&self) -> Box<dyn MeshHost>;
    /// The read-only view of this mesh.
    fn as_read(&self) -> &dyn MeshRead;
}

impl<T: MeshEdit + Clone + Send + Sync + 'static> MeshHost for T {
    fn clone_host(&self) -> Box<dyn MeshHost> {
        Box::new(self.clone())
    }
    fn as_read(&self) -> &dyn MeshRead {
        self
    }
}
