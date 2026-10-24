//! Stand-ins for `sm_mesh_api::reference::*` so the contract tests can run against `sm-mesh`.
#![allow(dead_code)]

use super::replay::{Mode, replay};
use sm_mesh_api::reference as r;
use sm_mesh_api::{MeshRead, Result, SerialAlloc};

#[allow(unused_imports)]
pub use sm_mesh::Catalog as ReferenceCatalog;
pub use sm_mesh::Mesh as ReferenceMesh;
pub use sm_mesh::MeshBuilder as ReferenceBuilder;

/// `hex_block` of the reference, re-created in the `sm-mesh` store (same ids).
pub fn hex_block(n: [u32; 3], h: [f64; 3], alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
    let src = r::hex_block(n, h, &mut alloc.clone())?;
    Ok(replay(&src as &dyn MeshRead, ReferenceBuilder::new(), Mode::Mesher, alloc)?.0.mesh)
}

/// `hex_block_with_edges` of the reference, re-created in the `sm-mesh` store (same ids).
pub fn hex_block_with_edges(n: [u32; 3], h: [f64; 3], alloc: &mut SerialAlloc) -> Result<ReferenceMesh> {
    let src = r::hex_block_with_edges(n, h, &mut alloc.clone())?;
    let (mut f, _) = replay(&src as &dyn MeshRead, ReferenceBuilder::new(), Mode::Mesher, alloc)?;
    // Edge tags are not buildable; the reference blocks carry none.
    super::replay::copy_edge_tags(&src, &mut f.mesh);
    Ok(f.mesh)
}
