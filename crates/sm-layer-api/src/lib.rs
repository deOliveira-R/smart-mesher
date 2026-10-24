//! `sm-layer-api`: the shared `model` and `meshes` state layers (W2 ACR-3).
//!
//! Five W2 crates read or write these two layers (model, mesher, fem, tess, server), so they
//! live in a contract crate that depends on no implementation crate: stores are trait objects
//! ([`TopoHost`], [`MeshHost`]).
//!
//! | Item | What |
//! |---|---|
//! | [`ModelLayer`] | topology host + [`Model`](sm_model_api::Model) tables + operation counter |
//! | [`topo_txn`] + [`commit_topology`] | the one way an op edits topology: run, then adopt ids, roles, retirements, lineage |
//! | [`model_txn`] | model-table edits (colors, groups, blocks): atomic, adopts minted ids |
//! | [`MeshLayer`], [`MeshEntry`] | meshes by `Mesh` id with a quantized digest; [`adopt_minted`] |
//! | [`CoreLayers`] | the plugin that registers both layers (once) with a topology-host factory |
//!
//! Id authority: the topology store mints topology cell ids, [`Model`](sm_model_api::Model)
//! mints color/group/boundary/... ids and the mesh builder mints mesh ids; the entity table
//! adopts them in ascending `(kind, serial)` order. Curve and surface handles are not entities
//! and are never adopted.
//!
//! Determinism: every map is a `BTreeMap`; ids are adopted in ascending order; digests use the
//! W0c Q2 quantization, so state hashes are platform independent.

mod plugin;
mod error;
mod host;
mod mesh;
mod model;

pub use plugin::CoreLayers;
pub use error::{LayerError, LayerResultExt};
pub use host::{MeshHost, TopoHost};
pub use mesh::{MeshEntry, MeshLayer, adopt_minted};
pub use model::{ModelLayer, TopoTxn, commit_topology, model_txn, topo_txn};
