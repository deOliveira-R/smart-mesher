//! Contract for the smart-mesh mesh database (layer "Meshes", `docs/design/ir.md` §5).
//!
//! Design in one paragraph: a mesh is a set of **typed columnar tables** (MOAB/STK style), one per
//! dimension, joined by CSR relations. Topology is **face-first**: every cell stores oriented
//! cell to face references and every face stores `(left, right)` under one declared
//! [orientation rule](orient). Every entity carries a [classification](classify) onto a
//! model/partition [`sm_core::EntityId`]. Extra data lives in typed [tag columns](tag) and
//! [sets](tag::EntitySet) addressed by a [selector algebra](mod@select). A [schema] declares
//! what an instance must contain. Consumers read through the object-safe traits in [`query`];
//! producers use [`build`]; [`hierarchy`] covers fine/coarse levels and instancing; [`mod@reference`]
//! is the in-memory implementor; [`validate`] checks any implementor.
//!
//! See `DESIGN.md` in this crate for the model, tradeoffs and requirement mapping.

pub mod build;
pub mod classify;
pub mod error;
pub mod hierarchy;
pub mod index;
pub mod orient;
pub mod query;
pub mod reference;
pub mod rules;
pub mod schema;
pub mod select;
pub mod shape;
pub mod table;
pub mod tag;
pub mod validate;

pub use build::{MeshBuilder, MeshEdit};
pub use classify::{Classification, Param};
pub use error::{MeshError, MeshResult};
pub use index::{CellIx, Dim, EdgeIx, FaceIx, Sign, SignedFace, VertexIx};
pub use query::{MeshClassification, MeshGeometry, MeshRead, MeshTags, MeshTopology};
pub use select::Selector;
pub use shape::{Basis, CellShape, CellSpec, FaceShape};
