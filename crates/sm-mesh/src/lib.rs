//! High-performance mesh store implementing `sm-mesh-api`: flat CSR face-first columns, parallel
//! `finish`, adjacency tables and zero-copy column views. See `DESIGN.md`.

#![forbid(unsafe_code)]

mod adjacency;
mod builder;
mod bulk;
mod catalog;
mod class;
mod edges;
mod edit;
mod export;
mod finish;
pub mod generate;
mod store;
mod util;

pub use adjacency::Csr;
pub use builder::{MeshBuilder, Validation};
pub use store::Mesh;
pub use catalog::Catalog;
pub use export::{RawColumns, as_bytes};
