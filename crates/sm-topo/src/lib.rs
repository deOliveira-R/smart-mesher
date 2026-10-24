//! Production cellular non-manifold B-rep store implementing `sm-topo-api`.
//!
//! [`TopoStore`] implements [`sm_topo_api::TopologyView`] and [`sm_topo_api::TopologyEdit`] with
//! exactly the observable behaviour of the reference [`sm_topo_api::CellComplex`] (ids, lineage,
//! validation results, canonical hashes), on a structure-of-arrays store with CSR incidence
//! rows edited in place and journaled as deltas. See `DESIGN.md`.

mod check;
mod hash;
mod helpers;
mod lifecycle;
mod norm;
mod ops;
mod radial;
mod rows;
mod store;

pub use store::{Counters, TopoStore};
