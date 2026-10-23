//! Model-semantics and decomposition contract (docs/design/ir.md §5), group-theory first.
//!
//! One abstraction carries the whole layer: a **fundamental piece**, a **group action** on it, and
//! **boundary identifications**. Symmetry (mirror/rotation/translation deck groups) and instancing
//! (prototypes, lattices) are both group actions ([`group::GroupAction`]); coloring is a function on
//! the quotient and is pulled back to every copy; unfold and flatten are the same functor.
//!
//! Modules:
//! - [`coloring`]: colors on volumes and faces, named groups (labels only, no physics).
//! - [`boundary`]: deck / response / interface kind of every boundary face.
//! - [`group`], [`symmetry`], [`detect`]: deck groups, cut-and-keep-chamber, unfold, detection.
//! - [`instancing`]: prototypes, instances, nestable cartesian/hex lattices, flatten with paths.
//! - [`decomposition`]: virtual, reversible pieces; never changes coloring.
//! - [`topo`]: the narrow [`topo::TopologyView`] / [`topo::GeometryView`] traits (input to sm-topo-api).
//! - [`validate`]: semantic invariants. [`mock`]: reference in-memory topology for tests.
//!
//! Everything is deterministic: ordered maps only, canonical element and slot orders, no RNG, no
//! threads.

pub mod boundary;
pub mod coloring;
pub mod decomposition;
pub mod detect;
pub mod error;
pub mod group;
pub mod ids;
pub mod instancing;
pub mod mock;
pub mod model;
pub mod symmetry;
pub mod topo;
pub mod validate;
pub mod xf;

pub use error::{ModelError, Result};
pub use model::Model;
