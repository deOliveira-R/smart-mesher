//! Typed selections and the selector algebra.
//!
//! Everything the model layer stores is a [`Selection`]: a sorted, duplicate-free set of ids of
//! one entity kind, tagged with a [`Lifetime`]. A color is a volume (or face) selection with an
//! exclusivity invariant; a boundary kind is a face selection that must cover the domain
//! boundary; a decomposition piece is a volume selection that must tile the domain. The
//! invariants live in [`crate::Model`] and [`crate::validate`], the algebra lives here.
//!
//! A [`Selector`] is an unevaluated expression (STK-style selector algebra). It is resolved
//! against a model and a topology, so it can be stored in an op and re-evaluated after the
//! topology changes.

use crate::boundary::KindFilter;
use crate::error::ModelError;
use crate::model::Model;
use crate::topology::TopologyView;
use sm_core::{EntityId, EntityKind};
use std::marker::PhantomData;

/// Marker for the entity kind a [`Selection`] holds. Implemented by [`Volume`] and [`Face`].
pub trait SelKind: Copy + Clone + core::fmt::Debug + PartialEq + Eq + 'static {
    /// Topology kind of geometry-bound members.
    const KIND: EntityKind;
    /// Mesh kind of mesh-bound members (their classification target is `KIND`).
    const MESH_KIND: EntityKind;
    /// Resolve a selector of this kind. Implemented in the model module.
    fn resolve(model: &Model, sel: &Selector<Self>, topo: &dyn TopologyView) -> Result<Selection<Self>, ModelError>;
}

/// Marker: volume selections.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Volume;
/// Marker: face selections.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Face;

/// What a selection is bound to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Lifetime {
    /// Members are topology entities. This is the source of truth; survives remeshing.
    GeometryBound,
    /// Members are entities of one mesh, obtained by lowering a geometry-bound selection through
    /// classification. Derived, never authoritative; dies with the mesh.
    MeshBound {
        /// The mesh the members belong to.
        mesh: EntityId,
    },
}

/// A sorted, duplicate-free set of entities of kind `K`.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(bound = ""))]
pub struct Selection<K: SelKind> {
    ids: Vec<EntityId>,
    lifetime: Lifetime,
    #[cfg_attr(feature = "serde", serde(skip))]
    _k: PhantomData<K>,
}

impl<K: SelKind> Default for Selection<K> {
    fn default() -> Self {
        Self::empty()
    }
}

impl<K: SelKind> Selection<K> {
    /// The empty geometry-bound selection.
    pub fn empty() -> Self {
        Self { ids: Vec::new(), lifetime: Lifetime::GeometryBound, _k: PhantomData }
    }

    /// Build a geometry-bound selection. Sorts, dedups, and rejects ids of the wrong kind.
    pub fn geometry(ids: impl IntoIterator<Item = EntityId>) -> Result<Self, ModelError> {
        Self::build(Lifetime::GeometryBound, ids, K::KIND)
    }

    /// Build a mesh-bound selection over `mesh`.
    pub fn mesh(mesh: EntityId, ids: impl IntoIterator<Item = EntityId>) -> Result<Self, ModelError> {
        Self::build(Lifetime::MeshBound { mesh }, ids, K::MESH_KIND)
    }

    fn build(lifetime: Lifetime, ids: impl IntoIterator<Item = EntityId>, kind: EntityKind) -> Result<Self, ModelError> {
        let mut v: Vec<EntityId> = ids.into_iter().collect();
        if let Some(bad) = v.iter().find(|i| i.kind() != kind) {
            return Err(ModelError::KindMismatch { expected: kind, found: *bad });
        }
        v.sort_unstable();
        v.dedup();
        Ok(Self { ids: v, lifetime, _k: PhantomData })
    }

    /// Members, ascending.
    pub fn ids(&self) -> &[EntityId] {
        &self.ids
    }
    /// Iterate members, ascending.
    pub fn iter(&self) -> impl Iterator<Item = EntityId> + '_ {
        self.ids.iter().copied()
    }
    /// Number of members.
    pub fn len(&self) -> usize {
        self.ids.len()
    }
    /// True if there are no members.
    pub fn is_empty(&self) -> bool {
        self.ids.is_empty()
    }
    /// Membership test, `O(log n)`.
    pub fn contains(&self, id: EntityId) -> bool {
        self.ids.binary_search(&id).is_ok()
    }
    /// The lifetime tag.
    pub fn lifetime(&self) -> Lifetime {
        self.lifetime
    }

    fn check_same(&self, o: &Self) -> Result<(), ModelError> {
        if self.lifetime == o.lifetime { Ok(()) } else { Err(ModelError::LifetimeMismatch) }
    }

    /// Set union.
    pub fn union(&self, o: &Self) -> Result<Self, ModelError> {
        self.check_same(o)?;
        Ok(self.merge(o, true, true, true))
    }
    /// Set intersection.
    pub fn intersection(&self, o: &Self) -> Result<Self, ModelError> {
        self.check_same(o)?;
        Ok(self.merge(o, false, false, true))
    }
    /// Set difference `self \ o`.
    pub fn difference(&self, o: &Self) -> Result<Self, ModelError> {
        self.check_same(o)?;
        Ok(self.merge(o, true, false, false))
    }
    /// Complement of `self` inside `universe`.
    pub fn complement(&self, universe: &Self) -> Result<Self, ModelError> {
        universe.difference(self)
    }

    /// Sorted merge keeping elements only in self / only in other / in both, as asked.
    fn merge(&self, o: &Self, only_a: bool, only_b: bool, both: bool) -> Self {
        let (a, b) = (&self.ids, &o.ids);
        let mut out = Vec::with_capacity(a.len() + b.len());
        let (mut i, mut j) = (0, 0);
        while i < a.len() || j < b.len() {
            let ord = match (a.get(i), b.get(j)) {
                (Some(x), Some(y)) => x.cmp(y),
                (Some(_), None) => core::cmp::Ordering::Less,
                (None, _) => core::cmp::Ordering::Greater,
            };
            match ord {
                core::cmp::Ordering::Less => {
                    if only_a {
                        out.push(a[i]);
                    }
                    i += 1;
                }
                core::cmp::Ordering::Greater => {
                    if only_b {
                        out.push(b[j]);
                    }
                    j += 1;
                }
                core::cmp::Ordering::Equal => {
                    if both {
                        out.push(a[i]);
                    }
                    i += 1;
                    j += 1;
                }
            }
        }
        Self { ids: out, lifetime: self.lifetime, _k: PhantomData }
    }

    /// Lower a geometry-bound selection onto `mesh`. `classify` maps a topology entity to the mesh
    /// entities classified on it (the mesh layer's classification table).
    pub fn lower(&self, mesh: EntityId, mut classify: impl FnMut(EntityId) -> Vec<EntityId>) -> Result<Self, ModelError> {
        if self.lifetime != Lifetime::GeometryBound {
            return Err(ModelError::LifetimeMismatch);
        }
        Self::mesh(mesh, self.ids.iter().flat_map(|i| classify(*i)))
    }
}

/// An unevaluated selection expression.
///
/// Variants that do not make sense for `K` (for example [`Selector::Kind`] on volumes) fail to
/// resolve with [`ModelError::InvalidSelector`].
#[derive(Clone, Debug, PartialEq)]
pub enum Selector<K: SelKind> {
    /// Everything of kind `K` in the topology.
    All,
    /// Everything of kind `K` inside the model domain (the fundamental domain after a cut).
    InDomain,
    /// Literal ids.
    Ids(Selection<K>),
    /// A named group (by id), restricted to members of kind `K`.
    Named(EntityId),
    /// A color: volume color for volumes, boundary color for faces.
    Color(EntityId),
    /// Faces with a given boundary kind.
    Kind(KindFilter),
    /// Faces on the boundary of the domain with no boundary kind yet.
    Unassigned,
    /// Faces on the boundary of the domain (exactly one side inside).
    DomainBoundary,
    /// Volumes of a decomposition piece.
    Piece(EntityId),
    /// Complement inside `All`.
    Not(Box<Selector<K>>),
    /// Intersection.
    And(Vec<Selector<K>>),
    /// Union.
    Or(Vec<Selector<K>>),
    /// `a \ b`.
    Diff(Box<Selector<K>>, Box<Selector<K>>),
    /// Faces bounding the given volumes (valid for faces).
    FacesOf(Box<Selector<Volume>>),
    /// Volumes on either side of the given faces (valid for volumes).
    VolumesOf(Box<Selector<Face>>),
}

impl SelKind for Volume {
    const KIND: EntityKind = EntityKind::TopoVolume;
    const MESH_KIND: EntityKind = EntityKind::MeshCell;
    fn resolve(model: &Model, sel: &Selector<Self>, topo: &dyn TopologyView) -> Result<Selection<Self>, ModelError> {
        model.resolve_volumes(sel, topo)
    }
}

impl SelKind for Face {
    const KIND: EntityKind = EntityKind::TopoFace;
    const MESH_KIND: EntityKind = EntityKind::MeshFace;
    fn resolve(model: &Model, sel: &Selector<Self>, topo: &dyn TopologyView) -> Result<Selection<Self>, ModelError> {
        model.resolve_faces(sel, topo)
    }
}

impl<K: SelKind> Selector<K> {
    /// Resolve against `model` and `topo`.
    pub fn resolve(&self, model: &Model, topo: &dyn TopologyView) -> Result<Selection<K>, ModelError> {
        K::resolve(model, self, topo)
    }
}
