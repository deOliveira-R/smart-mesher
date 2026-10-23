//! The model aggregate: colors, groups, boundary kinds, domain, symmetry, instancing, pieces.
//!
//! The model never owns topology. It stores *selections over topology ids* and the invariants
//! between them; operations that need geometry take a `&dyn TopologyView`.

use crate::action::Generator;
use crate::boundary::{BoundaryKind, DeckPairing, DeckSide, InterfaceIntent, KindSet};
use crate::decomposition::Decomposition;
use crate::error::ModelError;
use crate::hash::Fnv64;
use crate::instancing::Instancing;
use crate::selection::{Face, Lifetime, SelKind, Selection, Selector, Volume};
use crate::symmetry::SymmetrySpec;
use crate::topology::TopologyView;
use sm_core::{EntityId, EntityKind};
use std::collections::{BTreeMap, BTreeSet};

/// Geometric tolerances used by symmetry verification and the validator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    /// Absolute length tolerance.
    pub length: f64,
    /// Tolerance on `1 - |cos|` style angle checks (dimensionless).
    pub angle: f64,
    /// Relative tolerance for areas, volumes and moments.
    pub relative: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { length: 1e-6, angle: 1e-6, relative: 1e-6 }
    }
}

/// A named selection (a color, or a group of one kind).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(bound = ""))]
pub struct Named<K: SelKind> {
    /// Id (`Color` kind).
    pub id: EntityId,
    /// Unique name (abstract label such as "material 1"; no physics).
    pub name: String,
    /// The members.
    pub selection: Selection<K>,
}

/// Members of a named group (block, assembly, set).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GroupMembers {
    /// A set of volumes.
    Volumes(Selection<Volume>),
    /// A set of faces.
    Faces(Selection<Face>),
    /// A set of other groups (assemblies of blocks).
    Groups(Vec<EntityId>),
}

/// A named grouping. Groups are labels; they may overlap freely.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Group {
    /// Id (`Group` kind).
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// Members.
    pub members: GroupMembers,
}

/// Which side(s) of a domain a face touches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DomainSides {
    /// The in-domain volume that the face's outward normal leaves (the "near" side).
    pub near: EntityId,
    /// The other in-domain volume, if the face is interior to the domain.
    pub far: Option<EntityId>,
    /// True if the stored normal points *into* `near` (stored left was outside the domain).
    pub swapped: bool,
}

/// The model-semantics layer: coloring, boundary kinds, symmetry, instancing, decomposition.
#[derive(Clone, Debug, Default)]
pub struct Model {
    next: BTreeMap<EntityKind, u64>,
    volume_color_names: BTreeSet<String>,
    face_color_names: BTreeSet<String>,
    group_names: BTreeSet<String>,
    pub(crate) volume_colors: BTreeMap<EntityId, Named<Volume>>,
    pub(crate) face_colors: BTreeMap<EntityId, Named<Face>>,
    pub(crate) groups: BTreeMap<EntityId, Group>,
    vcolor_of: BTreeMap<EntityId, EntityId>,
    fcolor_of: BTreeMap<EntityId, EntityId>,
    pub(crate) kind_sets: BTreeMap<EntityId, KindSet>,
    kind_of: BTreeMap<EntityId, EntityId>,
    pub(crate) pairings: BTreeMap<EntityId, DeckPairing>,
    pub(crate) domain: Option<Selection<Volume>>,
    pub(crate) symmetry: Option<SymmetrySpec>,
    pub(crate) instancing: Instancing,
    pub(crate) decomposition: Option<Decomposition>,
}

impl Model {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    pub(crate) fn mint(&mut self, kind: EntityKind) -> Result<EntityId, ModelError> {
        let n = self.next.entry(kind).or_insert(0);
        *n += 1;
        EntityId::new(kind, *n).ok_or(ModelError::SerialOverflow(kind))
    }

    // ---------------- coloring ----------------

    fn require_geometry<K: SelKind>(s: &Selection<K>) -> Result<(), ModelError> {
        if s.lifetime() == Lifetime::GeometryBound { Ok(()) } else { Err(ModelError::LifetimeMismatch) }
    }

    /// Define a volume color. Colors partition the volumes: a volume may have at most one.
    /// (Coverage of *all* volumes is checked by the validator, so colors can be added one by one.)
    pub fn define_volume_color(&mut self, name: &str, members: Selection<Volume>) -> Result<EntityId, ModelError> {
        Self::require_geometry(&members)?;
        if self.volume_color_names.contains(name) {
            return Err(ModelError::DuplicateName(name.to_owned()));
        }
        let id = self.peek_id(EntityKind::Color)?;
        if let Some(m) = members.iter().find(|m| self.vcolor_of.contains_key(m)) {
            return Err(ModelError::Conflict { member: m, existing: self.vcolor_of[&m], wanted: id });
        }
        let id = self.mint(EntityKind::Color)?;
        self.volume_color_names.insert(name.to_owned());
        for m in members.iter() {
            self.vcolor_of.insert(m, id);
        }
        self.volume_colors.insert(id, Named { id, name: name.to_owned(), selection: members });
        Ok(id)
    }

    /// Define a boundary color on faces. A face has at most one boundary color.
    pub fn define_face_color(&mut self, name: &str, faces: Selection<Face>) -> Result<EntityId, ModelError> {
        Self::require_geometry(&faces)?;
        if self.face_color_names.contains(name) {
            return Err(ModelError::DuplicateName(name.to_owned()));
        }
        let id = self.peek_id(EntityKind::Color)?;
        if let Some(m) = faces.iter().find(|m| self.fcolor_of.contains_key(m)) {
            return Err(ModelError::Conflict { member: m, existing: self.fcolor_of[&m], wanted: id });
        }
        let id = self.mint(EntityKind::Color)?;
        self.face_color_names.insert(name.to_owned());
        for m in faces.iter() {
            self.fcolor_of.insert(m, id);
        }
        self.face_colors.insert(id, Named { id, name: name.to_owned(), selection: faces });
        Ok(id)
    }

    fn peek_id(&self, kind: EntityKind) -> Result<EntityId, ModelError> {
        EntityId::new(kind, self.next.get(&kind).copied().unwrap_or(0) + 1).ok_or(ModelError::SerialOverflow(kind))
    }

    /// Color of a volume.
    pub fn volume_color_of(&self, v: EntityId) -> Option<EntityId> {
        self.vcolor_of.get(&v).copied()
    }
    /// Boundary color of a face.
    pub fn face_color_of(&self, f: EntityId) -> Option<EntityId> {
        self.fcolor_of.get(&f).copied()
    }
    /// Volume colors, ascending by id.
    pub fn volume_colors(&self) -> impl Iterator<Item = &Named<Volume>> {
        self.volume_colors.values()
    }
    /// Boundary colors, ascending by id.
    pub fn face_colors(&self) -> impl Iterator<Item = &Named<Face>> {
        self.face_colors.values()
    }
    /// Look up a volume color by id.
    pub fn volume_color(&self, id: EntityId) -> Option<&Named<Volume>> {
        self.volume_colors.get(&id)
    }
    /// Look up a boundary color by id.
    pub fn face_color(&self, id: EntityId) -> Option<&Named<Face>> {
        self.face_colors.get(&id)
    }

    /// Define a named group. Members that are groups must already exist (so groups cannot form cycles).
    pub fn define_group(&mut self, name: &str, members: GroupMembers) -> Result<EntityId, ModelError> {
        match &members {
            GroupMembers::Volumes(s) => Self::require_geometry(s)?,
            GroupMembers::Faces(s) => Self::require_geometry(s)?,
            GroupMembers::Groups(g) => {
                if let Some(bad) = g.iter().find(|g| !self.groups.contains_key(g)) {
                    return Err(ModelError::UnknownEntity(*bad));
                }
            }
        }
        if self.group_names.contains(name) {
            return Err(ModelError::DuplicateName(name.to_owned()));
        }
        let id = self.mint(EntityKind::Group)?;
        self.group_names.insert(name.to_owned());
        let members = match members {
            GroupMembers::Groups(mut g) => {
                g.sort_unstable();
                g.dedup();
                GroupMembers::Groups(g)
            }
            m => m,
        };
        self.groups.insert(id, Group { id, name: name.to_owned(), members });
        Ok(id)
    }

    /// Groups, ascending by id.
    pub fn groups(&self) -> impl Iterator<Item = &Group> {
        self.groups.values()
    }

    /// A stable fingerprint of the coloring (volume and face colors). Unchanged by decomposition.
    pub fn coloring_fingerprint(&self) -> u64 {
        let mut h = Fnv64::new();
        for c in self.volume_colors.values() {
            h.id(c.id);
            h.str(&c.name);
            c.selection.iter().for_each(|i| h.id(i));
        }
        h.u64(u64::MAX);
        for c in self.face_colors.values() {
            h.id(c.id);
            h.str(&c.name);
            c.selection.iter().for_each(|i| h.id(i));
        }
        h.finish()
    }

    // ---------------- domain ----------------

    /// Restrict the model to a domain (the fundamental domain after a cut). `None` restores all volumes.
    pub fn set_domain(&mut self, domain: Option<Selection<Volume>>) -> Result<(), ModelError> {
        if let Some(d) = &domain {
            Self::require_geometry(d)?;
        }
        self.domain = domain;
        Ok(())
    }
    /// The explicit domain, if any.
    pub fn domain(&self) -> Option<&Selection<Volume>> {
        self.domain.as_ref()
    }
    /// Is `v` inside the domain?
    pub fn in_domain(&self, v: EntityId) -> bool {
        self.domain.as_ref().is_none_or(|d| d.contains(v))
    }
    /// Volumes inside the domain, ascending.
    pub fn domain_volumes(&self, topo: &dyn TopologyView) -> Vec<EntityId> {
        topo.volumes().into_iter().filter(|v| self.in_domain(*v)).collect()
    }

    /// Domain-relative sides of a face; `None` if no side is inside the domain or the face is unknown.
    pub fn domain_sides(&self, f: EntityId, topo: &dyn TopologyView) -> Option<DomainSides> {
        let s = topo.face_sides(f)?;
        let l = s.left.filter(|v| self.in_domain(*v));
        let r = s.right.filter(|v| self.in_domain(*v));
        match (l, r) {
            (Some(a), Some(b)) => Some(DomainSides { near: a, far: Some(b), swapped: false }),
            (Some(a), None) => Some(DomainSides { near: a, far: None, swapped: false }),
            (None, Some(b)) => Some(DomainSides { near: b, far: None, swapped: true }),
            (None, None) => None,
        }
    }

    /// Faces with exactly one side in the domain, ascending. Boundary-ness is relative to the domain.
    pub fn domain_boundary_faces(&self, topo: &dyn TopologyView) -> Vec<EntityId> {
        topo.faces().into_iter().filter(|f| self.domain_sides(*f, topo).is_some_and(|d| d.far.is_none())).collect()
    }

    /// Unit normal of a face pointing *out of* the domain (for a boundary face).
    pub fn outward_normal(&self, f: EntityId, topo: &dyn TopologyView) -> Option<sm_core::Vector3> {
        let n = topo.face_properties(f)?.normal;
        let s = self.domain_sides(f, topo)?;
        Some(if s.swapped { -n } else { n })
    }

    // ---------------- boundary kinds ----------------

    fn check_kind_free(&self, faces: &Selection<Face>, wanted: EntityId) -> Result<(), ModelError> {
        Self::require_geometry(faces)?;
        match faces.iter().find(|f| self.kind_of.contains_key(f)) {
            Some(m) => Err(ModelError::Conflict { member: m, existing: self.kind_of[&m], wanted }),
            None => Ok(()),
        }
    }

    fn insert_kind_set(&mut self, kind: BoundaryKind, faces: Selection<Face>) -> Result<EntityId, ModelError> {
        let id = self.mint(EntityKind::Boundary)?;
        for f in faces.iter() {
            self.kind_of.insert(f, id);
        }
        self.kind_sets.insert(id, KindSet { id, kind, faces });
        Ok(id)
    }

    /// Tag faces as a response boundary. The tag is opaque ("vacuum", "inlet 3"...).
    pub fn define_response(&mut self, tag: &str, faces: Selection<Face>) -> Result<EntityId, ModelError> {
        let wanted = self.peek_id(EntityKind::Boundary)?;
        self.check_kind_free(&faces, wanted)?;
        self.insert_kind_set(BoundaryKind::Response { tag: tag.to_owned() }, faces)
    }

    /// Tag faces as an interface (between colors, or declared non-conformal).
    pub fn define_interface(&mut self, intent: InterfaceIntent, faces: Selection<Face>) -> Result<EntityId, ModelError> {
        for c in intent.between.iter().flat_map(|(a, b)| [*a, *b]) {
            if !self.volume_colors.contains_key(&c) {
                return Err(ModelError::UnknownEntity(c));
            }
        }
        let wanted = self.peek_id(EntityKind::Boundary)?;
        self.check_kind_free(&faces, wanted)?;
        self.insert_kind_set(BoundaryKind::Interface(intent), faces)
    }

    /// Declare a deck pairing for already-existing faces (imported pre-cut geometry, or by
    /// [`Self::cut_keep_chamber`]). `pairs` are `(a, b)` with `b = g(a)`; a mirror takes `(f, f)`.
    /// Returns the pairing id. Geometric consistency is checked by the validator, not here.
    pub fn declare_deck(&mut self, generator: Generator, mut pairs: Vec<(EntityId, EntityId)>) -> Result<EntityId, ModelError> {
        generator.validate()?;
        pairs.sort_unstable();
        pairs.dedup();
        if pairs.is_empty() {
            return Err(ModelError::Invalid("deck pairing without faces".into()));
        }
        let mirror = generator.kind() == crate::action::DeckKind::Mirror;
        if mirror && pairs.iter().any(|(a, b)| a != b) {
            return Err(ModelError::Invalid("a mirror pairs each face with itself".into()));
        }
        let a_set = Selection::<Face>::geometry(pairs.iter().map(|p| p.0))?;
        let b_set = Selection::<Face>::geometry(pairs.iter().map(|p| p.1))?;
        if a_set.len() != pairs.len() || (!mirror && b_set.len() != pairs.len()) {
            return Err(ModelError::Invalid("deck pairs must be one-to-one".into()));
        }
        let wanted = self.peek_id(EntityKind::Boundary)?;
        self.check_kind_free(&a_set, wanted)?;
        if !mirror {
            self.check_kind_free(&b_set, wanted)?;
            if !a_set.intersection(&b_set)?.is_empty() {
                return Err(ModelError::Invalid("A and B face sets overlap".into()));
            }
        }
        let pairing = self.mint(EntityKind::Boundary)?;
        self.pairings.insert(pairing, DeckPairing { id: pairing, generator, pairs });
        if mirror {
            self.insert_kind_set(BoundaryKind::Deck { pairing, side: DeckSide::SelfPaired }, a_set)?;
        } else {
            self.insert_kind_set(BoundaryKind::Deck { pairing, side: DeckSide::A }, a_set)?;
            self.insert_kind_set(BoundaryKind::Deck { pairing, side: DeckSide::B }, b_set)?;
        }
        Ok(pairing)
    }

    /// Boundary kind of a face.
    pub fn kind_of_face(&self, f: EntityId) -> Option<&BoundaryKind> {
        self.kind_of.get(&f).and_then(|k| self.kind_sets.get(k)).map(|k| &k.kind)
    }
    /// The kind set that contains a face.
    pub fn kind_set_of_face(&self, f: EntityId) -> Option<EntityId> {
        self.kind_of.get(&f).copied()
    }
    /// Kind sets, ascending by id.
    pub fn kind_sets(&self) -> impl Iterator<Item = &KindSet> {
        self.kind_sets.values()
    }
    /// A deck pairing.
    pub fn pairing(&self, id: EntityId) -> Option<&DeckPairing> {
        self.pairings.get(&id)
    }
    /// Deck pairings, ascending by id.
    pub fn pairings(&self) -> impl Iterator<Item = &DeckPairing> {
        self.pairings.values()
    }

    // ---------------- other layers ----------------

    /// The declared symmetry, if any.
    pub fn symmetry(&self) -> Option<&SymmetrySpec> {
        self.symmetry.as_ref()
    }
    /// Instancing records.
    pub fn instancing(&self) -> &Instancing {
        &self.instancing
    }
    /// Mutable instancing records.
    pub fn instancing_mut(&mut self) -> &mut Instancing {
        &mut self.instancing
    }
    /// The current decomposition, if any.
    pub fn decomposition(&self) -> Option<&Decomposition> {
        self.decomposition.as_ref()
    }
    /// Mutable decomposition (split, composite, unsplit...). Coloring is not reachable from it.
    pub fn decomposition_mut(&mut self) -> Option<&mut Decomposition> {
        self.decomposition.as_mut()
    }
    /// Start a trivial decomposition: one piece per domain volume.
    pub fn decompose(&mut self, topo: &dyn TopologyView) -> Result<(), ModelError> {
        let vols = self.domain_volumes(topo);
        self.decomposition = Some(Decomposition::trivial(&vols, self.coloring_fingerprint())?);
        Ok(())
    }
    /// Drop the decomposition (reversal). Coloring is untouched.
    pub fn clear_decomposition(&mut self) -> Option<Decomposition> {
        self.decomposition.take()
    }
    /// Colors of the volumes a piece touches (derived, never stored on the piece), ascending.
    pub fn piece_colors(&self, piece: EntityId) -> Vec<EntityId> {
        let Some(d) = &self.decomposition else { return Vec::new() };
        let set: BTreeSet<EntityId> = d.volumes_of_piece(piece).into_iter().filter_map(|v| self.volume_color_of(v)).collect();
        set.into_iter().collect()
    }

    // ---------------- selector resolution ----------------

    fn fold<K: SelKind>(&self, parts: &[Selector<K>], topo: &dyn TopologyView, and: bool) -> Result<Selection<K>, ModelError> {
        let mut it = parts.iter();
        let Some(first) = it.next() else {
            return if and { Selector::<K>::All.resolve(self, topo) } else { Ok(Selection::empty()) };
        };
        let mut acc = first.resolve(self, topo)?;
        for p in it {
            let s = p.resolve(self, topo)?;
            acc = if and { acc.intersection(&s)? } else { acc.union(&s)? };
        }
        Ok(acc)
    }

    fn group_members(&self, g: EntityId, out_v: &mut Vec<EntityId>, out_f: &mut Vec<EntityId>) -> Result<(), ModelError> {
        let grp = self.groups.get(&g).ok_or(ModelError::UnknownEntity(g))?;
        match &grp.members {
            GroupMembers::Volumes(s) => out_v.extend(s.iter()),
            GroupMembers::Faces(s) => out_f.extend(s.iter()),
            GroupMembers::Groups(gs) => {
                for x in gs {
                    self.group_members(*x, out_v, out_f)?;
                }
            }
        }
        Ok(())
    }

    pub(crate) fn resolve_volumes(&self, sel: &Selector<Volume>, topo: &dyn TopologyView) -> Result<Selection<Volume>, ModelError> {
        use Selector as S;
        match sel {
            S::All => Selection::geometry(topo.volumes()),
            S::InDomain => Selection::geometry(self.domain_volumes(topo)),
            S::Ids(s) => Ok(s.clone()),
            S::Named(g) => {
                let (mut v, mut f) = (Vec::new(), Vec::new());
                self.group_members(*g, &mut v, &mut f)?;
                Selection::geometry(v)
            }
            S::Color(c) => self.volume_colors.get(c).map(|n| n.selection.clone()).ok_or(ModelError::UnknownEntity(*c)),
            S::Piece(p) => {
                let d = self.decomposition.as_ref().ok_or_else(|| ModelError::InvalidSelector("no decomposition".into()))?;
                Selection::geometry(d.volumes_of_piece(*p))
            }
            S::Not(a) => Selector::<Volume>::All.resolve(self, topo)?.difference(&a.resolve(self, topo)?),
            S::And(v) => self.fold(v, topo, true),
            S::Or(v) => self.fold(v, topo, false),
            S::Diff(a, b) => a.resolve(self, topo)?.difference(&b.resolve(self, topo)?),
            S::VolumesOf(faces) => {
                let fs = faces.resolve(self, topo)?;
                let mut out = Vec::new();
                for f in fs.iter() {
                    let s = topo.face_sides(f).ok_or(ModelError::UnknownEntity(f))?;
                    out.extend(s.left);
                    out.extend(s.right);
                }
                Selection::geometry(out)
            }
            S::Kind(_) | S::Unassigned | S::DomainBoundary | S::FacesOf(_) => {
                Err(ModelError::InvalidSelector("selector does not produce volumes".into()))
            }
        }
    }

    pub(crate) fn resolve_faces(&self, sel: &Selector<Face>, topo: &dyn TopologyView) -> Result<Selection<Face>, ModelError> {
        use Selector as S;
        match sel {
            S::All => Selection::geometry(topo.faces()),
            S::InDomain => Selection::geometry(topo.faces().into_iter().filter(|f| self.domain_sides(*f, topo).is_some())),
            S::Ids(s) => Ok(s.clone()),
            S::Named(g) => {
                let (mut v, mut f) = (Vec::new(), Vec::new());
                self.group_members(*g, &mut v, &mut f)?;
                Selection::geometry(f)
            }
            S::Color(c) => self.face_colors.get(c).map(|n| n.selection.clone()).ok_or(ModelError::UnknownEntity(*c)),
            S::Kind(filter) => {
                let pk = |p: EntityId| self.pairings.get(&p).map(DeckPairing::kind);
                let mut out = Vec::new();
                for ks in self.kind_sets.values().filter(|k| filter.matches(&k.kind, pk)) {
                    out.extend(ks.faces.iter());
                }
                Selection::geometry(out)
            }
            S::Unassigned => Selection::geometry(self.domain_boundary_faces(topo).into_iter().filter(|f| !self.kind_of.contains_key(f))),
            S::DomainBoundary => Selection::geometry(self.domain_boundary_faces(topo)),
            S::Piece(p) => {
                let d = self.decomposition.as_ref().ok_or_else(|| ModelError::InvalidSelector("no decomposition".into()))?;
                Selection::geometry(d.faces_of_piece(*p, topo).into_iter().filter_map(|pf| match pf {
                    crate::decomposition::PieceFace::Model(f) => Some(f),
                    _ => None,
                }))
            }
            S::Not(a) => Selector::<Face>::All.resolve(self, topo)?.difference(&a.resolve(self, topo)?),
            S::And(v) => self.fold(v, topo, true),
            S::Or(v) => self.fold(v, topo, false),
            S::Diff(a, b) => a.resolve(self, topo)?.difference(&b.resolve(self, topo)?),
            S::FacesOf(vols) => {
                let vs = vols.resolve(self, topo)?;
                Selection::geometry(vs.iter().flat_map(|v| topo.volume_faces(v)))
            }
            S::VolumesOf(_) => Err(ModelError::InvalidSelector("selector does not produce faces".into())),
        }
    }
}
