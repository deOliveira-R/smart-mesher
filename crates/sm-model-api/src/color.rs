//! Coloring tables: abstract colors on volumes and on faces, plus named groups.
//!
//! Colors are labels only. The mesher colors; the consuming program resolves colors into
//! materials and boundary conditions. Volume coloring is an exact partition (every volume has
//! exactly one color, no mixed-material cells in P0-P1); face coloring is sparse. Every color
//! carries a user-chosen numeric `label` (a Cubit block or sideset id, an MCNP material number),
//! unique per domain.

use crate::error::{ModelError, Result, expect_kind};
use crate::model::Model;
use sm_core::{EntityId, EntityKind};
use sm_topo_api::{Dim, TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

/// A face key: `TopoFace` or an overlay `PartitionFace`.
pub(crate) fn expect_face_kind(f: EntityId) -> Result<()> {
    match f.kind() {
        EntityKind::TopoFace | EntityKind::PartitionFace => Ok(()),
        found => Err(ModelError::WrongKind { id: f, expected: EntityKind::TopoFace, found }),
    }
}

/// Own color, else the color of the lowest non-live key whose `live_cover` contains `id`.
fn live_color(table: &BTreeMap<EntityId, EntityId>, id: EntityId, t: &dyn TopologyView) -> Option<EntityId> {
    if let Some(c) = table.get(&id) {
        return Some(*c);
    }
    table.iter().find(|(k, _)| **k != id && !t.is_alive(**k) && t.live_cover(**k).contains(&id)).map(|(_, c)| *c)
}

/// Inverse of the hidden-host covers of a coloring table: live cell -> inherited color (lowest
/// host id wins). Own colors are not included.
pub(crate) fn inherited_colors(table: &BTreeMap<EntityId, EntityId>, t: &dyn TopologyView) -> BTreeMap<EntityId, EntityId> {
    let mut out = BTreeMap::new();
    for (k, c) in table {
        if t.is_alive(*k) {
            continue;
        }
        for x in t.live_cover(*k) {
            out.entry(x).or_insert(*c);
        }
    }
    out
}

/// What a color may be attached to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum Domain {
    /// Colors volumes (an abstract material).
    Volume,
    /// Colors faces (an abstract boundary or interface label).
    Face,
}

/// Row of the color table.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct ColorRow {
    /// The `EntityKind::Color` id.
    pub id: EntityId,
    /// User-chosen numeric label, unique within the domain.
    pub label: u32,
    /// Unique, user-visible name.
    pub name: String,
    /// What the color may label.
    pub domain: Domain,
}

/// How [`Model::group_edit`] changes a group's members (Cubit `group ... add / remove / equals /
/// xor`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum GroupEdit {
    /// Add the entities.
    Add,
    /// Remove the entities (absent ones are ignored).
    Remove,
    /// Replace the members by the entities.
    Equals,
    /// Toggle membership of each entity.
    Xor,
}

/// Whether `id` may be a group member. The model is the single group store (W0f): topology
/// cells of every rank, decomposition entities, and the model-minted kinds; never geometry
/// handles (`Curve`, `Surface`, not op-addressable) or mesh entities (`MeshSet` holds those).
pub fn group_member_kind_ok(id: EntityId) -> bool {
    use EntityKind::*;
    matches!(
        id.kind(),
        TopoVertex | TopoEdge | TopoFace | TopoVolume | TopoBody | PartitionCell | PartitionFace | PartitionEdge | PartitionVertex | Color | Group | Boundary | Instance | Prototype | Lattice | Symmetry
    )
}

/// Row of the group table: a named set of entities of any kind, expanded recursively.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct GroupRow {
    /// The `EntityKind::Group` id.
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// Members: topology cells of any rank, decomposition entities (`Partition*`), colors,
    /// nested groups and other model rows (see [`group_member_kind_ok`]).
    pub members: BTreeSet<EntityId>,
}

impl Model {
    /// Add a color with a numeric `label` (unique per domain) and a unique `name`.
    pub fn add_color(&mut self, label: u32, name: &str, d: Domain) -> Result<EntityId> {
        if self.colors.values().any(|c| c.domain == d && c.label == label) {
            return Err(ModelError::LabelTaken { label });
        }
        if self.colors.values().any(|c| c.name == name) {
            return Err(ModelError::NameTaken(name.to_owned()));
        }
        let id = self.ids.alloc(EntityKind::Color)?;
        self.colors.insert(id, ColorRow { id, label, name: name.to_owned(), domain: d });
        Ok(id)
    }

    fn check_color(&self, color: EntityId, domain: Domain) -> Result<()> {
        expect_kind(color, EntityKind::Color)?;
        match self.colors.get(&color) {
            None => Err(ModelError::Unknown(color)),
            Some(row) if row.domain != domain => Err(ModelError::ColorDomain(color)),
            Some(_) => Ok(()),
        }
    }

    /// Color `vols` with the volume color `c`, replacing any previous color (the partition is
    /// kept: a volume has one color).
    pub fn color_volumes(&mut self, c: EntityId, vols: &[EntityId]) -> Result<()> {
        self.check_color(c, Domain::Volume)?;
        for v in vols {
            expect_kind(*v, EntityKind::TopoVolume)?;
        }
        for v in vols {
            self.volume_color.insert(*v, c);
        }
        Ok(())
    }

    /// Color `faces` with the face color `c`, replacing any previous color. A face is a
    /// `TopoFace` or an overlay `PartitionFace` (a cut or composite face of a decomposition); an
    /// own color on a live overlay face overrides the color it inherits from a hidden host (see
    /// [`Self::live_face_color`]).
    pub fn color_faces(&mut self, c: EntityId, faces: &[EntityId]) -> Result<()> {
        self.check_color(c, Domain::Face)?;
        for f in faces {
            expect_face_kind(*f)?;
        }
        for f in faces {
            self.face_color.insert(*f, c);
        }
        Ok(())
    }

    /// Remove the color of `faces` (`TopoFace` or `PartitionFace` keys; others are ignored).
    pub fn uncolor_faces(&mut self, faces: &[EntityId]) {
        for f in faces {
            self.face_color.remove(f);
        }
    }

    /// Remove a color. Fails while volumes still use it; faces that use it become uncolored.
    pub fn remove_color(&mut self, c: EntityId) -> Result<()> {
        if !self.colors.contains_key(&c) {
            return Err(ModelError::Unknown(c));
        }
        if self.volume_color.values().any(|x| *x == c) {
            return Err(ModelError::ColorInUse(c));
        }
        self.face_color.retain(|_, x| *x != c);
        self.colors.remove(&c);
        Ok(())
    }

    /// The color row for `id`.
    pub fn color(&self, id: EntityId) -> Option<&ColorRow> {
        self.colors.get(&id)
    }

    /// All colors, ascending by id.
    pub fn colors(&self) -> impl Iterator<Item = &ColorRow> {
        self.colors.values()
    }

    /// The color with numeric `label` in domain `d`.
    pub fn color_by_label(&self, d: Domain, label: u32) -> Option<EntityId> {
        self.colors.values().find(|c| c.domain == d && c.label == label).map(|c| c.id)
    }

    /// The color named `name`.
    pub fn color_by_name(&self, name: &str) -> Option<EntityId> {
        self.colors.values().find(|c| c.name == name).map(|c| c.id)
    }

    /// The color of a volume.
    pub fn volume_color(&self, v: EntityId) -> Option<EntityId> {
        self.volume_color.get(&v).copied()
    }

    /// The color of a face.
    pub fn face_color(&self, f: EntityId) -> Option<EntityId> {
        self.face_color.get(&f).copied()
    }

    /// Volumes with color `c`, ascending.
    pub fn volumes_of_color(&self, c: EntityId) -> Vec<EntityId> {
        self.volume_color.iter().filter(|(_, x)| **x == c).map(|(v, _)| *v).collect()
    }

    /// Faces with color `c`, ascending.
    pub fn faces_of_color(&self, c: EntityId) -> Vec<EntityId> {
        self.face_color.iter().filter(|(_, x)| **x == c).map(|(f, _)| *f).collect()
    }

    /// The whole volume coloring as `(volume, color)`, ascending by volume. Keys are stored ids,
    /// live or not (on an overlay a colored host volume may be hidden; see
    /// [`Self::live_volume_color`]).
    pub fn volume_coloring(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.volume_color.iter().map(|(k, v)| (*k, *v))
    }

    /// The whole face coloring as `(face, color)`, ascending by face (stored keys, live or not:
    /// this is how hidden host faces of an overlay are enumerated).
    pub fn face_coloring(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.face_color.iter().map(|(k, v)| (*k, *v))
    }

    /// The color a live face `f` carries in `t`: its own color first, otherwise the color of a
    /// stored (hidden) host face whose `live_cover` contains `f` (the lowest such host id). On a
    /// plain complex this is [`Self::face_color`] for live faces. The fallback scans the face
    /// coloring (`O(colored faces)`); `validate` builds the inverse index once instead.
    pub fn live_face_color(&self, f: EntityId, t: &dyn TopologyView) -> Option<EntityId> {
        live_color(&self.face_color, f, t)
    }

    /// The color a live volume (or overlay piece) `v` carries in `t`: its own color first,
    /// otherwise the color of a stored host volume whose `live_cover` contains `v` (the lowest
    /// such host id). Same cost note as [`Self::live_face_color`].
    pub fn live_volume_color(&self, v: EntityId, t: &dyn TopologyView) -> Option<EntityId> {
        live_color(&self.volume_color, v, t)
    }

    /// Check one prospective member: allowed kind, and present when the model mints its kind.
    pub(crate) fn check_group_member(&self, m: EntityId) -> Result<()> {
        if !group_member_kind_ok(m) {
            return Err(ModelError::Invalid(format!("{m} cannot be a group member")));
        }
        let known = match m.kind() {
            EntityKind::Color => self.colors.contains_key(&m),
            EntityKind::Group => self.groups.contains_key(&m),
            EntityKind::Boundary => self.boundaries.contains_key(&m),
            EntityKind::Instance => self.instancing.instances.contains_key(&m),
            EntityKind::Prototype => self.instancing.prototypes.contains_key(&m),
            EntityKind::Lattice => self.instancing.lattices.contains_key(&m),
            EntityKind::Cluster => self.instancing.clusters.contains_key(&m),
            EntityKind::PartitionCell => self.decomposition.cells.contains_key(&m),
            _ => true,
        };
        if known { Ok(()) } else { Err(ModelError::Unknown(m)) }
    }

    /// A group reachable from `from` through nested groups, if `target` is one (cycle check).
    fn group_reaches(&self, from: EntityId, target: EntityId, depth: u32) -> bool {
        if from == target {
            return true;
        }
        if depth > 1024 {
            return true;
        }
        self.groups.get(&from).is_some_and(|g| g.members.iter().filter(|m| m.kind() == EntityKind::Group).any(|m| self.group_reaches(*m, target, depth + 1)))
    }

    /// Add a named group (members: see [`group_member_kind_ok`]; model-minted members must
    /// exist).
    pub fn add_group(&mut self, name: &str, members: &[EntityId]) -> Result<EntityId> {
        if self.groups.values().any(|g| g.name == name) {
            return Err(ModelError::NameTaken(name.to_owned()));
        }
        for m in members {
            self.check_group_member(*m)?;
        }
        let id = self.ids.alloc(EntityKind::Group)?;
        self.groups.insert(id, GroupRow { id, name: name.to_owned(), members: members.iter().copied().collect() });
        Ok(id)
    }

    /// Edit a group's members. Atomic: every entity is checked first (kind, existence, and that
    /// a nested group would not create a cycle).
    pub fn group_edit(&mut self, g: EntityId, edit: GroupEdit, entities: &[EntityId]) -> Result<()> {
        expect_kind(g, EntityKind::Group)?;
        let row = self.groups.get(&g).ok_or(ModelError::Unknown(g))?;
        if !matches!(edit, GroupEdit::Remove) {
            for m in entities {
                self.check_group_member(*m)?;
                if m.kind() == EntityKind::Group && self.group_reaches(*m, g, 0) {
                    return Err(ModelError::Cycle(*m));
                }
            }
        }
        let mut members = row.members.clone();
        match edit {
            GroupEdit::Add => members.extend(entities.iter().copied()),
            GroupEdit::Remove => entities.iter().for_each(|m| {
                members.remove(m);
            }),
            GroupEdit::Equals => members = entities.iter().copied().collect(),
            GroupEdit::Xor => {
                for m in entities.iter().copied().collect::<BTreeSet<_>>() {
                    if !members.remove(&m) {
                        members.insert(m);
                    }
                }
            }
        }
        if let Some(row) = self.groups.get_mut(&g) {
            row.members = members;
        }
        Ok(())
    }

    /// Remove a group. Fails while another group lists it ([`ModelError::Duplicate`] is not
    /// used: the error is [`ModelError::Invalid`] naming the referrer).
    pub fn remove_group(&mut self, g: EntityId) -> Result<()> {
        expect_kind(g, EntityKind::Group)?;
        if !self.groups.contains_key(&g) {
            return Err(ModelError::Unknown(g));
        }
        if let Some(r) = self.groups.values().find(|r| r.members.contains(&g)) {
            return Err(ModelError::Invalid(format!("group {g} is a member of group {}", r.id)));
        }
        self.groups.remove(&g);
        Ok(())
    }

    /// Rename a group (names are unique).
    pub fn rename_group(&mut self, g: EntityId, name: &str) -> Result<()> {
        expect_kind(g, EntityKind::Group)?;
        if !self.groups.contains_key(&g) {
            return Err(ModelError::Unknown(g));
        }
        if self.groups.values().any(|r| r.name == name && r.id != g) {
            return Err(ModelError::NameTaken(name.to_owned()));
        }
        if let Some(row) = self.groups.get_mut(&g) {
            row.name = name.to_owned();
        }
        Ok(())
    }

    /// The group named `name`.
    pub fn group_by_name(&self, name: &str) -> Option<EntityId> {
        self.groups.values().find(|g| g.name == name).map(|g| g.id)
    }

    /// All groups, ascending by id.
    pub fn groups(&self) -> impl Iterator<Item = &GroupRow> {
        self.groups.values()
    }

    fn group_combine(&mut self, name: &str, a: EntityId, b: EntityId, f: impl Fn(&BTreeSet<EntityId>, &BTreeSet<EntityId>) -> BTreeSet<EntityId>) -> Result<EntityId> {
        for g in [a, b] {
            expect_kind(g, EntityKind::Group)?;
        }
        let ra = self.groups.get(&a).ok_or(ModelError::Unknown(a))?;
        let rb = self.groups.get(&b).ok_or(ModelError::Unknown(b))?;
        let members: Vec<EntityId> = f(&ra.members, &rb.members).into_iter().collect();
        self.add_group(name, &members)
    }

    /// New group `name` with the direct members of `a` or `b`.
    pub fn group_unite(&mut self, name: &str, a: EntityId, b: EntityId) -> Result<EntityId> {
        self.group_combine(name, a, b, |x, y| x | y)
    }

    /// New group `name` with the direct members of both `a` and `b`.
    pub fn group_intersect(&mut self, name: &str, a: EntityId, b: EntityId) -> Result<EntityId> {
        self.group_combine(name, a, b, |x, y| x & y)
    }

    /// New group `name` with the direct members of `a` that are not in `b`.
    pub fn group_subtract(&mut self, name: &str, a: EntityId, b: EntityId) -> Result<EntityId> {
        self.group_combine(name, a, b, |x, y| x - y)
    }

    /// Re-key group members through a successor map (the op layer's succession hook:
    /// `old -> successors`, a copy is `A -> [A, A']`, an empty list removes). Applies to every
    /// member kind; members without an entry pass through unchanged.
    pub fn apply_group_successors(&mut self, succ: &std::collections::BTreeMap<EntityId, Vec<EntityId>>) {
        for g in self.groups.values_mut() {
            if g.members.iter().any(|m| succ.contains_key(m)) {
                g.members = g.members.iter().flat_map(|m| succ.get(m).cloned().unwrap_or_else(|| vec![*m])).collect();
            }
        }
    }

    /// Whether any group lists one of `ids` (cheap test for the succession hook's gating).
    pub fn groups_reference_any(&self, ids: &[EntityId]) -> bool {
        self.groups.values().any(|g| ids.iter().any(|i| g.members.contains(i)))
    }

    /// Entities of rank `dim` reachable from group `g`: members of that rank, the closure of
    /// higher-rank members (`TopologyView::closure`, so a volume brings its faces, edges and
    /// vertices), the volumes or faces of member colors, and nested groups (cycle-checked).
    /// Ascending, deduplicated. Members of lower rank and non-topological members contribute
    /// nothing.
    ///
    /// Every topological member (`Topo*`, `Partition*`, and the volumes and faces of member
    /// colors) is first mapped through [`TopologyView::live_cover`] before the rank test and the
    /// closure: a live cell stays itself, a split host cell becomes its live descendants and,
    /// on an [`sm_topo_api::Overlay`] view, a hidden host cell becomes every live overlay cell
    /// standing for it (partition children, composites). A deleted member contributes nothing.
    pub fn group_entities(&self, g: EntityId, t: &dyn TopologyView, dim: Dim) -> Result<Vec<EntityId>> {
        let mut out = BTreeSet::new();
        self.collect_entities(g, t, dim, &mut Vec::new(), &mut out)?;
        Ok(out.into_iter().collect())
    }

    fn collect_entities(&self, g: EntityId, t: &dyn TopologyView, dim: Dim, stack: &mut Vec<EntityId>, out: &mut BTreeSet<EntityId>) -> Result<()> {
        if stack.contains(&g) {
            return Err(ModelError::Cycle(g));
        }
        let row = self.groups.get(&g).ok_or(ModelError::Unknown(g))?;
        stack.push(g);
        let take = |m: EntityId, out: &mut BTreeSet<EntityId>| {
            for e in t.live_cover(m) {
                match t.dim_of(e) {
                    Some(d) if d == dim => {
                        out.insert(e);
                    }
                    Some(d) if d > dim => out.extend(t.closure(e).into_iter().filter(|c| t.dim_of(*c) == Some(dim))),
                    _ => {}
                }
            }
        };
        for m in &row.members {
            match m.kind() {
                EntityKind::Group => self.collect_entities(*m, t, dim, stack, out)?,
                EntityKind::Color => match self.colors.get(m).map(|c| c.domain) {
                    Some(Domain::Volume) => self.volumes_of_color(*m).into_iter().for_each(|v| take(v, out)),
                    Some(Domain::Face) => self.faces_of_color(*m).into_iter().for_each(|f| take(f, out)),
                    None => {}
                },
                k if is_topo_kind(k) => take(*m, out),
                _ => {}
            }
        }
        stack.pop();
        Ok(())
    }

    /// The group row for `id`.
    pub fn group(&self, id: EntityId) -> Option<&GroupRow> {
        self.groups.get(&id)
    }

    /// Volumes reachable from a group: volume members, volumes of member bodies and colors,
    /// nested groups (cycle-checked). Ascending, deduplicated. Members are mapped through
    /// [`TopologyView::live_cover`] first, as in [`Model::group_entities`]; on an overlay view a
    /// partitioned volume yields its live pieces.
    pub fn group_volumes(&self, g: EntityId, t: &dyn TopologyView) -> Result<Vec<EntityId>> {
        let mut out = BTreeSet::new();
        self.collect_group(g, t, true, &mut Vec::new(), &mut out)?;
        Ok(out.into_iter().collect())
    }

    /// Faces reachable from a group: face members, faces of member volumes and bodies, faces of
    /// member face colors, nested groups. Ascending, deduplicated. Members are mapped through
    /// [`TopologyView::live_cover`] first, as in [`Model::group_entities`]; on an overlay view a
    /// hidden host face yields its live overlay faces.
    pub fn group_faces(&self, g: EntityId, t: &dyn TopologyView) -> Result<Vec<EntityId>> {
        let mut out = BTreeSet::new();
        self.collect_group(g, t, false, &mut Vec::new(), &mut out)?;
        Ok(out.into_iter().collect())
    }

    fn collect_group(&self, g: EntityId, t: &dyn TopologyView, vols: bool, stack: &mut Vec<EntityId>, out: &mut BTreeSet<EntityId>) -> Result<()> {
        if stack.contains(&g) {
            return Err(ModelError::Cycle(g));
        }
        let row = self.groups.get(&g).ok_or(ModelError::Unknown(g))?;
        stack.push(g);
        // One live cell: volumes and faces by rank, bodies through their volume members.
        let take = |e: EntityId, out: &mut BTreeSet<EntityId>| match t.dim_of(e) {
            Some(Dim::Volume) => {
                if vols {
                    out.insert(e);
                } else {
                    out.extend(t.volume_faces(e).iter().copied());
                }
            }
            Some(Dim::Face) if !vols => {
                out.insert(e);
            }
            Some(Dim::Body) => {
                for b in t.boundary(e) {
                    if t.dim_of(b.cell) == Some(Dim::Volume) {
                        if vols {
                            out.insert(b.cell);
                        } else {
                            out.extend(t.volume_faces(b.cell).iter().copied());
                        }
                    }
                }
            }
            _ => {}
        };
        let cover = |m: EntityId, out: &mut BTreeSet<EntityId>| t.live_cover(m).into_iter().for_each(|e| take(e, out));
        for m in &row.members {
            match m.kind() {
                EntityKind::Color => match (self.colors.get(m).map(|c| c.domain), vols) {
                    (Some(Domain::Volume), _) => self.volumes_of_color(*m).into_iter().for_each(|v| cover(v, out)),
                    (Some(Domain::Face), false) => self.faces_of_color(*m).into_iter().for_each(|f| cover(f, out)),
                    _ => {}
                },
                EntityKind::Group => self.collect_group(*m, t, vols, stack, out)?,
                k if is_topo_kind(k) => cover(*m, out),
                _ => {}
            }
        }
        stack.pop();
        Ok(())
    }
}

/// Topology cells (host or overlay) that groups map through `live_cover`.
fn is_topo_kind(k: EntityKind) -> bool {
    use EntityKind::*;
    matches!(k, TopoVertex | TopoEdge | TopoFace | TopoVolume | TopoBody | PartitionCell | PartitionFace | PartitionEdge | PartitionVertex)
}
