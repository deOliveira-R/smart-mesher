//! Coloring: labels on volumes and on faces/boundaries, plus named groupings.
//!
//! Colors are **labels only** (abstract material ids such as "material 1"). The mesher never
//! attaches physics; the consuming program resolves colors into properties. In group-theory terms a
//! coloring is a function on the quotient: `volume -> color` defined on the fundamental domain and
//! pulled back along the covering map to every unfolded or instanced copy (see
//! [`crate::symmetry::Cover::volume_color`] and [`crate::instancing::FlatItem::volume_color`]).

use crate::error::{ModelError, Result};
use crate::ids::IdAllocator;
use sm_core::{EntityId, EntityKind};
use std::collections::{BTreeMap, BTreeSet};

/// What a color labels. Volume colors partition volumes exactly; boundary colors label faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ColorRole {
    /// Labels volumes (a "material").
    Volume,
    /// Labels faces / boundaries (a "boundary color").
    Boundary,
}

/// A color definition.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColorInfo {
    /// Unique (per role) display name.
    pub name: String,
    /// Volume or boundary color.
    pub role: ColorRole,
}

/// How a named group is used. Purely descriptive.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GroupKind {
    /// Exodus-style block: a set of volumes (and via them the cells).
    Block,
    /// Assembly: a set of volumes and/or nested groups.
    Assembly,
    /// Generic named set (faces, volumes, groups).
    Set,
}

/// A named set of entities. Members may be volumes, faces, colors or other groups (nesting allowed,
/// acyclic).
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Group {
    /// Unique name.
    pub name: String,
    /// Usage hint.
    pub kind: GroupKind,
    /// Direct members, ordered by id.
    pub members: BTreeSet<EntityId>,
}

/// Colors, the volume/face assignments, and groupings. All maps are ordered (deterministic).
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Coloring {
    colors: BTreeMap<EntityId, ColorInfo>,
    volume_color: BTreeMap<EntityId, EntityId>,
    face_color: BTreeMap<EntityId, EntityId>,
    groups: BTreeMap<EntityId, Group>,
}

fn expect_kind(id: EntityId, kind: EntityKind, what: &'static str) -> Result<()> {
    if id.kind() == kind { Ok(()) } else { Err(ModelError::WrongKind { id, found: id.kind(), expected: what }) }
}

impl Coloring {
    /// An empty coloring.
    pub fn new() -> Self {
        Self::default()
    }

    /// Define a color. Names are unique within a role.
    pub fn define_color(&mut self, ids: &mut dyn IdAllocator, name: &str, role: ColorRole) -> Result<EntityId> {
        if self.colors.values().any(|c| c.role == role && c.name == name) {
            return Err(ModelError::DuplicateName(name.to_string()));
        }
        let id = ids.alloc(EntityKind::Color);
        self.colors.insert(id, ColorInfo { name: name.to_string(), role });
        Ok(id)
    }

    /// Look up a color by role and name.
    pub fn color_by_name(&self, role: ColorRole, name: &str) -> Option<EntityId> {
        self.colors.iter().find(|(_, c)| c.role == role && c.name == name).map(|(&id, _)| id)
    }

    /// Color definition.
    pub fn color_info(&self, color: EntityId) -> Option<&ColorInfo> {
        self.colors.get(&color)
    }

    /// All colors in id order.
    pub fn colors(&self) -> impl Iterator<Item = (EntityId, &ColorInfo)> {
        self.colors.iter().map(|(&id, c)| (id, c))
    }

    /// Color a volume (replacing any previous color: the partition stays exact by construction).
    pub fn assign_volume(&mut self, volume: EntityId, color: EntityId) -> Result<()> {
        expect_kind(volume, EntityKind::TopoVolume, "TopoVolume")?;
        self.check_color(color, ColorRole::Volume)?;
        self.volume_color.insert(volume, color);
        Ok(())
    }

    /// Color a face / boundary.
    pub fn assign_face(&mut self, face: EntityId, color: EntityId) -> Result<()> {
        expect_kind(face, EntityKind::TopoFace, "TopoFace")?;
        self.check_color(color, ColorRole::Boundary)?;
        self.face_color.insert(face, color);
        Ok(())
    }

    fn check_color(&self, color: EntityId, role: ColorRole) -> Result<()> {
        match self.colors.get(&color) {
            None => Err(ModelError::UnknownEntity(color)),
            Some(c) if c.role != role => Err(ModelError::Invalid(format!("color {color} has role {:?}, need {role:?}", c.role))),
            Some(_) => Ok(()),
        }
    }

    /// Color of a volume.
    pub fn volume_color(&self, volume: EntityId) -> Option<EntityId> {
        self.volume_color.get(&volume).copied()
    }

    /// Color of a face.
    pub fn face_color(&self, face: EntityId) -> Option<EntityId> {
        self.face_color.get(&face).copied()
    }

    /// Volumes of a color, sorted.
    pub fn volumes_of(&self, color: EntityId) -> Vec<EntityId> {
        self.volume_color.iter().filter(|&(_, &c)| c == color).map(|(&v, _)| v).collect()
    }

    /// Faces of a color, sorted.
    pub fn faces_of(&self, color: EntityId) -> Vec<EntityId> {
        self.face_color.iter().filter(|&(_, &c)| c == color).map(|(&f, _)| f).collect()
    }

    /// All `(volume, color)` assignments in volume order.
    pub fn volume_assignments(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.volume_color.iter().map(|(&v, &c)| (v, c))
    }

    /// All `(face, color)` assignments in face order.
    pub fn face_assignments(&self) -> impl Iterator<Item = (EntityId, EntityId)> + '_ {
        self.face_color.iter().map(|(&f, &c)| (f, c))
    }

    /// Define an empty named group.
    pub fn define_group(&mut self, ids: &mut dyn IdAllocator, name: &str, kind: GroupKind) -> Result<EntityId> {
        if self.groups.values().any(|g| g.name == name) {
            return Err(ModelError::DuplicateName(name.to_string()));
        }
        let id = ids.alloc(EntityKind::Group);
        self.groups.insert(id, Group { name: name.to_string(), kind, members: BTreeSet::new() });
        Ok(id)
    }

    /// Add a member (volume, face, color or group). Nesting must stay acyclic.
    pub fn add_to_group(&mut self, group: EntityId, member: EntityId) -> Result<()> {
        if !self.groups.contains_key(&group) {
            return Err(ModelError::UnknownEntity(group));
        }
        match member.kind() {
            EntityKind::TopoVolume | EntityKind::TopoFace | EntityKind::Color => {}
            EntityKind::Group => {
                if !self.groups.contains_key(&member) {
                    return Err(ModelError::UnknownEntity(member));
                }
                if member == group || self.group_reaches(member, group) {
                    return Err(ModelError::Cycle(member));
                }
            }
            k => return Err(ModelError::WrongKind { id: member, found: k, expected: "volume, face, color or group" }),
        }
        if let Some(g) = self.groups.get_mut(&group) {
            g.members.insert(member);
        }
        Ok(())
    }

    fn group_reaches(&self, from: EntityId, target: EntityId) -> bool {
        let mut stack = vec![from];
        let mut seen = BTreeSet::new();
        while let Some(g) = stack.pop() {
            if g == target {
                return true;
            }
            if !seen.insert(g) {
                continue;
            }
            if let Some(grp) = self.groups.get(&g) {
                stack.extend(grp.members.iter().copied().filter(|m| m.kind() == EntityKind::Group));
            }
        }
        false
    }

    /// True if `group` can reach itself through nested membership (only possible in loaded data).
    pub fn has_group_cycle(&self, group: EntityId) -> bool {
        let Some(g) = self.groups.get(&group) else { return false };
        g.members.iter().filter(|m| m.kind() == EntityKind::Group).any(|&m| m == group || self.group_reaches(m, group))
    }

    /// Group definition.
    pub fn group(&self, group: EntityId) -> Option<&Group> {
        self.groups.get(&group)
    }

    /// Look up a group by name.
    pub fn group_by_name(&self, name: &str) -> Option<EntityId> {
        self.groups.iter().find(|(_, g)| g.name == name).map(|(&id, _)| id)
    }

    /// All groups in id order.
    pub fn groups(&self) -> impl Iterator<Item = (EntityId, &Group)> {
        self.groups.iter().map(|(&id, g)| (id, g))
    }

    /// Transitive members of a group of the given kind (nested groups expanded), sorted.
    pub fn expand_group(&self, group: EntityId, kind: EntityKind) -> Result<Vec<EntityId>> {
        if !self.groups.contains_key(&group) {
            return Err(ModelError::UnknownEntity(group));
        }
        let mut out = BTreeSet::new();
        let mut stack = vec![group];
        let mut seen = BTreeSet::new();
        while let Some(g) = stack.pop() {
            if !seen.insert(g) {
                continue;
            }
            if let Some(grp) = self.groups.get(&g) {
                for &m in &grp.members {
                    if m.kind() == EntityKind::Group {
                        stack.push(m);
                    } else if m.kind() == kind {
                        out.insert(m);
                    }
                }
            }
        }
        Ok(out.into_iter().collect())
    }

    /// Topology evolution: `new` inherits everything `old` had (color and group membership). Used
    /// when an operation splits an entity; `old` stays until [`Coloring::retire`] is called.
    pub fn inherit(&mut self, old: EntityId, new: EntityId) {
        if old == new {
            return;
        }
        match old.kind() {
            EntityKind::TopoVolume => {
                if let Some(c) = self.volume_color.get(&old).copied() {
                    self.volume_color.entry(new).or_insert(c);
                }
            }
            EntityKind::TopoFace => {
                if let Some(c) = self.face_color.get(&old).copied() {
                    self.face_color.entry(new).or_insert(c);
                }
            }
            _ => return,
        }
        for g in self.groups.values_mut() {
            if g.members.contains(&old) {
                g.members.insert(new);
            }
        }
    }

    /// Remove every trace of a retired volume or face.
    pub fn retire(&mut self, entity: EntityId) {
        self.volume_color.remove(&entity);
        self.face_color.remove(&entity);
        for g in self.groups.values_mut() {
            g.members.remove(&entity);
        }
    }
}
