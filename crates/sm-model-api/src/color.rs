//! Coloring tables: abstract colors on volumes and on faces/boundaries, plus named groups.
//!
//! Colors are labels only. The mesher colors; the consuming program resolves colors into
//! materials and boundary conditions. Volume coloring is an exact partition (every volume has
//! exactly one color); face coloring is sparse.

use crate::error::{ModelError, Result};
use crate::model::Model;
use sm_core::{EntityId, EntityKind};

/// What a color may be attached to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ColorDomain {
    /// Colors volumes (an abstract material id, e.g. "material 1").
    Volume,
    /// Colors faces / boundaries (an abstract boundary id).
    Boundary,
}

/// Row of the color table (key: `EntityKind::Color` id).
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ColorRow {
    /// Unique, user-visible name.
    pub name: String,
    /// What the color may label.
    pub domain: ColorDomain,
}

/// What a group means to the consumer. Purely a label; membership works the same for all.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum GroupKind {
    /// A block (set of volumes, an Exodus-style element block).
    Block,
    /// An assembly (set of groups and/or volumes).
    Assembly,
    /// A generic named set (faces, volumes, colors).
    Set,
}

/// Row of the group table (key: `EntityKind::Group` id). A named set of entities.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupRow {
    /// Unique name.
    pub name: String,
    /// Meaning label.
    pub kind: GroupKind,
    /// Members, sorted ascending and unique. Allowed kinds: `TopoVolume`, `TopoFace`,
    /// `TopoBody`, `Color`, `Group` (nesting).
    pub members: Vec<EntityId>,
}

pub(crate) fn expect_kind(id: EntityId, expected: EntityKind) -> Result<()> {
    if id.kind() == expected { Ok(()) } else { Err(ModelError::WrongKind { id, expected, found: id.kind() }) }
}

impl Model {
    /// Add a color; returns its id. Names must be unique.
    pub fn add_color(&mut self, name: &str, domain: ColorDomain) -> Result<EntityId> {
        if self.colors.values().any(|c| c.name == name) {
            return Err(ModelError::Invalid(format!("duplicate color name {name:?}")));
        }
        let id = self.ids.alloc(EntityKind::Color)?;
        self.colors.insert(id, ColorRow { name: name.to_owned(), domain });
        Ok(id)
    }

    /// Color `volume` with `color` (replaces any previous color).
    pub fn color_volume(&mut self, volume: EntityId, color: EntityId) -> Result<()> {
        expect_kind(volume, EntityKind::TopoVolume)?;
        self.check_color(color, ColorDomain::Volume)?;
        self.volume_color.insert(volume, color);
        Ok(())
    }

    /// Color `face` with a boundary color (replaces any previous color).
    pub fn color_face(&mut self, face: EntityId, color: EntityId) -> Result<()> {
        expect_kind(face, EntityKind::TopoFace)?;
        self.check_color(color, ColorDomain::Boundary)?;
        self.face_color.insert(face, color);
        Ok(())
    }

    fn check_color(&self, color: EntityId, domain: ColorDomain) -> Result<()> {
        expect_kind(color, EntityKind::Color)?;
        match self.colors.get(&color) {
            None => Err(ModelError::Unknown(color)),
            Some(row) if row.domain != domain => Err(ModelError::ColorDomain(color)),
            Some(_) => Ok(()),
        }
    }

    /// Add a named group with the given members (sorted and deduplicated for you).
    pub fn add_group(&mut self, name: &str, kind: GroupKind, members: &[EntityId]) -> Result<EntityId> {
        if self.groups.values().any(|g| g.name == name) {
            return Err(ModelError::Invalid(format!("duplicate group name {name:?}")));
        }
        for m in members {
            if !matches!(m.kind(), EntityKind::TopoVolume | EntityKind::TopoFace | EntityKind::TopoBody | EntityKind::Color | EntityKind::Group) {
                return Err(ModelError::Invalid(format!("{m} cannot be a group member")));
            }
        }
        let mut members = members.to_vec();
        members.sort();
        members.dedup();
        let id = self.ids.alloc(EntityKind::Group)?;
        self.groups.insert(id, GroupRow { name: name.to_owned(), kind, members });
        Ok(id)
    }

    /// Volumes with the given color, ascending.
    pub fn volumes_of_color(&self, color: EntityId) -> Vec<EntityId> {
        self.volume_color.iter().filter(|(_, c)| **c == color).map(|(v, _)| *v).collect()
    }

    /// Volumes reachable from a group, expanding nested groups (cycle-safe, ascending).
    pub fn group_volumes(&self, group: EntityId) -> Result<Vec<EntityId>> {
        let mut out = Vec::new();
        let mut seen = Vec::new();
        self.collect_group(group, &mut seen, &mut out)?;
        out.sort();
        out.dedup();
        Ok(out)
    }

    fn collect_group(&self, g: EntityId, stack: &mut Vec<EntityId>, out: &mut Vec<EntityId>) -> Result<()> {
        if stack.contains(&g) {
            return Err(ModelError::Cycle(g));
        }
        let row = self.groups.get(&g).ok_or(ModelError::Unknown(g))?;
        stack.push(g);
        for m in &row.members {
            match m.kind() {
                EntityKind::TopoVolume => out.push(*m),
                EntityKind::Group => self.collect_group(*m, stack, out)?,
                _ => {}
            }
        }
        stack.pop();
        Ok(())
    }
}
