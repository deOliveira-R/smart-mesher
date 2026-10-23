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
use sm_topo_api::{Dim, TopologyView};
use std::collections::BTreeSet;

/// What a color may be attached to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Domain {
    /// Colors volumes (an abstract material).
    Volume,
    /// Colors faces (an abstract boundary or interface label).
    Face,
}

/// Row of the color table.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

/// Row of the group table: a named set of entities of any kind, expanded recursively.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupRow {
    /// The `EntityKind::Group` id.
    pub id: EntityId,
    /// Unique name.
    pub name: String,
    /// Members: `TopoVolume`, `TopoFace`, `TopoBody`, `Color` or nested `Group`.
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

    /// Color `faces` with the face color `c`, replacing any previous color.
    pub fn color_faces(&mut self, c: EntityId, faces: &[EntityId]) -> Result<()> {
        self.check_color(c, Domain::Face)?;
        for f in faces {
            expect_kind(*f, EntityKind::TopoFace)?;
        }
        for f in faces {
            self.face_color.insert(*f, c);
        }
        Ok(())
    }

    /// Remove the color of `faces`.
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

    /// Add a named group of entities (volumes, faces, bodies, colors, groups).
    pub fn add_group(&mut self, name: &str, members: &[EntityId]) -> Result<EntityId> {
        if self.groups.values().any(|g| g.name == name) {
            return Err(ModelError::NameTaken(name.to_owned()));
        }
        for m in members {
            if !matches!(m.kind(), EntityKind::TopoVolume | EntityKind::TopoFace | EntityKind::TopoBody | EntityKind::Color | EntityKind::Group) {
                return Err(ModelError::Invalid(format!("{m} cannot be a group member")));
            }
            if m.kind() == EntityKind::Color && !self.colors.contains_key(m) {
                return Err(ModelError::Unknown(*m));
            }
            if m.kind() == EntityKind::Group && !self.groups.contains_key(m) {
                return Err(ModelError::Unknown(*m));
            }
        }
        let id = self.ids.alloc(EntityKind::Group)?;
        self.groups.insert(id, GroupRow { id, name: name.to_owned(), members: members.iter().copied().collect() });
        Ok(id)
    }

    /// The group row for `id`.
    pub fn group(&self, id: EntityId) -> Option<&GroupRow> {
        self.groups.get(&id)
    }

    /// Volumes reachable from a group: volume members, volumes of member bodies and colors,
    /// nested groups (cycle-checked). Ascending, deduplicated.
    pub fn group_volumes(&self, g: EntityId, t: &dyn TopologyView) -> Result<Vec<EntityId>> {
        let mut out = BTreeSet::new();
        self.collect_group(g, t, true, &mut Vec::new(), &mut out)?;
        Ok(out.into_iter().collect())
    }

    /// Faces reachable from a group: face members, faces of member volumes and bodies, faces of
    /// member face colors, nested groups. Ascending, deduplicated.
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
        for m in &row.members {
            match m.kind() {
                EntityKind::TopoVolume => {
                    if vols {
                        out.insert(*m);
                    } else {
                        out.extend(t.volume_faces(*m).iter().copied());
                    }
                }
                EntityKind::TopoFace => {
                    if !vols {
                        out.insert(*m);
                    }
                }
                EntityKind::TopoBody => {
                    for e in t.boundary(*m) {
                        if t.dim_of(e.cell) == Some(Dim::Volume) {
                            if vols {
                                out.insert(e.cell);
                            } else {
                                out.extend(t.volume_faces(e.cell).iter().copied());
                            }
                        }
                    }
                }
                EntityKind::Color => match (self.colors.get(m).map(|c| c.domain), vols) {
                    (Some(Domain::Volume), true) => out.extend(self.volumes_of_color(*m)),
                    (Some(Domain::Volume), false) => {
                        for v in self.volumes_of_color(*m) {
                            out.extend(t.volume_faces(v).iter().copied());
                        }
                    }
                    (Some(Domain::Face), false) => out.extend(self.faces_of_color(*m)),
                    _ => {}
                },
                EntityKind::Group => self.collect_group(*m, t, vols, stack, out)?,
                _ => {}
            }
        }
        stack.pop();
        Ok(())
    }
}
