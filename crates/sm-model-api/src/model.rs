//! The [`Model`]: all model-semantics and decomposition tables in one value.
//!
//! The model never stores topology. It stores attribute tables keyed by topology ids
//! (the Salome-group / Simmetrix-attribute-case style): a row exists for an entity or it does not.
//! Mutation goes through methods (which keep kinds and domains right); [`crate::validate`]
//! checks the invariants that need topology.

use crate::boundary::{BoundaryRow, FaceBoundary};
use crate::color::{ColorRow, GroupRow};
use crate::decomposition::{Decomposition, PieceFace};
use crate::error::{ModelError, Result};
use crate::instancing::{Instancing, Member, Override};
use crate::symmetry::Symmetry;
use sm_core::{EntityId, EntityKind};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Deterministic per-kind serial allocator for ids created by this layer
/// (`Color`, `Group`, `Boundary`, `Instance`, `PartitionCell`). Serials start at 1, never reused.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IdAlloc {
    next: BTreeMap<u8, u64>,
}

impl IdAlloc {
    /// Allocate the next id of `kind`.
    pub fn alloc(&mut self, kind: EntityKind) -> Result<EntityId> {
        let n = self.next.entry(kind as u8).or_insert(1);
        let id = EntityId::new(kind, *n).ok_or(ModelError::IdExhausted(kind))?;
        *n += 1;
        Ok(id)
    }
}

/// Old entity -> its descendants after a topology operation (empty = dropped). This is the
/// "lineage" of docs/ARCHITECTURE.md §3: the model tables are re-keyed through it.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lineage {
    /// The entity before the operation.
    pub old: EntityId,
    /// Its descendants (usually one; two if split; none if removed).
    pub new: SmallVec<[EntityId; 2]>,
}

/// The model-semantics and decomposition layers.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Model {
    /// Id allocator for ids this layer creates.
    pub ids: IdAlloc,
    /// Color table.
    pub colors: BTreeMap<EntityId, ColorRow>,
    /// Volume coloring: `TopoVolume -> Color`. Must be total over the topology's volumes.
    pub volume_color: BTreeMap<EntityId, EntityId>,
    /// Boundary coloring: `TopoFace -> Color`. Sparse.
    pub face_color: BTreeMap<EntityId, EntityId>,
    /// Named groups.
    pub groups: BTreeMap<EntityId, GroupRow>,
    /// Boundary records.
    pub boundaries: BTreeMap<EntityId, BoundaryRow>,
    /// Face membership/pairing in boundary records.
    pub face_boundary: BTreeMap<EntityId, FaceBoundary>,
    /// Declared/detected/cut symmetry, if any.
    pub symmetry: Option<Symmetry>,
    /// Prototypes, instances, lattices.
    pub instancing: Instancing,
    /// Optional decomposition into pieces.
    pub decomposition: Option<Decomposition>,
}

fn expand(map: &BTreeMap<EntityId, SmallVec<[EntityId; 2]>>, id: EntityId) -> SmallVec<[EntityId; 2]> {
    map.get(&id).cloned().unwrap_or_else(|| SmallVec::from_slice(&[id]))
}

fn expand_sorted(map: &BTreeMap<EntityId, SmallVec<[EntityId; 2]>>, ids: &[EntityId]) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = ids.iter().flat_map(|i| expand(map, *i)).collect();
    v.sort();
    v.dedup();
    v
}

fn one(map: &BTreeMap<EntityId, SmallVec<[EntityId; 2]>>, id: EntityId) -> Result<Option<EntityId>> {
    let e = expand(map, id);
    match e.len() {
        0 => Ok(None),
        1 => Ok(Some(e[0])),
        _ => Err(ModelError::Invalid(format!("{id} split into several entities; it cannot be a pairing mate or cut target"))),
    }
}

impl Model {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    /// Re-key every table through `lineage` after a topology operation: colors, groups, boundary
    /// membership and mates, symmetry domain, prototype members, overrides, pieces. Entities
    /// with an empty descendant list vanish from the tables. Transactional: on error the
    /// model is unchanged.
    pub fn apply_lineage(&mut self, lineage: &[Lineage]) -> Result<()> {
        let map: BTreeMap<EntityId, SmallVec<[EntityId; 2]>> = lineage.iter().map(|l| (l.old, l.new.clone())).collect();
        let mut next = self.clone();

        let rekey = |m: &BTreeMap<EntityId, EntityId>| -> BTreeMap<EntityId, EntityId> {
            m.iter().flat_map(|(k, v)| expand(&map, *k).into_iter().map(move |nk| (nk, *v))).collect()
        };
        next.volume_color = rekey(&self.volume_color);
        next.face_color = rekey(&self.face_color);
        for g in next.groups.values_mut() {
            g.members = expand_sorted(&map, &g.members);
        }
        next.face_boundary.clear();
        for (f, fb) in &self.face_boundary {
            let mate = match fb.mate {
                Some(m) => one(&map, m)?,
                None => None,
            };
            for nf in expand(&map, *f) {
                next.face_boundary.insert(nf, FaceBoundary { mate, ..*fb });
            }
        }
        if let Some(s) = &mut next.symmetry {
            s.domain = expand_sorted(&map, &s.domain);
        }
        let ov = |o: &Override| -> Result<Vec<Override>> {
            Ok(match *o {
                Override::SetVolume { volume, to } => expand(&map, volume).into_iter().map(|v| Override::SetVolume { volume: v, to }).collect(),
                other => vec![other],
            })
        };
        let ov_all = |list: &mut Vec<Override>| -> Result<()> {
            let mut out = Vec::new();
            for o in list.iter() {
                out.extend(ov(o)?);
            }
            *list = out;
            Ok(())
        };
        for p in next.instancing.prototypes.values_mut() {
            p.members = p
                .members
                .iter()
                .flat_map(|m| match *m {
                    Member::Volume(v) => expand(&map, v).into_iter().map(Member::Volume).collect::<Vec<_>>(),
                    other => vec![other],
                })
                .collect();
        }
        for i in next.instancing.instances.values_mut() {
            ov_all(&mut i.overrides)?;
        }
        for l in next.instancing.lattices.values_mut() {
            for s in l.slots.values_mut() {
                ov_all(&mut s.overrides)?;
            }
        }
        if let Some(d) = &mut next.decomposition {
            for p in d.pieces.values_mut() {
                p.volumes = expand_sorted(&map, &p.volumes);
                let mut faces: Vec<PieceFace> = p
                    .faces
                    .iter()
                    .flat_map(|f| match *f {
                        PieceFace::Original(e) => expand(&map, e).into_iter().map(PieceFace::Original).collect::<Vec<_>>(),
                        c => vec![c],
                    })
                    .collect();
                faces.sort();
                faces.dedup();
                p.faces = faces;
            }
            d.pieces.retain(|_, p| !p.volumes.is_empty());
            for c in d.cuts.values_mut() {
                if let Some(v) = one(&map, c.volume)? {
                    c.volume = v;
                }
            }
        }
        *self = next;
        Ok(())
    }
}
