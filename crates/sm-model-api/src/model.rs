//! The [`Model`]: all model-semantics and decomposition tables in one value.
//!
//! The model never stores topology. It stores attribute tables keyed by topology ids (the
//! Salome-group / Simmetrix-attribute-case style): a row exists for an entity or it does not.
//! Tables are private; mutation goes through methods that keep kinds, domains and pairings right,
//! and [`crate::validate()`] checks the invariants that need topology.

use crate::boundary::{BoundaryRow, BoundaryKind, FaceBoundary, FaceRole};
use crate::color::{ColorRow, GroupRow};
use crate::decomposition::Decomposition;
use crate::error::{ModelError, Result};
use crate::instancing::{ColorMap, Instancing};
use crate::symmetry::{Provenance, Symmetry, pair_by_moments};
use crate::xf::{GeomCache, Tolerance};
use sm_core::{EntityId, EntityKind};
use sm_topo_api::{Lineage, TopologyView};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Deterministic per-kind serial allocator for ids created by this layer (`Color`, `Group`,
/// `Boundary`, `Prototype`, `Lattice`, `Instance`, `PartitionCell`). Serials start at 1 and are
/// never reused, so replaying the same edits reproduces the same ids.
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

    /// The serial the next id of `kind` will get.
    pub fn peek(&self, kind: EntityKind) -> u64 {
        self.next.get(&(kind as u8)).copied().unwrap_or(1)
    }
}

/// The model-semantics and decomposition layers.
///
/// With the `serde` feature, deserialization runs [`Model::check_integrity`], so a model read
/// from untrusted data cannot hold cyclic cut trees, rows under the wrong kind or a reset id
/// allocator.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "ModelData"))]
pub struct Model {
    pub(crate) ids: IdAlloc,
    pub(crate) colors: BTreeMap<EntityId, ColorRow>,
    pub(crate) volume_color: BTreeMap<EntityId, EntityId>,
    pub(crate) face_color: BTreeMap<EntityId, EntityId>,
    pub(crate) groups: BTreeMap<EntityId, GroupRow>,
    pub(crate) boundaries: BTreeMap<EntityId, BoundaryRow>,
    pub(crate) face_boundary: BTreeMap<EntityId, FaceBoundary>,
    pub(crate) symmetry: Option<Symmetry>,
    pub(crate) provenance: Vec<Provenance>,
    pub(crate) cuts: Vec<Vec<Lineage>>,
    pub(crate) instancing: Instancing,
    pub(crate) decomposition: Decomposition,
}

/// Wire shape of [`Model`]; converted through [`Model::check_integrity`].
#[cfg(feature = "serde")]
#[derive(serde::Deserialize)]
struct ModelData {
    ids: IdAlloc,
    colors: BTreeMap<EntityId, ColorRow>,
    volume_color: BTreeMap<EntityId, EntityId>,
    face_color: BTreeMap<EntityId, EntityId>,
    groups: BTreeMap<EntityId, GroupRow>,
    boundaries: BTreeMap<EntityId, BoundaryRow>,
    face_boundary: BTreeMap<EntityId, FaceBoundary>,
    symmetry: Option<Symmetry>,
    provenance: Vec<Provenance>,
    cuts: Vec<Vec<Lineage>>,
    instancing: Instancing,
    decomposition: Decomposition,
}

#[cfg(feature = "serde")]
impl TryFrom<ModelData> for Model {
    type Error = ModelError;

    fn try_from(d: ModelData) -> Result<Self> {
        let m = Model {
            ids: d.ids,
            colors: d.colors,
            volume_color: d.volume_color,
            face_color: d.face_color,
            groups: d.groups,
            boundaries: d.boundaries,
            face_boundary: d.face_boundary,
            symmetry: d.symmetry,
            provenance: d.provenance,
            cuts: d.cuts,
            instancing: d.instancing,
            decomposition: d.decomposition,
        };
        m.check_integrity()?;
        Ok(m)
    }
}

/// Transitive lineage resolution: old id -> live descendants.
pub(crate) struct Resolver {
    rows: BTreeMap<EntityId, SmallVec<[EntityId; 2]>>,
}

impl Resolver {
    pub(crate) fn new(lineage: &[Lineage]) -> Self {
        let mut rows: BTreeMap<EntityId, SmallVec<[EntityId; 2]>> = BTreeMap::new();
        for l in lineage {
            let e = rows.entry(l.old).or_default();
            for n in &l.new {
                if !e.contains(n) {
                    e.push(*n);
                }
            }
        }
        Self { rows }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    pub(crate) fn changed(&self, e: EntityId) -> bool {
        self.rows.get(&e).is_some_and(|n| n.as_slice() != [e])
    }

    fn go(&self, e: EntityId, depth: u32, out: &mut Vec<EntityId>) {
        match self.rows.get(&e) {
            None => out.push(e),
            Some(_) if depth > 64 => out.push(e),
            Some(new) => {
                for n in new {
                    if *n == e { out.push(e) } else { self.go(*n, depth + 1, out) }
                }
            }
        }
    }

    pub(crate) fn resolve(&self, e: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        self.go(e, 0, &mut out);
        out.sort();
        out.dedup();
        out
    }

    fn set(&self, s: &BTreeSet<EntityId>) -> BTreeSet<EntityId> {
        s.iter().flat_map(|e| self.resolve(*e)).collect()
    }

    /// Re-key a map whose keys are entities; on a merge conflict the entry of the survivor wins,
    /// else the lowest old key. `strict` turns a value conflict into an error.
    fn map(&self, m: &BTreeMap<EntityId, EntityId>, strict: bool) -> Result<BTreeMap<EntityId, EntityId>> {
        let mut cand: BTreeMap<EntityId, (EntityId, EntityId)> = BTreeMap::new();
        for (k, v) in m {
            for nk in self.resolve(*k) {
                match cand.get(&nk) {
                    None => {
                        cand.insert(nk, (*k, *v));
                    }
                    Some((ok, ov)) => {
                        if strict && ov != v {
                            return Err(ModelError::Invalid(format!("merging {ok} and {k} would give {nk} two different colors")));
                        }
                        if (*k == nk) && (*ok != nk) {
                            cand.insert(nk, (*k, *v));
                        }
                    }
                }
            }
        }
        Ok(cand.into_iter().map(|(k, (_, v))| (k, v)).collect())
    }

    fn color_map(&self, c: &ColorMap) -> ColorMap {
        let keyed = |m: &BTreeMap<EntityId, EntityId>| -> BTreeMap<EntityId, EntityId> {
            let mut out = BTreeMap::new();
            for (k, v) in m {
                if matches!(k.kind(), EntityKind::TopoVolume | EntityKind::TopoFace) {
                    for nk in self.resolve(*k) {
                        out.entry(nk).or_insert(*v);
                    }
                } else {
                    out.insert(*k, *v);
                }
            }
            out
        };
        ColorMap { volume: keyed(&c.volume), face: keyed(&c.face) }
    }
}

impl Model {
    /// An empty model.
    pub fn new() -> Self {
        Self::default()
    }

    /// Structural integrity of tables that did not necessarily come through the mutators (for
    /// example after deserialization): row ids match their keys and kinds, the id allocator is
    /// ahead of every minted id (ids are never reused), layouts are bounded, lattices are acyclic
    /// and cut trees are well-formed. Geometry-dependent checks are [`crate::validate()`]'s.
    pub fn check_integrity(&self) -> Result<()> {
        let bad = |what: String| Err(ModelError::Invalid(format!("model integrity: {what}")));
        let mut minted: Vec<EntityId> = Vec::new();
        for (k, r) in &self.colors {
            if r.id != *k || k.kind() != EntityKind::Color {
                return bad(format!("color row {k} is mis-keyed"));
            }
            minted.push(*k);
        }
        for (k, r) in &self.groups {
            if r.id != *k || k.kind() != EntityKind::Group {
                return bad(format!("group row {k} is mis-keyed"));
            }
            minted.push(*k);
        }
        for (k, r) in &self.boundaries {
            if r.id != *k || k.kind() != EntityKind::Boundary {
                return bad(format!("boundary row {k} is mis-keyed"));
            }
            minted.push(*k);
        }
        let ins = &self.instancing;
        for (ks, kind) in [(ins.prototypes.keys().collect::<Vec<_>>(), EntityKind::Prototype), (ins.lattices.keys().collect(), EntityKind::Lattice), (ins.instances.keys().collect(), EntityKind::Instance), (self.decomposition.cells.keys().collect(), EntityKind::PartitionCell)] {
            for k in ks {
                if k.kind() != kind {
                    return bad(format!("{k} is stored under the wrong kind"));
                }
                minted.push(*k);
            }
        }
        if let Some(k) = minted.iter().find(|k| self.ids.peek(k.kind()) <= k.serial()) {
            return bad(format!("the id allocator would reuse {k}"));
        }
        for r in self.boundaries.values() {
            if let Some(d) = r.kind.as_deck() {
                d.check()?;
            }
        }
        for l in ins.lattices.values() {
            l.layout.check()?;
            if let Some(ix) = l.slots.keys().find(|ix| !l.layout.contains(**ix)) {
                return bad(format!("slot {ix:?} is outside its layout"));
            }
        }
        if let Some(l) = ins.find_cycle() {
            return Err(ModelError::Cycle(l));
        }
        self.decomposition.check_integrity()
    }

    /// The id allocator state (read only).
    pub fn ids(&self) -> &IdAlloc {
        &self.ids
    }

    /// Re-key every table through a topology lineage (successions of retired entities): colors,
    /// groups, boundary membership and deck mates, symmetry domain, prototype volumes, recolor
    /// maps. Chains of successions (split, then delete) are resolved transitively, so a color
    /// never lands on a dead child. A split deck mate re-pairs its children by matching face
    /// moments under the deck transform. Entities with no live descendant vanish from the tables.
    /// Transactional: on error the model is unchanged.
    ///
    /// Volumes with an active decomposition, or whose faces are listed by a piece, must be
    /// undone first ([`ModelError::DecompositionActive`]).
    pub fn apply_lineage(&mut self, lineage: &[Lineage], t: &dyn TopologyView) -> Result<()> {
        let r = Resolver::new(lineage);
        if r.is_empty() {
            return Ok(());
        }
        if let Some(v) = self.decomposition.decomposed_volumes().into_iter().find(|v| r.changed(*v)) {
            return Err(ModelError::DecompositionActive(v));
        }
        // A face of a decomposed volume may change while the volume id stays: the piece face
        // lists would go stale silently.
        if let Some((v, _)) = self.decomposition.find_stale_face(|f| r.changed(f)) {
            return Err(ModelError::DecompositionActive(v));
        }
        let mut next = self.clone();
        next.volume_color = r.map(&self.volume_color, true)?;
        next.face_color = r.map(&self.face_color, false)?;
        for g in next.groups.values_mut() {
            g.members = g.members.iter().flat_map(|m| if matches!(m.kind(), EntityKind::TopoVolume | EntityKind::TopoFace | EntityKind::TopoBody) { r.resolve(*m) } else { vec![*m] }).collect();
        }
        if let Some(s) = &mut next.symmetry {
            s.domain = r.set(&s.domain);
        }
        for p in next.instancing.prototypes.values_mut() {
            p.volumes = r.set(&p.volumes);
        }
        for i in next.instancing.instances.values_mut() {
            i.recolor = r.color_map(&i.recolor);
        }
        for l in next.instancing.lattices.values_mut() {
            for s in l.slots.values_mut() {
                s.recolor = r.color_map(&s.recolor);
            }
        }
        for o in &mut next.instancing.overrides {
            o.recolor = r.color_map(&o.recolor);
        }
        next.rekey_boundaries(&r, t)?;
        *self = next;
        Ok(())
    }

    fn rekey_boundaries(&mut self, r: &Resolver, t: &dyn TopologyView) -> Result<()> {
        let mut out: BTreeMap<EntityId, FaceBoundary> = BTreeMap::new();
        let mut touched: BTreeSet<EntityId> = BTreeSet::new();
        for (f, fb) in &self.face_boundary {
            let unchanged_pair = match fb.role {
                FaceRole::Deck { mate: Some(m), .. } => !r.changed(*f) && !r.changed(m),
                _ => true,
            };
            let role = if unchanged_pair {
                fb.role
            } else {
                touched.insert(fb.boundary);
                match fb.role {
                    FaceRole::Deck { side, .. } => FaceRole::Deck { side, mate: None },
                    other => other,
                }
            };
            for nf in r.resolve(*f) {
                match out.get(&nf) {
                    Some(prev) if prev.boundary != fb.boundary || std::mem::discriminant(&prev.role) != std::mem::discriminant(&role) => {
                        return Err(ModelError::CutInvalid(format!("face {nf} would merge faces of different boundary records")));
                    }
                    _ => {
                        out.insert(nf, FaceBoundary { boundary: fb.boundary, role });
                    }
                }
            }
        }
        if !touched.is_empty() {
            let tol = Tolerance::of(t);
            let mut cache = GeomCache::new(t);
            for b in touched {
                let Some(deck) = self.boundaries.get(&b).and_then(|row| match row.kind {
                    BoundaryKind::Deck(d) => Some(d),
                    _ => None,
                }) else {
                    continue;
                };
                let side = |s| -> Vec<EntityId> { out.iter().filter(|(_, fb)| fb.boundary == b && matches!(fb.role, FaceRole::Deck { side, .. } if side == s)).map(|(f, _)| *f).collect() };
                let (a_faces, b_faces) = (side(crate::boundary::DeckSide::A), side(crate::boundary::DeckSide::B));
                let pairs = pair_by_moments(&mut cache, &a_faces, &b_faces, &deck.transform(), &tol).map_err(|e| ModelError::CutInvalid(format!("deck {b}: {e}; the cut removed or changed the partner of a paired face (a deck pair must be cut symmetrically, or unassigned first)")))?;
                for (x, y) in pairs {
                    out.insert(x, FaceBoundary { boundary: b, role: FaceRole::Deck { side: crate::boundary::DeckSide::A, mate: Some(y) } });
                    out.insert(y, FaceBoundary { boundary: b, role: FaceRole::Deck { side: crate::boundary::DeckSide::B, mate: Some(x) } });
                }
            }
        }
        self.face_boundary = out;
        Ok(())
    }

    /// Order-independent fingerprint of the coloring (volume and face colors), used to prove that
    /// decomposition does not change it.
    pub(crate) fn coloring_fingerprint(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |x: u64| {
            for b in x.to_le_bytes() {
                h ^= u64::from(b);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for (k, v) in &self.volume_color {
            eat(k.to_raw());
            eat(v.to_raw());
        }
        eat(u64::MAX);
        for (k, v) in &self.face_color {
            eat(k.to_raw());
            eat(v.to_raw());
        }
        h
    }
}
