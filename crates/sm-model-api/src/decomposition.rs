//! Decomposition: virtual, reversible split of the colored model into meshable pieces.
//!
//! A [`Piece`] (`PartitionCell`) is a set of [`Part`]s; a part is a volume, optionally restricted
//! to one side of a recorded virtual cut. Cuts are *recorded, not applied*: no topology changes.
//! Pieces never store colors. A piece's colors are derived from its volumes, so decomposition
//! cannot change coloring (the validator also checks a coloring fingerprint).
//!
//! Cuts form a binary tree per volume: a part's path is the list of `(cutter, side)` choices
//! from the whole volume down. The leaves of that tree must tile the volume exactly.

use crate::action::Plane;
use crate::error::ModelError;
use crate::selection::{Face, Selection};
use crate::topology::TopologyView;
use sm_core::{EntityId, EntityKind};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Side of a cutter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// Behind the cutter (negative side of a plane).
    Neg,
    /// In front of the cutter.
    Pos,
}

impl Side {
    fn other(self) -> Self {
        match self {
            Side::Neg => Side::Pos,
            Side::Pos => Side::Neg,
        }
    }
}

/// The cutting surface of a virtual cut.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CutSurface {
    /// A plane.
    Plane(Plane),
    /// An existing model surface (`Surface` id), e.g. an imprint or a sweep level surface.
    Surface(EntityId),
}

/// A recorded virtual cut. Faces of the cut volume that lie wholly on one side are listed;
/// every other face is taken to straddle the cutter and belongs to both sides.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Cutter {
    /// The cutting surface.
    pub surface: CutSurface,
    /// Faces that belong only to the negative side (e.g. a sweep source).
    pub neg_only: Selection<Face>,
    /// Faces that belong only to the positive side (e.g. a sweep target).
    pub pos_only: Selection<Face>,
}

/// A volume restricted by a cut path.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Part {
    /// The volume.
    pub volume: EntityId,
    /// `(cutter index, side)` choices from the whole volume. Empty: the whole volume.
    pub path: SmallVec<[(u32, Side); 4]>,
}

/// Hints for the sweep scheme (Cubit source/target).
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SweepHint {
    /// Source faces.
    pub source: Selection<Face>,
    /// Target faces.
    pub target: Selection<Face>,
}

/// A meshable piece.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Piece {
    /// Id (`PartitionCell`).
    pub id: EntityId,
    /// Parts, sorted. More than one part makes this a composite.
    pub parts: Vec<Part>,
    /// Optional sweep hints.
    pub hint: SweepHint,
}

/// A face of a piece: either an original model face or a virtual cut face shared with a neighbour.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PieceFace {
    /// A model face.
    Model(EntityId),
    /// A virtual face where `cutter` separates this piece from `other`.
    Cut {
        /// Cutter index.
        cutter: u32,
        /// The volume being cut.
        volume: EntityId,
        /// The piece on the other side.
        other: EntityId,
    },
}

/// The decomposition of a model into pieces.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Decomposition {
    pieces: BTreeMap<EntityId, Piece>,
    cutters: Vec<Cutter>,
    next: u64,
    /// Fingerprint of the coloring when the decomposition was last built or edited.
    pub coloring_fingerprint: u64,
}

impl Decomposition {
    /// One piece per volume (ascending), recording the coloring fingerprint.
    pub fn trivial(volumes: &[EntityId], coloring_fingerprint: u64) -> Result<Self, ModelError> {
        let mut d = Self { pieces: BTreeMap::new(), cutters: Vec::new(), next: 0, coloring_fingerprint };
        for v in volumes {
            if v.kind() != EntityKind::TopoVolume {
                return Err(ModelError::KindMismatch { expected: EntityKind::TopoVolume, found: *v });
            }
            let part = Part { volume: *v, path: SmallVec::new() };
            d.insert_piece(vec![part], SweepHint::default())?;
        }
        Ok(d)
    }

    fn insert_piece(&mut self, mut parts: Vec<Part>, hint: SweepHint) -> Result<EntityId, ModelError> {
        self.next += 1;
        let id = EntityId::new(EntityKind::PartitionCell, self.next).ok_or(ModelError::SerialOverflow(EntityKind::PartitionCell))?;
        parts.sort();
        self.pieces.insert(id, Piece { id, parts, hint });
        Ok(id)
    }

    /// Pieces, ascending by id.
    pub fn pieces(&self) -> impl Iterator<Item = &Piece> {
        self.pieces.values()
    }
    /// Lookup a piece.
    pub fn piece(&self, id: EntityId) -> Option<&Piece> {
        self.pieces.get(&id)
    }
    /// Recorded cutters.
    pub fn cutters(&self) -> &[Cutter] {
        &self.cutters
    }

    /// Split a single-part piece by a new virtual cut. Returns `(neg, pos)` piece ids. The old
    /// piece is removed; [`Self::unsplit`] reverses this.
    pub fn split_piece(&mut self, piece: EntityId, cutter: Cutter) -> Result<(EntityId, EntityId), ModelError> {
        let old = self.pieces.get(&piece).ok_or(ModelError::UnknownEntity(piece))?.clone();
        let [part] = old.parts.as_slice() else {
            return Err(ModelError::Decomposition("only single-part pieces can be split".into()));
        };
        let idx = self.cutters.len() as u32;
        let mk = |side| {
            let mut p = part.clone();
            p.path.push((idx, side));
            p
        };
        let (pn, pp) = (mk(Side::Neg), mk(Side::Pos));
        self.cutters.push(cutter);
        self.pieces.remove(&piece);
        let neg = self.insert_piece(vec![pn], SweepHint { source: old.hint.source.clone(), target: Selection::empty() })?;
        let pos = self.insert_piece(vec![pp], SweepHint { source: Selection::empty(), target: old.hint.target })?;
        Ok((neg, pos))
    }

    /// Undo the most recent split of `a` and `b`, which must be the two leaves of the last cutter.
    pub fn unsplit(&mut self, a: EntityId, b: EntityId) -> Result<EntityId, ModelError> {
        let bad = || ModelError::Decomposition("pieces are not the leaves of the last cut".into());
        let last = (self.cutters.len() as u32).checked_sub(1).ok_or_else(bad)?;
        let single = |d: &Self, id: EntityId| -> Result<Part, ModelError> {
            let p = d.pieces.get(&id).ok_or(ModelError::UnknownEntity(id))?;
            match p.parts.as_slice() {
                [one] => Ok(one.clone()),
                _ => Err(ModelError::Decomposition("not a split leaf".into())),
            }
        };
        let (pa, pb) = (single(self, a)?, single(self, b)?);
        let (neg, pos, mut merged) = match (pa.path.last(), pb.path.last()) {
            (Some(&(c1, Side::Neg)), Some(&(c2, Side::Pos))) if c1 == last && c2 == last => (a, b, pa.clone()),
            (Some(&(c1, Side::Pos)), Some(&(c2, Side::Neg))) if c1 == last && c2 == last => (b, a, pb.clone()),
            _ => return Err(bad()),
        };
        if pa.volume != pb.volume || pa.path[..pa.path.len() - 1] != pb.path[..pb.path.len() - 1] {
            return Err(bad());
        }
        merged.path.pop();
        let hint = SweepHint { source: self.pieces[&neg].hint.source.clone(), target: self.pieces[&pos].hint.target.clone() };
        self.pieces.remove(&a);
        self.pieces.remove(&b);
        self.cutters.pop();
        self.insert_piece(vec![merged], hint)
    }

    /// Merge whole-volume pieces into one composite piece (a mappable chunk spanning volumes).
    pub fn composite(&mut self, pieces: &[EntityId]) -> Result<EntityId, ModelError> {
        let mut parts = Vec::new();
        for id in pieces {
            let p = self.pieces.get(id).ok_or(ModelError::UnknownEntity(*id))?;
            if p.parts.iter().any(|x| !x.path.is_empty()) {
                return Err(ModelError::Decomposition("cannot composite a cut piece".into()));
            }
            parts.extend(p.parts.iter().cloned());
        }
        for id in pieces {
            self.pieces.remove(id);
        }
        self.insert_piece(parts, SweepHint::default())
    }

    /// Attach sweep hints to a piece.
    pub fn set_hint(&mut self, piece: EntityId, hint: SweepHint) -> Result<(), ModelError> {
        self.pieces.get_mut(&piece).ok_or(ModelError::UnknownEntity(piece))?.hint = hint;
        Ok(())
    }

    // ---- queries ----

    /// Pieces that contain (part of) `volume`, ascending.
    pub fn pieces_of_volume(&self, volume: EntityId) -> Vec<EntityId> {
        self.pieces.values().filter(|p| p.parts.iter().any(|x| x.volume == volume)).map(|p| p.id).collect()
    }

    /// Volumes touched by `piece`, ascending, deduplicated.
    pub fn volumes_of_piece(&self, piece: EntityId) -> Vec<EntityId> {
        let set: BTreeSet<EntityId> = self.pieces.get(&piece).into_iter().flat_map(|p| p.parts.iter().map(|x| x.volume)).collect();
        set.into_iter().collect()
    }

    fn part_faces(&self, part: &Part, topo: &dyn TopologyView) -> Vec<EntityId> {
        topo.volume_faces(part.volume)
            .into_iter()
            .filter(|f| {
                part.path.iter().all(|(c, side)| {
                    let cut = &self.cutters[*c as usize];
                    match side {
                        Side::Neg => !cut.pos_only.contains(*f),
                        Side::Pos => !cut.neg_only.contains(*f),
                    }
                })
            })
            .collect()
    }

    /// Faces of a piece: model faces (ascending) then virtual cut faces (sorted).
    pub fn faces_of_piece(&self, piece: EntityId, topo: &dyn TopologyView) -> Vec<PieceFace> {
        let Some(p) = self.pieces.get(&piece) else { return Vec::new() };
        let mut model: BTreeSet<EntityId> = BTreeSet::new();
        let mut cuts: BTreeSet<PieceFace> = BTreeSet::new();
        for part in &p.parts {
            model.extend(self.part_faces(part, topo));
            for (i, (c, side)) in part.path.iter().enumerate() {
                for q in self.pieces.values().filter(|q| q.id != piece) {
                    for qp in q.parts.iter().filter(|qp| qp.volume == part.volume) {
                        if qp.path.len() > i && qp.path[..i] == part.path[..i] && qp.path[i] == (*c, side.other()) {
                            cuts.insert(PieceFace::Cut { cutter: *c, volume: part.volume, other: q.id });
                        }
                    }
                }
            }
        }
        model.into_iter().map(PieceFace::Model).chain(cuts).collect()
    }

    /// Pieces that include model face `face`, ascending.
    pub fn pieces_of_face(&self, face: EntityId, topo: &dyn TopologyView) -> Vec<EntityId> {
        self.pieces
            .values()
            .filter(|p| p.parts.iter().any(|part| self.part_faces(part, topo).contains(&face)))
            .map(|p| p.id)
            .collect()
    }

    /// Check that, for every volume in `volumes`, the pieces tile it exactly. Returns one message
    /// per defect, ordered by volume.
    pub fn check_tiling(&self, volumes: &[EntityId]) -> Vec<(EntityId, String)> {
        let mut by_vol: BTreeMap<EntityId, Vec<&[(u32, Side)]>> = BTreeMap::new();
        for p in self.pieces.values() {
            for part in &p.parts {
                by_vol.entry(part.volume).or_default().push(part.path.as_slice());
            }
        }
        let mut out = Vec::new();
        for v in volumes {
            match by_vol.get(v) {
                None => out.push((*v, "volume is not covered by any piece".to_owned())),
                Some(paths) => {
                    if let Err(e) = tile(paths, 0) {
                        out.push((*v, e));
                    }
                }
            }
        }
        for v in by_vol.keys().filter(|v| !volumes.contains(v)) {
            out.push((*v, "piece refers to a volume outside the domain".to_owned()));
        }
        out.sort();
        out
    }
}

fn tile(paths: &[&[(u32, Side)]], depth: usize) -> Result<(), String> {
    if paths.iter().any(|p| p.len() == depth) {
        return if paths.len() == 1 { Ok(()) } else { Err("a whole region overlaps other parts".into()) };
    }
    let cutter = paths[0][depth].0;
    if paths.iter().any(|p| p[depth].0 != cutter) {
        return Err("parts disagree on the cutter at the same level".into());
    }
    for side in [Side::Neg, Side::Pos] {
        let sub: Vec<&[(u32, Side)]> = paths.iter().copied().filter(|p| p[depth].1 == side).collect();
        if sub.is_empty() {
            return Err(format!("side {side:?} of cutter {cutter} is not covered"));
        }
        tile(&sub, depth + 1)?;
    }
    Ok(())
}
