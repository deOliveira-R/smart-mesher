//! Decomposition: a virtual, reversible split of the colored model into meshable **pieces**
//! (`EntityKind::PartitionCell`).
//!
//! Each volume has an implicit binary cut tree. A volume with no cuts is implicitly one piece,
//! [`PieceRef::Whole`] (a deterministic virtual id, nothing minted). [`Model::split`] cuts a leaf
//! in two and mints two `PartitionCell` ids; the parent stays in the tree, inactive, so
//! [`Model::unsplit`] restores it with its original id (ir.md §6, as for refinement). Ids are
//! never reused: re-splitting mints new ones. The binary tree tiles the volume by construction.
//!
//! A piece has **no color of its own**: it reads through its volume, so a decomposition cannot
//! change the coloring. Cut faces are local [`CutId`] handles (this layer mints no `PartitionFace`
//! ids, which belong to the topology overlay); the part of a model face that straddles a cut is
//! [`PieceFace::Part`], the key the mesh classifier uses.
//!
//! A cut face of a parent that straddles a further split is inherited by both children as
//! [`PieceFace::CutPart`] (distinct keys); [`Model::split_sides`] lets the caller say which child
//! gets a parent cut face whole.

use crate::error::{ModelError, Result, expect_kind};
use crate::model::Model;
use crate::symmetry::Keep;
use crate::xf::Plane;
use sm_core::{EntityId, EntityKind};
use sm_topo_api::TopologyView;
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Reference to a piece: a volume that is not decomposed, or a minted cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceRef {
    /// The whole volume as one implicit piece (virtual id, nothing minted).
    Whole(EntityId),
    /// A `PartitionCell` piece.
    Cell(EntityId),
}

/// The geometry of a virtual cut. There is no "unspecified" cut.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CutGeometry {
    /// A plane.
    Plane(Plane),
    /// An existing `Surface` entity of the shape layer.
    Surface(EntityId),
    /// A set of existing faces that separate the volume.
    Faces(Vec<EntityId>),
}

/// Identifier of a virtual cut. Local to the model; never reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutId(pub u32);

/// A face as seen by a piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceFace {
    /// A model face wholly on this piece's boundary.
    Whole(EntityId),
    /// The part of a model face that straddles a cut and bounds `piece`.
    Part {
        /// The model face.
        face: EntityId,
        /// The piece it bounds.
        piece: EntityId,
    },
    /// A virtual cut face; each cut is shared by a negative and a positive side.
    Cut {
        /// The cut.
        cut: CutId,
        /// Which side of the cut this piece is on.
        side: Keep,
    },
    /// The part of a cut face that straddles a later cut and bounds `piece` (the analogue of
    /// [`PieceFace::Part`] for virtual faces). Two grandchildren that both touch an earlier
    /// cut face carry distinct `CutPart` keys, so the key is never ambiguous.
    CutPart {
        /// The earlier cut.
        cut: CutId,
        /// Which side of that cut the piece is on.
        side: Keep,
        /// The piece it bounds.
        piece: EntityId,
    },
}

impl PieceFace {
    /// The underlying model face, for `Whole` and `Part`.
    pub fn base(&self) -> Option<EntityId> {
        match self {
            PieceFace::Whole(f) | PieceFace::Part { face: f, .. } => Some(*f),
            PieceFace::Cut { .. } | PieceFace::CutPart { .. } => None,
        }
    }
}

/// Meshing-relevant shape class of a piece.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceShape {
    /// No assertion; the mesher decides.
    #[default]
    Any,
    /// Sweepable between source and target faces (which must not share an edge).
    Sweep {
        /// Source faces (meshed first).
        source: Vec<PieceFace>,
        /// Target faces.
        target: Vec<PieceFace>,
    },
    /// Mappable.
    Map,
}

/// Row of the piece table. There is no color field.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Piece {
    /// The `PartitionCell` id.
    pub id: EntityId,
    /// The volume (for a composite, the lowest member volume).
    pub volume: EntityId,
    /// Bounding faces, sorted and unique (empty for a composite: read through its members).
    pub faces: Vec<PieceFace>,
    /// Shape class.
    pub shape: PieceShape,
    /// Members of a composite (empty otherwise).
    pub members: Vec<PieceRef>,
}

/// A recorded split.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SplitRec {
    /// `[negative side, positive side]`, the negative one with the lower id.
    pub children: [EntityId; 2],
    /// The cut.
    pub cut: CutId,
    /// Its geometry.
    pub geometry: CutGeometry,
}

/// The piece tables (part of the [`Model`], serializable).
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Decomposition {
    pub(crate) cells: BTreeMap<EntityId, Piece>,
    #[cfg_attr(feature = "serde", serde(with = "crate::serde_pairs"))]
    pub(crate) splits: BTreeMap<PieceRef, SplitRec>,
    pub(crate) parent: BTreeMap<EntityId, PieceRef>,
    pub(crate) composites: BTreeSet<EntityId>,
    pub(crate) consumed: BTreeSet<PieceRef>,
    pub(crate) by_volume: BTreeMap<EntityId, SmallVec<[PieceRef; 4]>>,
    pub(crate) by_face: BTreeMap<EntityId, SmallVec<[PieceRef; 2]>>,
    pub(crate) cut_serial: u32,
    /// Faces of the `Whole` members of each composite (captured when it is formed).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) whole_faces: BTreeMap<EntityId, Vec<EntityId>>,
}

fn cut_key(pf: &PieceFace) -> Option<(CutId, Keep)> {
    match pf {
        PieceFace::Cut { cut, side } | PieceFace::CutPart { cut, side, .. } => Some((*cut, *side)),
        _ => None,
    }
}

/// How the faces of a piece fall on the two sides of a new cut (see [`Model::split_sides`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SplitSides<'a> {
    /// Model faces bounding the negative part (a face also in `pos` straddles the cut).
    pub neg: &'a [EntityId],
    /// Model faces bounding the positive part.
    pub pos: &'a [EntityId],
    /// Parent cut faces `(cut, side)` bounding the negative part.
    pub neg_cuts: &'a [(CutId, Keep)],
    /// Parent cut faces bounding the positive part.
    pub pos_cuts: &'a [(CutId, Keep)],
}

fn whole_volume(p: &PieceRef) -> Option<EntityId> {
    match p {
        PieceRef::Whole(v) => Some(*v),
        PieceRef::Cell(_) => None,
    }
}

impl Decomposition {
    /// All piece rows ever created and not undone, ascending by id (inactive parents included).
    pub fn cells(&self) -> impl Iterator<Item = &Piece> {
        self.cells.values()
    }

    /// The row of a cell piece.
    pub fn piece(&self, id: EntityId) -> Option<&Piece> {
        self.cells.get(&id)
    }

    /// The split record whose parent is `p`.
    pub fn split_of(&self, p: PieceRef) -> Option<&SplitRec> {
        self.splits.get(&p)
    }

    /// The parent a cell was split from.
    pub fn parent_of(&self, id: EntityId) -> Option<PieceRef> {
        self.parent.get(&id).copied()
    }

    /// The active cell pieces (leaves not covered by a composite, and composites), ascending.
    pub fn active_cells(&self) -> Vec<EntityId> {
        self.cells.keys().copied().filter(|c| self.is_active(PieceRef::Cell(*c))).collect()
    }

    /// True if `p` is an active piece: a leaf of the cut tree not consumed by a composite, or a
    /// composite.
    pub fn is_active(&self, p: PieceRef) -> bool {
        match p {
            PieceRef::Whole(_) => !self.splits.contains_key(&p) && !self.consumed.contains(&p),
            PieceRef::Cell(c) => self.cells.contains_key(&c) && (self.composites.contains(&c) || (!self.splits.contains_key(&p) && !self.consumed.contains(&p))),
        }
    }

    /// The first `(volume, face)` for which a piece lists a model face that `changed` reports
    /// as changed (used to refuse topology edits that would leave piece face lists stale).
    pub(crate) fn find_stale_face(&self, changed: impl Fn(EntityId) -> bool) -> Option<(EntityId, EntityId)> {
        for c in self.cells.values() {
            let own = c.faces.iter().filter_map(PieceFace::base);
            let whole = self.whole_faces.get(&c.id).into_iter().flatten().copied();
            if let Some(f) = own.chain(whole).find(|f| changed(*f)) {
                return Some((c.volume, f));
            }
        }
        None
    }

    /// Volumes that have any decomposition structure.
    pub fn decomposed_volumes(&self) -> BTreeSet<EntityId> {
        self.splits.keys().chain(self.consumed.iter()).filter_map(whole_volume).collect()
    }

    /// The faces of any piece (cells: stored; `Whole`: every face of the volume).
    pub fn piece_faces(&self, p: PieceRef, t: &dyn TopologyView) -> Vec<PieceFace> {
        match p {
            PieceRef::Whole(v) => t.volume_faces(v).iter().map(|f| PieceFace::Whole(*f)).collect(),
            PieceRef::Cell(c) => match self.cells.get(&c) {
                Some(pc) if !pc.members.is_empty() => {
                    let mut v: Vec<PieceFace> = pc.members.iter().flat_map(|m| self.piece_faces(*m, t)).collect();
                    // A cut seen from both sides, or a model face held whole by two members, is
                    // internal to the composite.
                    let mut sides: BTreeMap<CutId, [bool; 2]> = BTreeMap::new();
                    let mut wholes: BTreeMap<EntityId, u32> = BTreeMap::new();
                    for f in &v {
                        if let Some((c, s)) = cut_key(f) {
                            sides.entry(c).or_default()[usize::from(s == Keep::Positive)] = true;
                        } else if let PieceFace::Whole(m) = f {
                            *wholes.entry(*m).or_default() += 1;
                        }
                    }
                    v.retain(|f| match f {
                        PieceFace::Whole(m) => wholes.get(m).copied().unwrap_or(0) < 2,
                        _ => cut_key(f).is_none_or(|(c, _)| sides.get(&c).is_none_or(|b| !(b[0] && b[1]))),
                    });
                    v.sort();
                    v.dedup();
                    v
                }
                Some(pc) => pc.faces.clone(),
                None => Vec::new(),
            },
        }
    }

    /// Leaves of the cut tree below `p`. Iterative with a visited set, so a cyclic tree from
    /// untrusted data cannot recurse without bound (it is reported by `check_integrity`).
    fn leaves(&self, p: PieceRef, out: &mut Vec<PieceRef>) {
        let mut seen: BTreeSet<PieceRef> = BTreeSet::new();
        let mut stack = vec![p];
        while let Some(q) = stack.pop() {
            if !seen.insert(q) {
                continue;
            }
            match self.splits.get(&q) {
                None => out.push(q),
                // Children pushed in reverse so the negative child is visited first.
                Some(rec) => stack.extend(rec.children.iter().rev().map(|c| PieceRef::Cell(*c))),
            }
        }
    }

    /// Structural checks for data that did not come through the mutators (deserialization):
    /// every split has two existing children that point back to it, the cut trees are acyclic,
    /// and composites refer to existing members.
    pub(crate) fn check_integrity(&self) -> Result<()> {
        let bad = |what: String| Err(ModelError::Invalid(format!("decomposition: {what}")));
        for (parent, rec) in &self.splits {
            for c in rec.children {
                if !self.cells.contains_key(&c) {
                    return bad(format!("split child {c} has no row"));
                }
                if self.parent.get(&c) != Some(parent) {
                    return bad(format!("split child {c} does not point back to its parent"));
                }
                if PieceRef::Cell(c) == *parent {
                    return bad(format!("piece {c} is its own child"));
                }
            }
        }
        // Cut ids are never reused: the serial must exceed every id in use.
        let used = self.splits.values().map(|r| r.cut.0).chain(self.cells.values().flat_map(|p| p.faces.iter().filter_map(cut_key).map(|(c, _)| c.0))).max();
        if let Some(m) = used
            && self.cut_serial <= m
        {
            return bad(format!("cut serial {} does not exceed the cut id {m} in use", self.cut_serial));
        }
        for (c, p) in &self.parent {
            // Walking up must reach a root within `cells.len()` steps.
            let mut cur = *p;
            for _ in 0..=self.cells.len() {
                match cur {
                    PieceRef::Cell(x) if x == *c => return bad(format!("cut tree has a cycle through {c}")),
                    PieceRef::Cell(x) => match self.parent.get(&x) {
                        Some(up) => cur = *up,
                        None => break,
                    },
                    PieceRef::Whole(_) => break,
                }
            }
        }
        for c in &self.composites {
            let Some(row) = self.cells.get(c) else { return bad(format!("composite {c} has no row")) };
            for m in &row.members {
                if let PieceRef::Cell(x) = m
                    && (!self.cells.contains_key(x) || self.composites.contains(x))
                {
                    return bad(format!("composite {c} has an unknown or composite member {x}"));
                }
            }
        }
        Ok(())
    }

    fn volume_of(&self, p: PieceRef) -> Option<EntityId> {
        match p {
            PieceRef::Whole(v) => Some(v),
            PieceRef::Cell(c) => self.cells.get(&c).map(|x| x.volume),
        }
    }

    fn member_volumes(&self, c: EntityId) -> BTreeSet<EntityId> {
        self.cells.get(&c).map(|p| p.members.iter().filter_map(|m| self.volume_of(*m)).collect()).unwrap_or_default()
    }

    fn reindex(&mut self) {
        let mut vols: BTreeSet<EntityId> = self.decomposed_volumes();
        let comp_vols: BTreeMap<EntityId, BTreeSet<EntityId>> = self.composites.iter().map(|c| (*c, self.member_volumes(*c))).collect();
        for s in comp_vols.values() {
            vols.extend(s.iter().copied());
        }
        let mut by_volume = BTreeMap::new();
        for v in vols {
            let mut ls = Vec::new();
            self.leaves(PieceRef::Whole(v), &mut ls);
            let mut list: SmallVec<[PieceRef; 4]> = ls.into_iter().filter(|l| !self.consumed.contains(l)).collect();
            list.extend(comp_vols.iter().filter(|(_, s)| s.contains(&v)).map(|(c, _)| PieceRef::Cell(*c)));
            list.sort();
            list.dedup();
            by_volume.insert(v, list);
        }
        let mut by_face: BTreeMap<EntityId, SmallVec<[PieceRef; 2]>> = BTreeMap::new();
        for c in self.active_cells() {
            let faces: Vec<PieceFace> = match self.cells.get(&c) {
                Some(p) if !p.members.is_empty() => p
                    .members
                    .iter()
                    .filter_map(|m| if let PieceRef::Cell(x) = m { self.cells.get(x) } else { None })
                    .flat_map(|m| m.faces.iter().copied())
                    .chain(self.whole_faces.get(&c).into_iter().flatten().map(|f| PieceFace::Whole(*f)))
                    .collect(),
                Some(p) => p.faces.clone(),
                None => continue,
            };
            for f in faces.iter().filter_map(PieceFace::base) {
                let e = by_face.entry(f).or_default();
                if !e.contains(&PieceRef::Cell(c)) {
                    e.push(PieceRef::Cell(c));
                }
            }
        }
        self.by_volume = by_volume;
        self.by_face = by_face;
    }
}

impl Model {
    /// The decomposition tables (read only).
    pub fn decomposition(&self) -> &Decomposition {
        &self.decomposition
    }

    /// Split the active piece `piece` in two along `cut`. `neg` and `pos` are the model faces
    /// that bound the negative and positive part; a face in both straddles the cut (each child
    /// gets a [`PieceFace::Part`]). Every model face of the piece must be in at least one of them.
    /// Cut faces of the parent are assumed to straddle the new cut (both children get a
    /// [`PieceFace::CutPart`]); use [`Model::split_sides`] to classify them. Returns
    /// `[negative, positive]` ids. Colors are untouched by construction.
    pub fn split(&mut self, piece: PieceRef, cut: CutGeometry, neg: &[EntityId], pos: &[EntityId], t: &dyn TopologyView) -> Result<[EntityId; 2]> {
        let parent_cuts: Vec<(CutId, Keep)> = self.decomposition.piece_faces(piece, t).iter().filter_map(cut_key).collect();
        self.split_sides(piece, cut, &SplitSides { neg, pos, neg_cuts: &parent_cuts, pos_cuts: &parent_cuts }, t)
    }

    /// Like [`Model::split`], also classifying the parent's cut faces: a `(cut, side)` listed only
    /// in `neg_cuts` bounds the negative child alone, one listed in both straddles the new cut
    /// (each child gets a [`PieceFace::CutPart`]). Every cut face of the parent must be listed.
    pub fn split_sides(&mut self, piece: PieceRef, cut: CutGeometry, sides: &SplitSides<'_>, t: &dyn TopologyView) -> Result<[EntityId; 2]> {
        let (neg, pos) = (sides.neg, sides.pos);
        let d = &self.decomposition;
        if !d.is_active(piece) {
            return Err(ModelError::Invalid("only an active, non-composite piece can be split".into()));
        }
        if let PieceRef::Cell(c) = piece
            && d.composites.contains(&c)
        {
            return Err(ModelError::Invalid("a composite cannot be split; uncomposite it first".into()));
        }
        let volume = d.volume_of(piece).ok_or(ModelError::Invalid("unknown piece".into()))?;
        match &cut {
            CutGeometry::Surface(s) => expect_kind(*s, EntityKind::Surface)?,
            CutGeometry::Faces(fs) => fs.iter().try_for_each(|f| expect_kind(*f, EntityKind::TopoFace))?,
            CutGeometry::Plane(_) => {}
        }
        let parent_faces = d.piece_faces(piece, t);
        let bases: BTreeSet<EntityId> = parent_faces.iter().filter_map(PieceFace::base).collect();
        for f in neg.iter().chain(pos) {
            if !bases.contains(f) {
                return Err(ModelError::Invalid(format!("face {f} does not bound the piece")));
            }
        }
        if let Some(f) = bases.iter().find(|f| !neg.contains(f) && !pos.contains(f)) {
            return Err(ModelError::Invalid(format!("face {f} of the piece is on neither side of the cut")));
        }
        let parent_cuts: BTreeSet<(CutId, Keep)> = parent_faces.iter().filter_map(cut_key).collect();
        if let Some(k) = sides.neg_cuts.iter().chain(sides.pos_cuts).find(|k| !parent_cuts.contains(k)) {
            return Err(ModelError::Invalid(format!("{k:?} is not a cut face of the piece")));
        }
        if let Some(k) = parent_cuts.iter().find(|k| !sides.neg_cuts.contains(k) && !sides.pos_cuts.contains(k)) {
            return Err(ModelError::Invalid(format!("cut face {k:?} of the piece is on neither side of the new cut")));
        }
        let before = self.coloring_fingerprint();
        let next_serial = d.cut_serial.checked_add(1).ok_or_else(|| ModelError::Invalid("cut id space exhausted".into()))?;
        // Allocate both ids from a copy so a failure leaves the allocator untouched.
        let mut ids = self.ids.clone();
        let n_id = ids.alloc(EntityKind::PartitionCell)?;
        let p_id = ids.alloc(EntityKind::PartitionCell)?;
        self.ids = ids;
        let d = &mut self.decomposition;
        let cut_id = CutId(d.cut_serial);
        d.cut_serial = next_serial;
        let mk = |child: EntityId, mine: &[EntityId], other: &[EntityId], mine_cuts: &[(CutId, Keep)], other_cuts: &[(CutId, Keep)], side: Keep| {
            let mut faces: Vec<PieceFace> = Vec::new();
            for pf in &parent_faces {
                match pf {
                    PieceFace::Cut { cut, side } | PieceFace::CutPart { cut, side, .. } => {
                        let key = (*cut, *side);
                        if mine_cuts.contains(&key) {
                            let straddles = other_cuts.contains(&key) || matches!(pf, PieceFace::CutPart { .. });
                            faces.push(if straddles { PieceFace::CutPart { cut: *cut, side: *side, piece: child } } else { *pf });
                        }
                    }
                    PieceFace::Whole(f) | PieceFace::Part { face: f, .. } => {
                        if mine.contains(f) {
                            let straddles = other.contains(f) || matches!(pf, PieceFace::Part { .. });
                            faces.push(if straddles { PieceFace::Part { face: *f, piece: child } } else { PieceFace::Whole(*f) });
                        }
                    }
                }
            }
            faces.push(PieceFace::Cut { cut: cut_id, side });
            faces.sort();
            faces.dedup();
            Piece { id: child, volume, faces, shape: PieceShape::Any, members: Vec::new() }
        };
        let np = mk(n_id, neg, pos, sides.neg_cuts, sides.pos_cuts, Keep::Negative);
        let pp = mk(p_id, pos, neg, sides.pos_cuts, sides.neg_cuts, Keep::Positive);
        d.cells.insert(n_id, np);
        d.cells.insert(p_id, pp);
        d.splits.insert(piece, SplitRec { children: [n_id, p_id], cut: cut_id, geometry: cut });
        d.parent.insert(n_id, piece);
        d.parent.insert(p_id, piece);
        d.reindex();
        debug_assert_eq!(before, self.coloring_fingerprint());
        Ok([n_id, p_id])
    }

    /// Merge active, same-colored pieces into one composite piece (internal cuts vanish). The
    /// members stay in the tree but are covered by the composite. `t` supplies the faces of
    /// members that are whole volumes, so [`Model::pieces_of_face`] finds the composite.
    pub fn composite(&mut self, pieces: &[PieceRef], t: &dyn TopologyView) -> Result<EntityId> {
        if pieces.len() < 2 {
            return Err(ModelError::Invalid("a composite needs at least two pieces".into()));
        }
        let mut color = None;
        for (i, p) in pieces.iter().enumerate() {
            if pieces[..i].contains(p) {
                return Err(ModelError::Invalid("duplicate piece in composite".into()));
            }
            let d = &self.decomposition;
            if !d.is_active(*p) || matches!(p, PieceRef::Cell(c) if d.composites.contains(c)) {
                return Err(ModelError::Invalid("composite members must be active non-composite pieces".into()));
            }
            let c = self.piece_color(*p).ok_or_else(|| ModelError::Invalid("composite members must be colored".into()))?;
            if *color.get_or_insert(c) != c {
                return Err(ModelError::Invalid("a composite cannot mix colors".into()));
            }
        }
        let volume = pieces.iter().filter_map(|p| self.decomposition.volume_of(*p)).min().ok_or(ModelError::Invalid("unknown piece".into()))?;
        let id = self.ids.alloc(EntityKind::PartitionCell)?;
        let d = &mut self.decomposition;
        let mut wf: Vec<EntityId> = pieces.iter().filter_map(whole_volume).flat_map(|v| t.volume_faces(v).iter().copied()).collect();
        wf.sort();
        wf.dedup();
        if !wf.is_empty() {
            d.whole_faces.insert(id, wf);
        }
        d.cells.insert(id, Piece { id, volume, faces: Vec::new(), shape: PieceShape::Any, members: pieces.to_vec() });
        d.composites.insert(id);
        d.consumed.extend(pieces.iter().copied());
        d.reindex();
        Ok(id)
    }

    /// Dissolve a composite; its members become active again.
    pub fn uncomposite(&mut self, composite: EntityId) -> Result<()> {
        let d = &mut self.decomposition;
        if !d.composites.remove(&composite) {
            return Err(ModelError::Unknown(composite));
        }
        d.whole_faces.remove(&composite);
        if let Some(p) = d.cells.remove(&composite) {
            for m in p.members {
                d.consumed.remove(&m);
            }
        }
        d.reindex();
        Ok(())
    }

    /// Set the shape class of a cell piece.
    pub fn set_shape(&mut self, piece: EntityId, s: PieceShape) -> Result<()> {
        expect_kind(piece, EntityKind::PartitionCell)?;
        self.decomposition.cells.get_mut(&piece).ok_or(ModelError::Unknown(piece))?.shape = s;
        Ok(())
    }

    /// Undo the split that produced `piece` (a child cell): both children must still be leaves.
    /// Restores and returns the parent with its **original** id; the children's ids are retired
    /// and never reused.
    pub fn unsplit(&mut self, piece: EntityId) -> Result<PieceRef> {
        let d = &mut self.decomposition;
        let parent = *d.parent.get(&piece).ok_or(ModelError::Unknown(piece))?;
        let rec = d.splits.get(&parent).ok_or(ModelError::Unknown(piece))?.clone();
        for c in rec.children {
            let r = PieceRef::Cell(c);
            if d.splits.contains_key(&r) || d.consumed.contains(&r) {
                return Err(ModelError::Invalid("both children must be unsplit leaves".into()));
            }
        }
        for c in rec.children {
            d.cells.remove(&c);
            d.parent.remove(&c);
        }
        d.splits.remove(&parent);
        d.reindex();
        Ok(parent)
    }

    /// Drop the whole decomposition. Colors and every other table are untouched; piece ids stay
    /// retired.
    pub fn clear_decomposition(&mut self) {
        let serial = self.decomposition.cut_serial;
        self.decomposition = Decomposition { cut_serial: serial, ..Decomposition::default() };
    }

    /// The active pieces covering volume `v`: `[Whole(v)]` when it is not decomposed.
    pub fn pieces_of_volume(&self, v: EntityId) -> SmallVec<[PieceRef; 4]> {
        self.decomposition.by_volume.get(&v).cloned().unwrap_or_else(|| SmallVec::from_slice(&[PieceRef::Whole(v)]))
    }

    /// The active decomposed pieces bounded by model face `f` (indexed), composites of whole
    /// volumes included. Faces of volumes that are not decomposed are not listed; see
    /// [`Model::pieces_of_face_in`].
    pub fn pieces_of_face(&self, f: EntityId) -> SmallVec<[PieceRef; 2]> {
        self.decomposition.by_face.get(&f).cloned().unwrap_or_default()
    }

    /// Like [`Model::pieces_of_face`], adding `Whole(v)` for every adjacent volume that is not
    /// decomposed.
    pub fn pieces_of_face_in(&self, f: EntityId, t: &dyn TopologyView) -> SmallVec<[PieceRef; 2]> {
        use sm_topo_api::{Region, TopologyViewExt};
        let mut out = self.pieces_of_face(f);
        if let Some(s) = t.face_sides(f) {
            for r in [s.left, s.right] {
                if let Region::Volume(v) = r {
                    let w = PieceRef::Whole(v);
                    if self.decomposition.is_active(w) && !out.contains(&w) {
                        out.push(w);
                    }
                }
            }
        }
        out.sort();
        out
    }

    /// The volume a piece belongs to (lowest member volume for a composite).
    pub fn piece_volume(&self, p: PieceRef) -> EntityId {
        match p {
            PieceRef::Whole(v) => v,
            PieceRef::Cell(c) => self.decomposition.cells.get(&c).map_or(c, |x| x.volume),
        }
    }

    /// The color of a piece, read through its volume(s). `None` if uncolored or (for a composite)
    /// mixed.
    pub fn piece_color(&self, p: PieceRef) -> Option<EntityId> {
        match p {
            PieceRef::Whole(v) => self.volume_color.get(&v).copied(),
            PieceRef::Cell(c) => {
                let piece = self.decomposition.cells.get(&c)?;
                if piece.members.is_empty() {
                    return self.volume_color.get(&piece.volume).copied();
                }
                let mut it = piece.members.iter().map(|m| self.piece_color(*m));
                let first = it.next()??;
                it.all(|x| x == Some(first)).then_some(first)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    #[test]
    fn piece_face_base() {
        let f = EntityId::new(EntityKind::TopoFace, 1).unwrap();
        assert_eq!(PieceFace::Whole(f).base(), Some(f));
        assert_eq!(PieceFace::Cut { cut: CutId(0), side: Keep::Negative }.base(), None);
    }
}
