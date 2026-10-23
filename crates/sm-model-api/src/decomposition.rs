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
//!
//! **Realized splits.** [`Model::split_realized`] also realizes a plane cut in the topology
//! overlay (`sm_topo_api::Overlay::partition_volume_by_plane`): the overlay mints the cut face,
//! its edges and vertices as `Partition*` cells, the two overlay volumes take the piece ids, and
//! this layer records (persisted, deterministic, never reused) the `CutId -> overlay ids` mapping
//! ([`CutCells`]), the cells of every non-`Whole` piece face and the faces the cut partitioned.
//! [`Decomposition::piece_face_cells`] resolves any [`PieceFace`] to the cells of the view, so a
//! split piece can be meshed and its mesh classified on the virtual cells.

use crate::error::{ModelError, Result, expect_kind};
use crate::model::Model;
use crate::symmetry::Keep;
use crate::xf::Plane;
use sm_core::{EntityId, EntityKind, Point3};
use sm_topo_api::{Overlay, TopologyView};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Reference to a piece: a volume that is not decomposed, or a minted cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct CutId(pub u32);

/// A face as seen by a piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
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

/// The topology-overlay cells a realized cut created or changed (see [`Model::split_realized`]).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutCells {
    /// The virtual cut face(s) (`PartitionFace`; one for a plane cut).
    pub faces: Vec<EntityId>,
    /// The edges of the cut face's loop, ascending.
    pub edges: Vec<EntityId>,
    /// The vertices of the cut face's loop, ascending.
    pub vertices: Vec<EntityId>,
    /// Faces the cut partitioned, with their halves `[negative side, positive side]`, in the
    /// order the overlay made them.
    pub split_faces: Vec<(EntityId, [EntityId; 2])>,
    /// Edges the cut partitioned: `(edge, new virtual vertex, halves in edge order)`.
    pub split_edges: Vec<(EntityId, EntityId, [EntityId; 2])>,
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
    /// The faces of each composite (captured when it is formed; internal faces removed).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) composite_faces: BTreeMap<EntityId, Vec<PieceFace>>,
    /// Realized cuts: the overlay cells of each cut.
    #[cfg_attr(feature = "serde", serde(default, with = "crate::serde_pairs"))]
    pub(crate) realized: BTreeMap<CutId, CutCells>,
    /// Overlay cells of the `Part` / `CutPart` faces of realized pieces (as recorded at the split;
    /// expanded through `split_cells` when later cuts partition them).
    #[cfg_attr(feature = "serde", serde(default, with = "crate::serde_pairs"))]
    pub(crate) face_cells: BTreeMap<PieceFace, Vec<EntityId>>,
    /// Faces partitioned by realized cuts: face -> `[negative half, positive half]`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) split_cells: BTreeMap<EntityId, [EntityId; 2]>,
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

/// Faces of a composite from the faces of its members, each tagged with the member's volume: a
/// cut seen from both sides, or a model face held (whole or in part) by members of two
/// different volumes (an interface between them), is internal and dropped. Parts of one model
/// face held by members of the same volume stay (they bound the composite).
fn merge_member_faces(items: Vec<(PieceFace, Option<EntityId>)>) -> Vec<PieceFace> {
    let mut sides: BTreeMap<CutId, [bool; 2]> = BTreeMap::new();
    let mut vols: BTreeMap<EntityId, BTreeSet<EntityId>> = BTreeMap::new();
    for (f, vol) in &items {
        if let Some((c, s)) = cut_key(f) {
            sides.entry(c).or_default()[usize::from(s == Keep::Positive)] = true;
        } else if let (Some(b), Some(v)) = (f.base(), vol) {
            vols.entry(b).or_default().insert(*v);
        }
    }
    let mut v: Vec<PieceFace> = items
        .into_iter()
        .map(|x| x.0)
        .filter(|f| match f.base() {
            Some(b) => vols.get(&b).is_none_or(|s| s.len() < 2),
            None => cut_key(f).is_none_or(|(c, _)| sides.get(&c).is_none_or(|b| !(b[0] && b[1]))),
        })
        .collect();
    v.sort();
    v.dedup();
    v
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

    /// The faces of any piece (cells: stored; `Whole`: every face of the volume; composites: the
    /// faces captured when the composite was formed, internal faces removed). `t` is the host
    /// topology the model is defined on (not an overlay that has partitioned faces).
    pub fn piece_faces(&self, p: PieceRef, t: &dyn TopologyView) -> Vec<PieceFace> {
        match p {
            PieceRef::Whole(v) => t.volume_faces(v).iter().map(|f| PieceFace::Whole(*f)).collect(),
            PieceRef::Cell(c) => match self.cells.get(&c) {
                Some(_) if self.composite_faces.contains_key(&c) => self.composite_faces[&c].clone(),
                Some(pc) if !pc.members.is_empty() => self.member_faces(pc, t),
                Some(pc) => pc.faces.clone(),
                None => Vec::new(),
            },
        }
    }

    /// Faces of a composite computed from its members (see `merge_member_faces`).
    fn member_faces(&self, pc: &Piece, t: &dyn TopologyView) -> Vec<PieceFace> {
        let items: Vec<(PieceFace, Option<EntityId>)> = pc.members.iter().flat_map(|m| {
            let vol = self.volume_of(*m);
            self.piece_faces(*m, t).into_iter().map(move |f| (f, vol))
        }).collect();
        merge_member_faces(items)
    }

    /// The overlay cut cells recorded for a realized cut.
    pub fn cut_cells(&self, cut: CutId) -> Option<&CutCells> {
        self.realized.get(&cut)
    }

    /// The cells of the view `topo` that realize piece face `pf`, ascending: `Whole(f)` is `f`;
    /// a realized `Cut` is its cut face; a realized `Part` / `CutPart` the cells recorded at the
    /// split. Faces that later realized cuts partitioned are replaced by their halves. `None` if
    /// the face has no realization or a cell is not live in `topo` (an overlay that does not
    /// match this decomposition).
    pub fn piece_face_cells(&self, pf: &PieceFace, topo: &dyn TopologyView) -> Option<Vec<EntityId>> {
        let base: Vec<EntityId> = match pf {
            PieceFace::Whole(f) => vec![*f],
            PieceFace::Cut { cut, .. } => self.realized.get(cut)?.faces.clone(),
            PieceFace::Part { .. } | PieceFace::CutPart { .. } => self.face_cells.get(pf)?.clone(),
        };
        let mut out = Vec::new();
        let mut stack = base;
        let mut guard = 0usize;
        while let Some(x) = stack.pop() {
            guard += 1;
            if guard > 1 << 20 {
                return None;
            }
            if topo.is_alive(x) {
                out.push(x);
            } else {
                stack.extend_from_slice(self.split_cells.get(&x)?);
            }
        }
        out.sort();
        out.dedup();
        Some(out)
    }

    /// True if the split that produced `p`'s children was realized in an overlay.
    pub fn is_realized(&self, p: PieceRef) -> bool {
        self.splits.get(&p).is_some_and(|r| self.realized.contains_key(&r.cut))
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
        // Cut ids are never reused: the serial must exceed every id in use (split records, piece
        // faces, sweep hints, realized cuts and recorded face cells).
        let sweep_cuts = self.cells.values().flat_map(|p| match &p.shape {
            PieceShape::Sweep { source, target } => source.iter().chain(target).filter_map(cut_key).map(|(c, _)| c.0).collect::<Vec<_>>(),
            _ => Vec::new(),
        });
        let used = self
            .splits
            .values()
            .map(|r| r.cut.0)
            .chain(self.cells.values().flat_map(|p| p.faces.iter().filter_map(cut_key).map(|(c, _)| c.0)))
            .chain(sweep_cuts)
            .chain(self.realized.keys().map(|c| c.0))
            .chain(self.face_cells.keys().filter_map(cut_key).map(|(c, _)| c.0))
            .chain(self.composite_faces.values().flatten().filter_map(cut_key).map(|(c, _)| c.0))
            .max();
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
        for (cut, cells) in &self.realized {
            if !self.splits.values().any(|r| r.cut == *cut) {
                return bad(format!("realized cut {cut:?} has no split record"));
            }
            let ok = |id: &EntityId, want: &[EntityKind]| want.contains(&id.kind());
            let faces = [EntityKind::TopoFace, EntityKind::PartitionFace];
            let edges = [EntityKind::TopoEdge, EntityKind::PartitionEdge];
            let verts = [EntityKind::TopoVertex, EntityKind::PartitionVertex];
            if cells.faces.is_empty()
                || !cells.faces.iter().all(|f| ok(f, &faces))
                || !cells.edges.iter().all(|e| ok(e, &edges))
                || !cells.vertices.iter().all(|v| ok(v, &verts))
                || !cells.split_faces.iter().all(|(f, h)| ok(f, &faces) && h.iter().all(|x| ok(x, &faces)))
                || !cells.split_edges.iter().all(|(e, v, h)| ok(e, &edges) && ok(v, &verts) && h.iter().all(|x| ok(x, &edges)))
            {
                return bad(format!("realized cut {cut:?} names cells of the wrong kind"));
            }
        }
        for (pf, cells) in &self.face_cells {
            let piece = match pf {
                PieceFace::Part { piece, .. } | PieceFace::CutPart { piece, .. } => *piece,
                _ => return bad(format!("face cells recorded for {pf:?}")),
            };
            if !self.cells.contains_key(&piece) || cells.is_empty() {
                return bad(format!("face cells of {pf:?} name an unknown piece or no cell"));
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
                // The same faces `piece_faces` reports (internal faces excluded).
                Some(_) if self.composite_faces.contains_key(&c) => self.composite_faces[&c].clone(),
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
        let row = Piece { id, volume, faces: Vec::new(), shape: PieceShape::Any, members: pieces.to_vec() };
        let faces = d.member_faces(&row, t);
        d.composite_faces.insert(id, faces);
        d.cells.insert(id, row);
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
        d.composite_faces.remove(&composite);
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
        let d = &self.decomposition;
        let parent = *d.parent.get(&piece).ok_or(ModelError::Unknown(piece))?;
        if d.is_realized(parent) {
            return Err(ModelError::Invalid("the split is realized in an overlay; use unsplit_realized".into()));
        }
        self.unsplit_inner(piece)
    }

    fn unsplit_inner(&mut self, piece: EntityId) -> Result<PieceRef> {
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

    /// Split `piece` by a plane **and realize the cut in `overlay`** (the decomposition overlay
    /// over the host topology the model is defined on). The overlay volume of `piece` (the
    /// volume itself for `Whole`, the piece id for a cell realized earlier) is cut by
    /// `Overlay::partition_volume_by_plane` with the two new piece ids, the model faces and
    /// parent cut faces are classified onto the two sides from the overlay result, the split is
    /// recorded like [`Model::split_sides`], and the overlay cells are recorded ([`CutCells`],
    /// the cells of every `Part` / `CutPart` face of the children, the partitioned faces).
    /// Returns `[negative, positive]` (the negative side is opposite the plane normal).
    ///
    /// Atomic: on error neither the model nor the overlay changes. The piece's faces must all be
    /// resolvable in the overlay ([`Decomposition::piece_face_cells`]), which holds for whole
    /// volumes and for pieces made by earlier realized splits.
    pub fn split_realized(&mut self, piece: PieceRef, plane: Plane, overlay: &mut Overlay) -> Result<[EntityId; 2]> {
        let d = &self.decomposition;
        if !d.is_active(piece) || matches!(piece, PieceRef::Cell(c) if d.composites.contains(&c)) {
            return Err(ModelError::Invalid("only an active, non-composite piece can be split".into()));
        }
        let vol = match piece {
            PieceRef::Whole(v) => v,
            PieceRef::Cell(c) => c,
        };
        if !overlay.is_alive(vol) {
            return Err(ModelError::Invalid(format!("piece {vol} is not a volume of the overlay (split it with split_realized from the start)")));
        }
        let host = overlay.host_arc();
        let parent_faces = d.piece_faces(piece, host.as_ref());
        let mut cells_of: Vec<(PieceFace, Vec<EntityId>)> = Vec::with_capacity(parent_faces.len());
        for pf in &parent_faces {
            let cells = d.piece_face_cells(pf, overlay).ok_or_else(|| ModelError::Invalid(format!("piece face {pf:?} has no cells in the overlay")))?;
            cells_of.push((*pf, cells));
        }
        let mut ids = self.ids.clone();
        let kids = [ids.alloc(EntityKind::PartitionCell)?, ids.alloc(EntityKind::PartitionCell)?];
        let snapshot = overlay.clone();
        let origin = Point3::new(0.0, 0.0, 0.0) + plane.normal * plane.offset;
        let result = (|| -> Result<[EntityId; 2]> {
            let pc = overlay.partition_volume_by_plane(vol, origin, plane.normal, kids).map_err(|e| ModelError::CutInvalid(e.to_string()))?;
            let halves: BTreeMap<EntityId, [EntityId; 2]> = pc.split_faces.iter().map(|(f, _, h)| (*f, *h)).collect();
            let (neg_faces, pos_faces) = (overlay.volume_faces(kids[0]).to_vec(), overlay.volume_faces(kids[1]).to_vec());
            // Per parent piece face: its cells on the negative and on the positive side.
            let mut sided: Vec<(PieceFace, Vec<EntityId>, Vec<EntityId>)> = Vec::with_capacity(cells_of.len());
            for (pf, cells) in &cells_of {
                let (mut n, mut p) = (Vec::new(), Vec::new());
                for c in cells {
                    if let Some(h) = halves.get(c) {
                        n.push(h[0]);
                        p.push(h[1]);
                    } else if neg_faces.contains(c) {
                        n.push(*c);
                    } else if pos_faces.contains(c) {
                        p.push(*c);
                    } else {
                        return Err(ModelError::Invalid(format!("cell {c} of {pf:?} is on neither side of the cut")));
                    }
                }
                sided.push((*pf, n, p));
            }
            let side_list = |neg: bool| -> Vec<EntityId> { sided.iter().filter(|(_, n, p)| if neg { !n.is_empty() } else { !p.is_empty() }).filter_map(|(pf, _, _)| pf.base()).collect() };
            let cut_list = |neg: bool| -> Vec<(CutId, Keep)> { sided.iter().filter(|(_, n, p)| if neg { !n.is_empty() } else { !p.is_empty() }).filter_map(|(pf, _, _)| cut_key(pf)).collect() };
            let (neg, pos, neg_cuts, pos_cuts) = (side_list(true), side_list(false), cut_list(true), cut_list(false));
            let got = self.split_sides(piece, CutGeometry::Plane(plane), &SplitSides { neg: &neg, pos: &pos, neg_cuts: &neg_cuts, pos_cuts: &pos_cuts }, host.as_ref())?;
            // Same allocator state as the clone above, so the ids are the overlay's volumes.
            debug_assert_eq!(got, kids, "piece ids differ from the overlay volumes");
            let d = &mut self.decomposition;
            let cut = d.splits.get(&piece).map(|r| r.cut).ok_or_else(|| ModelError::Invalid("internal: split not recorded".into()))?;
            d.realized.insert(cut, CutCells {
                faces: vec![pc.face],
                edges: pc.loop_edges.clone(),
                vertices: pc.loop_vertices.clone(),
                split_faces: pc.split_faces.iter().map(|(f, _, h)| (*f, *h)).collect(),
                split_edges: pc.split_edges.clone(),
            });
            d.split_cells.extend(halves.iter().map(|(f, h)| (*f, *h)));
            for (k, child) in kids.iter().enumerate() {
                let faces = d.cells.get(child).map(|p| p.faces.clone()).unwrap_or_default();
                for pf in faces {
                    let key = match pf {
                        PieceFace::Part { face, .. } => sided.iter().find(|(q, _, _)| q.base() == Some(face)),
                        PieceFace::CutPart { cut, side, .. } => sided.iter().find(|(q, _, _)| cut_key(q) == Some((cut, side))),
                        _ => continue,
                    };
                    let Some((_, n, p)) = key else { return Err(ModelError::Invalid(format!("internal: {pf:?} has no parent face"))) };
                    let mut cells = if k == 0 { n.clone() } else { p.clone() };
                    cells.sort();
                    d.face_cells.insert(pf, cells);
                }
            }
            Ok(kids)
        })();
        if result.is_err() {
            *overlay = snapshot;
        }
        result
    }

    /// Undo a realized split (see [`Model::split_realized`]): removes the overlay operations of
    /// the cut (the volume split, then the face and edge partitions, newest first), then undoes
    /// the split like [`Model::unsplit`] and drops its recorded cells. Atomic: fails (changing
    /// nothing) if a later overlay operation still uses one of the cut's cells.
    pub fn unsplit_realized(&mut self, piece: EntityId, overlay: &mut Overlay) -> Result<PieceRef> {
        let d = &self.decomposition;
        let parent = *d.parent.get(&piece).ok_or(ModelError::Unknown(piece))?;
        let rec = d.splits.get(&parent).ok_or(ModelError::Unknown(piece))?.clone();
        let cells = d.realized.get(&rec.cut).cloned().ok_or_else(|| ModelError::Invalid("the split is not realized; use unsplit".into()))?;
        for c in rec.children {
            let r = PieceRef::Cell(c);
            if d.splits.contains_key(&r) || d.consumed.contains(&r) {
                return Err(ModelError::Invalid("both children must be unsplit leaves".into()));
            }
        }
        let mut ov = overlay.clone();
        let undo = |ov: &mut Overlay, id: EntityId| ov.remove(id).map_err(|e| ModelError::CutInvalid(e.to_string()));
        for f in &cells.faces {
            undo(&mut ov, *f)?;
        }
        for (_, h) in cells.split_faces.iter().rev() {
            undo(&mut ov, h[0])?;
        }
        for (_, v, _) in cells.split_edges.iter().rev() {
            undo(&mut ov, *v)?;
        }
        let out = self.unsplit_inner(piece)?;
        let d = &mut self.decomposition;
        d.realized.remove(&rec.cut);
        for (f, _) in &cells.split_faces {
            d.split_cells.remove(f);
        }
        d.face_cells.retain(|pf, _| !matches!(pf, PieceFace::Part { piece, .. } | PieceFace::CutPart { piece, .. } if rec.children.contains(piece)));
        *overlay = ov;
        Ok(out)
    }

    /// Drop the whole decomposition. Colors and every other table are untouched; piece ids stay
    /// retired. Recorded overlay cells are dropped too; clearing the overlay itself is the
    /// caller's job (`Overlay::clear`).
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
