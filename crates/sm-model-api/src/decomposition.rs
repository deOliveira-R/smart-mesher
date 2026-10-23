//! Decomposition: a virtual, reversible split of the colored model into meshable **pieces**
//! (`EntityKind::PartitionCell`): sweep blocks, mappable chunks, composites.
//!
//! Invariants:
//! - A piece stores **no color**. Its color is the color of its parent volume(s), always derived
//!   ([`Decomposition::piece_volume_color`]). Decomposition therefore cannot change coloring; the only
//!   way to get it wrong is a composite of differently colored volumes, which [`Decomposition::composite`]
//!   refuses and the validator re-checks on loaded data.
//! - Pieces of one volume cover it exactly; virtual cuts are internal and shared by exactly two
//!   pieces (or one, on a not-yet-closed edit).
//! - Every mutation pushes a snapshot, so [`Decomposition::undo`] is exact and the model is untouched.
//!
//! The geometry of the cuts belongs to the virtual-topology layer; this layer records the
//! combinatorics it needs: which model faces each piece touches (whole or partially), which virtual
//! cuts bound it, and its sweep roles.

use crate::coloring::Coloring;
use crate::error::{ModelError, Result};
use crate::ids::IdAllocator;
use crate::topo::TopologyView;
use crate::xf::Plane;
use sm_core::{EntityId, EntityKind};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};

/// Id of a virtual cut (an internal piece-to-piece interface that is not a model face).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutId(pub u32);

/// Geometry of a virtual cut, as far as this layer cares.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CutGeometry {
    /// A plane.
    Plane(Plane),
    /// An existing model surface.
    Surface(EntityId),
    /// Defined elsewhere (virtual-topology overlay).
    Unspecified,
}

/// A virtual cut.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VirtualCut {
    /// Id.
    pub id: CutId,
    /// Label for display and scripting.
    pub label: String,
    /// Geometry.
    pub geometry: CutGeometry,
}

/// A face of a piece boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceFace {
    /// A model face touched by the piece. `whole = false` means only a portion is on the piece.
    Model {
        /// The model face.
        face: EntityId,
        /// True if the piece covers the entire face.
        whole: bool,
    },
    /// A virtual cut between pieces.
    Virtual(CutId),
}

/// Sweep roles (Cubit-style): source(s) and target(s); the remaining faces are linking.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SweepRoles {
    /// Source faces (sorted).
    pub sources: Vec<PieceFace>,
    /// Target faces (sorted).
    pub targets: Vec<PieceFace>,
}

/// Meshing-relevant shape of a piece.
#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceShape {
    /// No claim.
    Free,
    /// Mappable / submappable chunk.
    Mappable,
    /// Sweepable between source and target faces.
    Sweep(SweepRoles),
}

/// A meshable piece.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Piece {
    /// `EntityKind::PartitionCell` id.
    pub id: EntityId,
    /// Parent volume(s); more than one means a composite.
    pub volumes: SmallVec<[EntityId; 1]>,
    /// Shape.
    pub shape: PieceShape,
    /// Boundary faces (sorted, unique).
    pub faces: Vec<PieceFace>,
    /// Measure of the piece if known (checked against its volume's moments by the validator).
    pub measure: Option<f64>,
}

/// One side of a split.
#[derive(Clone, Debug, PartialEq)]
pub struct PieceSpec {
    /// Model faces (and any pre-existing virtual cuts) of this side. The new cut is added
    /// automatically.
    pub faces: Vec<PieceFace>,
    /// Shape; sweep roles may reference the new cut as `PieceFace::Virtual(cut)`.
    pub shape: PieceShape,
    /// Measure if known.
    pub measure: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq)]
struct State {
    pieces: BTreeMap<EntityId, Piece>,
    cuts: BTreeMap<CutId, VirtualCut>,
}

/// The decomposition: current pieces, virtual cuts and an undo stack.
#[derive(Clone, Debug, Default)]
pub struct Decomposition {
    state: State,
    history: Vec<State>,
    next_cut: u32,
}

fn norm_faces(mut f: Vec<PieceFace>) -> Vec<PieceFace> {
    f.sort();
    f.dedup();
    f
}

impl Decomposition {
    /// The identity decomposition: one `Free` piece per volume covering it whole.
    pub fn identity(topo: &dyn TopologyView, ids: &mut dyn IdAllocator) -> Self {
        let mut state = State::default();
        for v in topo.volumes() {
            let id = ids.alloc(EntityKind::PartitionCell);
            let faces = topo.volume_faces(v).into_iter().map(|face| PieceFace::Model { face, whole: true }).collect();
            state.pieces.insert(id, Piece { id, volumes: SmallVec::from_elem(v, 1), shape: PieceShape::Free, faces: norm_faces(faces), measure: None });
        }
        Self { state, history: Vec::new(), next_cut: 0 }
    }

    /// All pieces in id order.
    pub fn pieces(&self) -> impl Iterator<Item = &Piece> {
        self.state.pieces.values()
    }

    /// Piece by id.
    pub fn piece(&self, id: EntityId) -> Option<&Piece> {
        self.state.pieces.get(&id)
    }

    /// Virtual cut by id.
    pub fn cut(&self, id: CutId) -> Option<&VirtualCut> {
        self.state.cuts.get(&id)
    }

    /// Number of undoable steps.
    pub fn depth(&self) -> usize {
        self.history.len()
    }

    /// Register a virtual cut to be used by a following [`Decomposition::split`].
    pub fn new_cut(&mut self, label: &str, geometry: CutGeometry) -> CutId {
        let id = CutId(self.next_cut);
        self.next_cut += 1;
        self.state.cuts.insert(id, VirtualCut { id, label: label.to_string(), geometry });
        id
    }

    /// Pieces whose parent volumes include `v`, in id order.
    pub fn pieces_of_volume(&self, v: EntityId) -> Vec<EntityId> {
        self.pieces().filter(|p| p.volumes.contains(&v)).map(|p| p.id).collect()
    }

    /// Parent volumes of a piece.
    pub fn volumes_of_piece(&self, p: EntityId) -> Vec<EntityId> {
        self.piece(p).map(|p| p.volumes.to_vec()).unwrap_or_default()
    }

    /// Pieces touching model face `f`, in id order.
    pub fn pieces_of_face(&self, f: EntityId) -> Vec<EntityId> {
        self.pieces().filter(|p| p.faces.iter().any(|pf| matches!(pf, PieceFace::Model { face, .. } if *face == f))).map(|p| p.id).collect()
    }

    /// Model faces touched by a piece (with the `whole` flag), sorted.
    pub fn faces_of_piece(&self, p: EntityId) -> Vec<(EntityId, bool)> {
        self.piece(p)
            .map(|p| {
                p.faces
                    .iter()
                    .filter_map(|pf| match pf {
                        PieceFace::Model { face, whole } => Some((*face, *whole)),
                        PieceFace::Virtual(_) => None,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Virtual cuts bounding a piece.
    pub fn cuts_of_piece(&self, p: EntityId) -> Vec<CutId> {
        self.piece(p)
            .map(|p| p.faces.iter().filter_map(|pf| if let PieceFace::Virtual(c) = pf { Some(*c) } else { None }).collect())
            .unwrap_or_default()
    }

    /// Pieces across a virtual cut from `p`.
    pub fn neighbors(&self, p: EntityId) -> Vec<EntityId> {
        let cuts = self.cuts_of_piece(p);
        self.pieces().filter(|q| q.id != p && q.faces.iter().any(|f| matches!(f, PieceFace::Virtual(c) if cuts.contains(c)))).map(|q| q.id).collect()
    }

    /// Pieces with a `Sweep` shape, in id order.
    pub fn sweep_pieces(&self) -> Vec<EntityId> {
        self.pieces().filter(|p| matches!(p.shape, PieceShape::Sweep(_))).map(|p| p.id).collect()
    }

    /// The color of a piece: the color of its parent volume(s), derived and never stored. `None` if
    /// uncolored or if a (malformed) composite mixes colors.
    pub fn piece_volume_color(&self, coloring: &Coloring, p: EntityId) -> Option<EntityId> {
        let piece = self.piece(p)?;
        let mut it = piece.volumes.iter().map(|&v| coloring.volume_color(v));
        let first = it.next()??;
        for c in it {
            if c? != first {
                return None;
            }
        }
        Some(first)
    }

    /// Split a single-volume piece in two across the virtual cut `cut`. The union of the sides'
    /// model faces must equal the parent's; a face present on both sides is partial (`whole=false`
    /// on both). The cut is appended to both sides. Returns the two new piece ids; the parent is
    /// removed (undo restores it).
    pub fn split(&mut self, piece: EntityId, cut: CutId, a: PieceSpec, b: PieceSpec, ids: &mut dyn IdAllocator) -> Result<[EntityId; 2]> {
        let parent = self.state.pieces.get(&piece).ok_or(ModelError::UnknownEntity(piece))?.clone();
        if parent.volumes.len() != 1 {
            return Err(ModelError::Invalid("cannot split a composite piece; undo the composite first".into()));
        }
        if !self.state.cuts.contains_key(&cut) {
            return Err(ModelError::Invalid(format!("unknown virtual cut {cut:?}")));
        }
        let model_faces = |fs: &[PieceFace]| -> BTreeSet<EntityId> {
            fs.iter().filter_map(|f| if let PieceFace::Model { face, .. } = f { Some(*face) } else { None }).collect()
        };
        let want = model_faces(&parent.faces);
        let (fa, fb) = (model_faces(&a.faces), model_faces(&b.faces));
        let got: BTreeSet<EntityId> = fa.union(&fb).copied().collect();
        if got != want {
            return Err(ModelError::Invalid("split sides do not cover exactly the parent's model faces".into()));
        }
        let both: BTreeSet<EntityId> = fa.intersection(&fb).copied().collect();
        let fix = |spec: PieceSpec| -> Vec<PieceFace> {
            let mut v: Vec<PieceFace> = spec
                .faces
                .into_iter()
                .map(|f| match f {
                    PieceFace::Model { face, whole } => PieceFace::Model { face, whole: whole && !both.contains(&face) },
                    other => other,
                })
                .collect();
            v.push(PieceFace::Virtual(cut));
            norm_faces(v)
        };
        let (shape_a, shape_b, ma, mb) = (a.shape.clone(), b.shape.clone(), a.measure, b.measure);
        let (faces_a, faces_b) = (fix(a), fix(b));
        self.history.push(self.state.clone());
        self.state.pieces.remove(&piece);
        let vol = parent.volumes[0];
        let (ia, ib) = (ids.alloc(EntityKind::PartitionCell), ids.alloc(EntityKind::PartitionCell));
        for (id, shape, faces, measure) in [(ia, shape_a, faces_a, ma), (ib, shape_b, faces_b, mb)] {
            self.state.pieces.insert(id, Piece { id, volumes: SmallVec::from_elem(vol, 1), shape, faces, measure });
        }
        Ok([ia, ib])
    }

    /// Merge pieces into one composite (shape `Free`). Virtual cuts internal to the merged set
    /// disappear. Refuses to mix volume colors, so coloring cannot change.
    pub fn composite(&mut self, pieces: &[EntityId], coloring: &Coloring, ids: &mut dyn IdAllocator) -> Result<EntityId> {
        if pieces.len() < 2 {
            return Err(ModelError::Invalid("composite needs at least two pieces".into()));
        }
        let set: BTreeSet<EntityId> = pieces.iter().copied().collect();
        let mut members = Vec::new();
        for p in &set {
            members.push(self.state.pieces.get(p).ok_or(ModelError::UnknownEntity(*p))?.clone());
        }
        let mut colors = BTreeSet::new();
        for m in &members {
            for &v in &m.volumes {
                colors.insert(coloring.volume_color(v).ok_or_else(|| ModelError::Invalid(format!("volume {v} is uncolored")))?);
            }
        }
        if colors.len() != 1 {
            return Err(ModelError::Invalid("composite would mix volume colors".into()));
        }
        // cuts internal to the set: referenced only by members
        let mut cut_users: BTreeMap<CutId, usize> = BTreeMap::new();
        for p in self.state.pieces.values() {
            for f in &p.faces {
                if let PieceFace::Virtual(c) = f {
                    *cut_users.entry(*c).or_insert(0) += usize::from(set.contains(&p.id));
                }
            }
        }
        let total_users: BTreeMap<CutId, usize> = {
            let mut m = BTreeMap::new();
            for p in self.state.pieces.values() {
                for f in &p.faces {
                    if let PieceFace::Virtual(c) = f {
                        *m.entry(*c).or_insert(0) += 1;
                    }
                }
            }
            m
        };
        let internal: BTreeSet<CutId> = cut_users.iter().filter(|(c, n)| **n == total_users[*c]).map(|(c, _)| *c).collect();
        // model faces touched by non-members
        let mut outside: BTreeSet<EntityId> = BTreeSet::new();
        for p in self.state.pieces.values().filter(|p| !set.contains(&p.id)) {
            for f in &p.faces {
                if let PieceFace::Model { face, .. } = f {
                    outside.insert(*face);
                }
            }
        }
        let mut faces: BTreeMap<EntityId, bool> = BTreeMap::new();
        let mut virt = Vec::new();
        for m in &members {
            for f in &m.faces {
                match f {
                    PieceFace::Model { face, whole } => {
                        let e = faces.entry(*face).or_insert(false);
                        *e |= *whole;
                    }
                    PieceFace::Virtual(c) if !internal.contains(c) => virt.push(PieceFace::Virtual(*c)),
                    PieceFace::Virtual(_) => {}
                }
            }
        }
        let mut all: Vec<PieceFace> = faces.into_iter().map(|(face, w)| PieceFace::Model { face, whole: w || !outside.contains(&face) }).collect();
        all.extend(virt);
        let mut vols: SmallVec<[EntityId; 1]> = members.iter().flat_map(|m| m.volumes.iter().copied()).collect();
        vols.sort();
        vols.dedup();
        let measure = members.iter().map(|m| m.measure).try_fold(0.0, |acc, m| m.map(|x| acc + x));
        self.history.push(self.state.clone());
        for p in &set {
            self.state.pieces.remove(p);
        }
        for c in &internal {
            self.state.cuts.remove(c);
        }
        let id = ids.alloc(EntityKind::PartitionCell);
        self.state.pieces.insert(id, Piece { id, volumes: vols, shape: PieceShape::Free, faces: norm_faces(all), measure });
        Ok(id)
    }

    /// Set the shape of a piece (e.g. mark a composite as sweepable).
    pub fn set_shape(&mut self, piece: EntityId, shape: PieceShape) -> Result<()> {
        if !self.state.pieces.contains_key(&piece) {
            return Err(ModelError::UnknownEntity(piece));
        }
        self.history.push(self.state.clone());
        if let Some(p) = self.state.pieces.get_mut(&piece) {
            p.shape = shape;
        }
        Ok(())
    }

    /// Undo the last mutation. Returns false if there is nothing to undo.
    pub fn undo(&mut self) -> bool {
        match self.history.pop() {
            Some(s) => {
                self.state = s;
                true
            }
            None => false,
        }
    }

    /// Undo everything, back to the state after `identity`.
    pub fn reset(&mut self) {
        if let Some(first) = self.history.first().cloned() {
            self.state = first;
            self.history.clear();
        }
    }
}
