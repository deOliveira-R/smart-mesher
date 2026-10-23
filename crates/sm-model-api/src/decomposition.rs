//! Decomposition: a virtual, reversible split of the colored model into meshable **pieces**
//! (`EntityKind::PartitionCell`).
//!
//! A piece has **no color of its own**. Its color is always read through its volumes, so a
//! decomposition cannot change the coloring by construction; the validator additionally checks
//! that every piece is color-uniform. Dropping the decomposition ([`Model::undo_decomposition`])
//! restores the plain colored model exactly, because no other table refers to pieces.

use crate::action::Plane;
use crate::error::{ModelError, Result};
use crate::model::Model;
use sm_core::{EntityId, EntityKind};
use std::collections::BTreeMap;

/// Identifier of a virtual cut inside a [`Decomposition`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutId(pub u32);

/// The surface of a virtual cut.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CutSurface {
    /// A plane.
    Plane(Plane),
    /// An existing `Surface` entity of the shape layer.
    Surface(EntityId),
}

/// A virtual cut of one volume. It exists only here; the topology is untouched.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Cut {
    /// The `TopoVolume` being cut.
    pub volume: EntityId,
    /// The cutting surface.
    pub surface: CutSurface,
}

/// A face as seen by a piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceFace {
    /// An original `TopoFace` (or the part of it that bounds this piece).
    Original(EntityId),
    /// A virtual cut face; each cut is shared by exactly two pieces.
    Cut(CutId),
}

/// Meshing-relevant shape class of a piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum PieceShape {
    /// Sweepable between a source and a target face.
    Sweep {
        /// Source face (meshed first).
        source: PieceFace,
        /// Target face.
        target: PieceFace,
    },
    /// Mappable (hex-meshable by mapping).
    Mappable,
    /// No assertion; the mesher decides.
    Free,
}

/// Row of the piece table (key: `EntityKind::PartitionCell` id).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Piece {
    /// Name.
    pub name: String,
    /// Volumes (>= 1) that this piece covers (a part of, or, for a composite, several whole volumes).
    /// All must carry the same color.
    pub volumes: Vec<EntityId>,
    /// Bounding faces, sorted and unique.
    pub faces: Vec<PieceFace>,
    /// Shape class.
    pub shape: PieceShape,
}

/// The piece and cut tables.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Decomposition {
    /// Piece table.
    pub pieces: BTreeMap<EntityId, Piece>,
    /// Cut table.
    pub cuts: BTreeMap<CutId, Cut>,
}

impl Decomposition {
    /// Pieces that cover `volume`, ascending.
    pub fn pieces_of_volume(&self, volume: EntityId) -> Vec<EntityId> {
        self.pieces.iter().filter(|(_, p)| p.volumes.contains(&volume)).map(|(id, _)| *id).collect()
    }

    /// Pieces bounded by the original face `face`, ascending.
    pub fn pieces_of_face(&self, face: EntityId) -> Vec<EntityId> {
        self.pieces.iter().filter(|(_, p)| p.faces.contains(&PieceFace::Original(face))).map(|(id, _)| *id).collect()
    }

    /// Volumes covered by `piece` (empty if unknown).
    pub fn volumes_of_piece(&self, piece: EntityId) -> &[EntityId] {
        self.pieces.get(&piece).map_or(&[], |p| &p.volumes)
    }

    /// Original faces bounding `piece`, ascending (virtual cut faces excluded).
    pub fn original_faces_of_piece(&self, piece: EntityId) -> Vec<EntityId> {
        self.pieces
            .get(&piece)
            .map(|p| p.faces.iter().filter_map(|f| if let PieceFace::Original(f) = f { Some(*f) } else { None }).collect())
            .unwrap_or_default()
    }

    /// The pieces on the two sides of `cut`.
    pub fn pieces_of_cut(&self, cut: CutId) -> Vec<EntityId> {
        self.pieces.iter().filter(|(_, p)| p.faces.contains(&PieceFace::Cut(cut))).map(|(id, _)| *id).collect()
    }
}

impl Model {
    /// Color of a piece: the common color of its volumes. `None` if uncolored or mixed
    /// (the validator reports mixed pieces).
    pub fn piece_color(&self, piece: EntityId) -> Option<EntityId> {
        let p = self.decomposition.as_ref()?.pieces.get(&piece)?;
        let mut it = p.volumes.iter().map(|v| self.volume_color.get(v).copied());
        let first = it.next()??;
        it.all(|c| c == Some(first)).then_some(first)
    }

    /// Start (or continue) a decomposition: register a virtual cut of `volume`.
    pub fn add_cut(&mut self, volume: EntityId, surface: CutSurface) -> Result<CutId> {
        crate::color::expect_kind(volume, EntityKind::TopoVolume)?;
        let d = self.decomposition.get_or_insert_with(Decomposition::default);
        let id = CutId(d.cuts.keys().next_back().map_or(0, |c| c.0 + 1));
        d.cuts.insert(id, Cut { volume, surface });
        Ok(id)
    }

    /// Add a piece. `faces` is sorted and deduplicated for you.
    pub fn add_piece(&mut self, name: &str, volumes: &[EntityId], faces: &[PieceFace], shape: PieceShape) -> Result<EntityId> {
        if volumes.is_empty() {
            return Err(ModelError::Invalid("a piece needs at least one volume".into()));
        }
        for v in volumes {
            crate::color::expect_kind(*v, EntityKind::TopoVolume)?;
        }
        let id = self.ids.alloc(EntityKind::PartitionCell)?;
        let mut faces = faces.to_vec();
        faces.sort();
        faces.dedup();
        let mut volumes = volumes.to_vec();
        volumes.sort();
        volumes.dedup();
        self.decomposition.get_or_insert_with(Decomposition::default).pieces.insert(id, Piece { name: name.to_owned(), volumes, faces, shape });
        Ok(id)
    }

    /// Drop the decomposition. The coloring and all other tables are untouched.
    pub fn undo_decomposition(&mut self) {
        self.decomposition = None;
    }

    /// The trivial decomposition: one piece per volume, with all its faces taken from `topo`.
    pub fn decompose_trivially(&mut self, topo: &dyn crate::topology::TopologyView) -> Result<()> {
        let faces = topo.faces();
        for v in topo.volumes() {
            let fs: Vec<PieceFace> = faces.iter().filter(|f| topo.face_sides(**f).iter().any(|s| s.volume == v)).map(|f| PieceFace::Original(*f)).collect();
            self.add_piece(&format!("piece-{v}"), &[v], &fs, PieceShape::Free)?;
        }
        Ok(())
    }
}
