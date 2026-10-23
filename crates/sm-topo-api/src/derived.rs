//! Derived views over the incidence relation. None of these is stored; they are computed from
//! [`TopologyView`] rows, so they work for any implementor (including virtual overlays).
//!
//! The radial-edge style entities map as follows: `Loop` = a ring of a face row, `Coedge` = one
//! entry of it, `Shell` = a ring of a volume row, `FaceUse` = one `(volume, face, side)`.

use crate::types::{Coincidence, Dim, Incidence, LineageKind, LineageRecord, Region, Sense, Side};
use crate::view::TopologyView;
use sm_core::EntityId;
use smallvec::SmallVec;

/// One use of an edge by a face loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Coedge {
    /// The face.
    pub face: EntityId,
    /// The edge.
    pub edge: EntityId,
    /// `Forward` if the loop runs along the edge direction (face on the edge's left).
    pub sense: Sense,
    /// Loop index within the face.
    pub ring: u16,
    /// Position within the loop.
    pub index: u32,
}

/// A closed loop of coedges of a face. Ring 0 is the outer loop by convention of the builder.
#[derive(Clone, Copy, Debug)]
pub struct Loop<'a> {
    /// The face.
    pub face: EntityId,
    /// Loop index.
    pub ring: u16,
    /// The cyclic coedge entries.
    pub entries: &'a [Incidence],
}

impl Loop<'_> {
    /// The coedges in order.
    pub fn coedges(&self) -> impl Iterator<Item = Coedge> + '_ {
        let (face, ring) = (self.face, self.ring);
        self.entries.iter().enumerate().map(move |(i, e)| Coedge { face, edge: e.cell, sense: e.side.sense(), ring, index: i as u32 })
    }
    /// Number of coedges.
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    /// True if the loop has no coedges.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// One side of one face as seen from a volume (the radial-edge "face use").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceUse {
    /// The face.
    pub face: EntityId,
    /// The volume.
    pub volume: EntityId,
    /// `Forward`: the face normal points out of the volume (volume on the left side).
    pub sense: Sense,
    /// Shell index within the volume.
    pub ring: u16,
}

/// A shell of a volume: a ring of its boundary row.
#[derive(Clone, Copy, Debug)]
pub struct Shell<'a> {
    /// The volume.
    pub volume: EntityId,
    /// Shell index (0 = outer by convention of the builder).
    pub ring: u16,
    /// The face-use entries.
    pub entries: &'a [Incidence],
}

impl Shell<'_> {
    /// The face uses; a baffle (`Both`) yields two uses with opposite senses.
    pub fn face_uses(&self) -> impl Iterator<Item = FaceUse> + '_ {
        let (volume, ring) = (self.volume, self.ring);
        self.entries.iter().flat_map(move |e| expand_uses(volume, ring, *e))
    }
}

fn expand_uses(volume: EntityId, ring: u16, e: Incidence) -> SmallVec<[FaceUse; 2]> {
    let mut v = SmallVec::new();
    let mk = |sense| FaceUse { face: e.cell, volume, sense, ring };
    match e.side {
        Side::Left => v.push(mk(Sense::Forward)),
        Side::Right => v.push(mk(Sense::Reversed)),
        Side::Both => {
            v.push(mk(Sense::Forward));
            v.push(mk(Sense::Reversed));
        }
    }
    v
}

/// The two sides of a face. The normal points from `left` to `right`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaceSides {
    /// Region the normal points away from.
    pub left: Region,
    /// Region the normal points into.
    pub right: Region,
}

/// One use of an edge in the radial star around it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadialUse {
    /// The face using the edge.
    pub face: EntityId,
    /// Coedge sense in that face.
    pub sense: Sense,
}

/// Splits a row into its consecutive ring groups.
fn rings(row: &[Incidence]) -> impl Iterator<Item = (u16, &[Incidence])> {
    row.chunk_by(|a, b| a.ring == b.ring).map(|g| (g[0].ring, g))
}

/// Derived queries available on every [`TopologyView`] (including `dyn TopologyView`).
pub trait TopologyViewExt: TopologyView {
    /// `(start, end)` vertices of an edge.
    fn edge_ends(&self, edge: EntityId) -> Option<(EntityId, EntityId)> {
        match self.boundary(edge) {
            [only] if only.side == Side::Both => Some((only.cell, only.cell)),
            [a, b] if a.side == Side::Left && b.side == Side::Right => Some((a.cell, b.cell)),
            _ => None,
        }
    }

    /// The loops of a face (embedded members excluded).
    fn loops(&self, face: EntityId) -> impl Iterator<Item = Loop<'_>> {
        rings(self.boundary(face)).filter(|(r, _)| *r != Incidence::EMBEDDED).map(move |(ring, entries)| Loop { face, ring, entries })
    }

    /// All coedges of a face, loop by loop.
    fn coedges(&self, face: EntityId) -> impl Iterator<Item = Coedge> + '_ {
        self.loops(face).flat_map(|l| l.coedges().collect::<SmallVec<[Coedge; 8]>>())
    }

    /// The shells of a volume.
    fn shells(&self, volume: EntityId) -> impl Iterator<Item = Shell<'_>> {
        rings(self.boundary(volume)).filter(|(r, _)| *r != Incidence::EMBEDDED).map(move |(ring, entries)| Shell { volume, ring, entries })
    }

    /// Every face use of a volume (a baffle gives two).
    fn face_uses(&self, volume: EntityId) -> impl Iterator<Item = FaceUse> + '_ {
        self.shells(volume).flat_map(|s| s.face_uses().collect::<SmallVec<[FaceUse; 8]>>())
    }

    /// Every face use of a face, one per side occupied by a volume.
    fn face_uses_of(&self, face: EntityId) -> SmallVec<[FaceUse; 2]> {
        let mut out = SmallVec::new();
        for e in self.coboundary(face) {
            if self.dim_of(e.cell) == Some(Dim::Volume) {
                out.extend(expand_uses(e.cell, e.ring, Incidence { cell: face, side: e.side, ring: e.ring }));
            }
        }
        out
    }

    /// The regions on the two sides of a face. A missing volume is [`Region::Exterior`].
    fn face_sides(&self, face: EntityId) -> FaceSides {
        let (mut left, mut right) = (Region::Exterior, Region::Exterior);
        for e in self.coboundary(face) {
            if self.dim_of(e.cell) != Some(Dim::Volume) {
                continue;
            }
            match e.side {
                Side::Left => left = Region::Volume(e.cell),
                Side::Right => right = Region::Volume(e.cell),
                Side::Both => {
                    left = Region::Volume(e.cell);
                    right = Region::Volume(e.cell);
                }
            }
        }
        FaceSides { left, right }
    }

    /// True if the same volume lies on both sides (internal baffle).
    fn is_baffle(&self, face: EntityId) -> bool {
        let s = self.face_sides(face);
        s.left == s.right && s.left != Region::Exterior
    }

    /// True if exactly one side is a volume (a boundary of the model).
    fn is_boundary_face(&self, face: EntityId) -> bool {
        let s = self.face_sides(face);
        (s.left == Region::Exterior) != (s.right == Region::Exterior)
    }

    /// True if two different volumes meet at the face (a merged interface).
    fn is_interface_face(&self, face: EntityId) -> bool {
        let s = self.face_sides(face);
        s.left != s.right && s.left != Region::Exterior && s.right != Region::Exterior
    }

    /// True if no volume touches the face (a sheet).
    fn is_free_face(&self, face: EntityId) -> bool {
        let s = self.face_sides(face);
        s.left == Region::Exterior && s.right == Region::Exterior
    }

    /// The radial star of an edge: every face coedge using it, in face-id order.
    fn radial_uses(&self, edge: EntityId) -> impl Iterator<Item = RadialUse> + '_ {
        self.coboundary(edge).iter().filter(move |e| self.dim_of(e.cell) == Some(Dim::Face)).map(|e| RadialUse { face: e.cell, sense: e.side.sense() })
    }

    /// Edges used twice by `face` with opposite senses (periodic seams), ascending.
    fn seam_edges(&self, face: EntityId) -> SmallVec<[EntityId; 2]> {
        let mut seen: SmallVec<[(EntityId, Side); 8]> = self.loops(face).flat_map(|l| l.entries.iter().map(|e| (e.cell, e.side)).collect::<SmallVec<[_; 8]>>()).collect();
        seen.sort();
        let mut out: SmallVec<[EntityId; 2]> = SmallVec::new();
        for w in seen.windows(2) {
            if w[0].0 == w[1].0 && w[0].1 != w[1].1 && out.last() != Some(&w[0].0) {
                out.push(w[0].0);
            }
        }
        out
    }

    /// Edges embedded (as wires) in the interior of a face, volume or body.
    fn wire_edges(&self, parent: EntityId) -> impl Iterator<Item = EntityId> + '_ {
        self.boundary(parent).iter().filter(move |e| self.dim_of(e.cell) == Some(Dim::Edge) && (e.ring == Incidence::EMBEDDED || self.dim_of(parent) == Some(Dim::Body))).map(|e| e.cell)
    }

    /// The closure of a cell: itself plus all lower cells it is built from, ascending.
    fn closure(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = vec![id];
        let mut i = 0;
        while i < out.len() {
            let cur = out[i];
            out.extend(self.boundary(cur).iter().map(|e| e.cell));
            i += 1;
        }
        out.sort();
        out.dedup();
        out
    }

    /// Lineage records that mention `id` as source or result, in log order.
    fn lineage_of(&self, id: EntityId) -> Vec<&LineageRecord> {
        self.lineage().iter().filter(|r| r.from.contains(&id) || r.to.contains(&id)).collect()
    }

    /// The live cells `id` has become through splits and merges (itself if still alive; empty if
    /// deleted). Ascending, deduplicated.
    fn descendants(&self, id: EntityId) -> Vec<EntityId> {
        let mut out = Vec::new();
        let mut stack = vec![id];
        let mut seen: Vec<EntityId> = Vec::new();
        while let Some(cur) = stack.pop() {
            if seen.contains(&cur) {
                continue;
            }
            seen.push(cur);
            if self.is_alive(cur) {
                out.push(cur);
                continue;
            }
            for r in self.lineage() {
                if r.from.first() == Some(&cur) || (r.kind == LineageKind::Merged && r.from.get(1) == Some(&cur)) {
                    stack.extend(r.to.iter().copied());
                }
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// The recorded relation between two faces, if any.
    fn coincidence_between(&self, a: EntityId, b: EntityId) -> Option<Coincidence> {
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        self.coincidences().iter().find(|p| p.a == a && p.b == b).map(|p| p.status)
    }
}

impl<T: TopologyView + ?Sized> TopologyViewExt for T {}
