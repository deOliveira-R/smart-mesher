//! Derived views over the incidence relation. None of these is stored; they are computed from
//! [`TopologyView`] rows, so they work for any implementor (including virtual overlays).
//!
//! The radial-edge style entities map as follows: `Loop` = a ring of a face row, `Coedge` = one
//! entry of it (named by a stable [`CoedgeKey`]), `Shell` = a ring of a volume row, `FaceUse` =
//! one `(volume, face, side)`.

use crate::types::{Carrier, CellFlags, CoedgeKey, CoedgeKind, Dim, EntityState, FacePair, Incidence, Region, Sense, Side};
use crate::view::TopologyView;
use sm_core::{EntityId, Point3};
use sm_geom_api::{Crossing, RayHit};
use smallvec::SmallVec;

/// One use of an edge by a face loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Coedge {
    /// Stable name `{face, edge, side}`.
    pub key: CoedgeKey,
    /// Loop index within the face.
    pub ring: u16,
    /// Position within the loop (changes when the loop is edited; use `key` to keep a reference).
    pub index: u32,
    /// Real edge or degenerate pole coedge.
    pub kind: CoedgeKind,
}

impl Coedge {
    /// `Forward` if the loop runs along the edge direction (face on the edge's left).
    pub fn sense(&self) -> Sense {
        self.key.side.sense()
    }
}

/// A closed loop of coedges of a face. Ring 0 is the outer loop by convention of the builder.
#[derive(Clone, Copy, Debug)]
pub struct Loop<'a> {
    /// The face.
    pub face: EntityId,
    /// Loop index.
    pub ring: u16,
    /// The cyclic coedge entries (`cell` = edge, `side` = Left/Right).
    pub entries: &'a [Incidence],
}

impl Loop<'_> {
    /// The coedge keys in loop order.
    pub fn keys(&self) -> impl Iterator<Item = CoedgeKey> + '_ {
        let face = self.face;
        self.entries.iter().map(move |e| CoedgeKey { face, edge: e.cell, side: e.side })
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
    /// Shell index within the volume.
    pub shell: u16,
    /// `Forward`: the face normal points out of the volume (volume on the left side).
    pub sense: Sense,
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

fn expand_uses(volume: EntityId, shell: u16, e: Incidence) -> SmallVec<[FaceUse; 2]> {
    let mut v = SmallVec::new();
    let mk = |sense| FaceUse { face: e.cell, volume, shell, sense };
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
    /// Region the normal points away from. For two volumes, the lower id (docs/design/ir.md §6).
    pub left: Region,
    /// Region the normal points into. `Exterior` on a boundary face (normal outward).
    pub right: Region,
}

/// What a face is, topologically.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClass {
    /// No volume touches the face (a sheet).
    Free,
    /// Exactly one side is a volume (a boundary of the model).
    Boundary,
    /// Two different volumes meet at the face.
    Interface,
    /// The same volume lies on both sides (internal baffle).
    Baffle,
}

/// Splits a row into its consecutive ring groups.
fn rings(row: &[Incidence]) -> impl Iterator<Item = (u16, &[Incidence])> {
    row.chunk_by(|a, b| a.ring == b.ring).map(|g| (g[0].ring, g))
}

/// Derived queries available on every [`TopologyView`] (including `dyn TopologyView`).
pub trait TopologyViewExt: TopologyView {
    /// Live volumes, ascending (`cells(Dim::Volume)`).
    fn volumes(&self) -> &[EntityId] {
        self.cells(Dim::Volume)
    }

    /// Live faces, ascending (`cells(Dim::Face)`).
    fn faces(&self) -> &[EntityId] {
        self.cells(Dim::Face)
    }

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

    /// All coedges of a face, loop by loop, with stable keys. Degenerate edges give
    /// [`CoedgeKind::Pole`] coedges.
    fn coedges(&self, face: EntityId) -> impl Iterator<Item = Coedge> + '_ {
        self.loops(face).flat_map(move |l| {
            l.entries.iter().enumerate().map(move |(i, e)| Coedge {
                key: CoedgeKey { face, edge: e.cell, side: e.side },
                ring: l.ring,
                index: i as u32,
                kind: if self.flags(e.cell).contains(CellFlags::DEGENERATE) { CoedgeKind::Pole } else { CoedgeKind::Edge },
            })
        })
    }

    /// The corner (tail vertex) of every coedge of a loop, in loop order; a seam vertex appears
    /// twice. Pole coedges are skipped.
    fn loop_vertices(&self, face: EntityId, ring: u16) -> impl Iterator<Item = EntityId> + '_ {
        self.loops(face).filter(move |l| l.ring == ring).flat_map(move |l| {
            l.entries
                .iter()
                .filter(move |e| !self.flags(e.cell).contains(CellFlags::DEGENERATE))
                .filter_map(move |e| self.edge_ends(e.cell).map(|(s, t)| if e.side == Side::Right { t } else { s }))
        })
    }

    /// Corner points of the outer loop in loop order (seam vertices twice, poles skipped).
    fn face_outline(&self, face: EntityId) -> Vec<Point3> {
        self.loop_vertices(face, 0)
            .filter_map(|v| match self.carrier(v) {
                Carrier::Point(p) => Some(*p),
                _ => None,
            })
            .collect()
    }

    /// The shells of a volume.
    fn shells(&self, volume: EntityId) -> impl Iterator<Item = Shell<'_>> {
        rings(self.boundary(volume)).filter(|(r, _)| *r != Incidence::EMBEDDED).map(move |(ring, entries)| Shell { volume, ring, entries })
    }

    /// Every face use of a volume (a baffle gives two).
    fn face_uses(&self, volume: EntityId) -> impl Iterator<Item = FaceUse> + '_ {
        self.shells(volume).flat_map(|s| s.face_uses().collect::<SmallVec<[FaceUse; 8]>>())
    }

    /// Every face use of a face, one per side occupied by a volume, ascending by volume.
    fn face_uses_of(&self, face: EntityId) -> SmallVec<[FaceUse; 2]> {
        let mut out = SmallVec::new();
        for e in self.coboundary(face) {
            if self.dim_of(e.cell) == Some(Dim::Volume) {
                out.extend(expand_uses(e.cell, e.ring, Incidence { cell: face, side: e.side, ring: e.ring }));
            }
        }
        out
    }

    /// The senses in which `volume` uses `face`: one entry normally, two for a baffle, none if
    /// the volume does not use the face.
    fn face_use_in(&self, face: EntityId, volume: EntityId) -> SmallVec<[Sense; 2]> {
        self.face_uses_of(face).iter().filter(|u| u.volume == volume).map(|u| u.sense).collect()
    }

    /// The regions on the two sides of a face (`None` for an unknown or dead face). A missing
    /// volume is [`Region::Exterior`]. Normal points from `left` to `right`.
    fn face_sides(&self, face: EntityId) -> Option<FaceSides> {
        if self.dim_of(face) != Some(Dim::Face) || !self.is_alive(face) {
            return None;
        }
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
        Some(FaceSides { left, right })
    }

    /// Classification of a face by its two sides.
    fn face_class(&self, face: EntityId) -> Option<FaceClass> {
        let s = self.face_sides(face)?;
        Some(match (s.left, s.right) {
            (Region::Exterior, Region::Exterior) => FaceClass::Free,
            (Region::Exterior, _) | (_, Region::Exterior) => FaceClass::Boundary,
            (l, r) if l == r => FaceClass::Baffle,
            _ => FaceClass::Interface,
        })
    }

    /// Faces with exactly one volume side, ascending.
    fn boundary_faces(&self) -> impl Iterator<Item = EntityId> + '_ {
        self.faces().iter().copied().filter(|f| self.face_class(*f) == Some(FaceClass::Boundary))
    }

    /// True if the same volume lies on both sides (internal baffle).
    fn is_baffle(&self, face: EntityId) -> bool {
        self.face_class(face) == Some(FaceClass::Baffle)
    }

    /// True if exactly one side is a volume (a boundary of the model).
    fn is_boundary_face(&self, face: EntityId) -> bool {
        self.face_class(face) == Some(FaceClass::Boundary)
    }

    /// True if two different volumes meet at the face (a merged interface).
    fn is_interface_face(&self, face: EntityId) -> bool {
        self.face_class(face) == Some(FaceClass::Interface)
    }

    /// True if no volume touches the face (a sheet).
    fn is_free_face(&self, face: EntityId) -> bool {
        self.face_class(face) == Some(FaceClass::Free)
    }

    /// The face reached from `face` by turning around `edge` inside `volume`: the neighbour in
    /// the radial ring on the side of `face` that `volume` occupies. For a face with `volume` on
    /// both sides (baffle) the counter-clockwise neighbour is returned. `None` if `volume` does
    /// not touch `face` at `edge`, or the ring has no other face.
    fn face_across_edge(&self, face: EntityId, edge: EntityId, volume: EntityId) -> Option<EntityId> {
        let ring = self.radial_uses(edge);
        let n = ring.len();
        if n < 2 {
            return None;
        }
        let sides = self.face_sides(face)?;
        let want = Region::Volume(volume);
        let i = ring.iter().position(|u| u.face == face)?;
        // Counter-clockwise about the edge direction a Forward coedge's face is left behind on
        // its right side; a Reversed coedge on its left side.
        let (ccw, cw) = if ring[i].sense == Sense::Forward { (sides.right, sides.left) } else { (sides.left, sides.right) };
        if ccw == want {
            Some(ring[(i + 1) % n].face)
        } else if cw == want {
            Some(ring[(i + n - 1) % n].face)
        } else {
            None
        }
    }

    /// The region a ray enters after crossing `face` at `hit`: combines the hit's crossing, the
    /// surface normal at the hit, the carrier sense and the face sides. `None` for a tangent
    /// hit, a face without a surface carrier, or an unknown face.
    fn entered_region(&self, face: EntityId, hit: &RayHit) -> Option<Region> {
        let sides = self.face_sides(face)?;
        let Carrier::Surface(sc) = self.carrier(face) else { return None };
        let along_surface_normal = match hit.crossing {
            Crossing::AlongNormal => true,
            Crossing::AgainstNormal => false,
            Crossing::Tangent => return None,
        };
        // The face normal points left -> right; moving along it enters the right region.
        let along_face_normal = along_surface_normal == (sc.sense == Sense::Forward);
        Some(if along_face_normal { sides.right } else { sides.left })
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

    /// The live cells `id` has become through splits and merges (itself if still alive; empty if
    /// deleted). Ascending, deduplicated.
    fn live_descendants(&self, id: EntityId) -> Vec<EntityId> {
        self.lineage().successors(id).into_iter().filter(|c| self.state(*c) == Some(EntityState::Live)).collect()
    }

    /// The recorded relation between two faces, if any (binary search).
    fn coincidence(&self, a: EntityId, b: EntityId) -> Option<&FacePair> {
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        let ps = self.coincident_faces(a);
        ps.binary_search_by(|p| (p.a, p.b).cmp(&(a, b))).ok().map(|i| &ps[i])
    }
}

impl<T: TopologyView + ?Sized> TopologyViewExt for T {}
