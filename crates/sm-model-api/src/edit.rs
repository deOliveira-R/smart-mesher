//! Row editing (W0f gaps 2): change or remove boundary, instance, prototype and lattice rows,
//! and carry the model's geometric parameters through a rigid motion of volumes
//! ([`Model::apply_transform`]).
//!
//! Removals are refused with [`ModelError::InUse`] while anything still refers to the row, so the
//! tables never dangle and [`Model::check_integrity`] keeps holding.

use crate::boundary::{BoundaryKind, Deck, DeckSide, FaceRole, InterfaceIntent};
use crate::decomposition::{CutGeometry, PieceRef};
use crate::error::{ModelError, Result, expect_kind};
use crate::instancing::{Fill, InstanceRow, PathStep};
use crate::model::Model;
use crate::xf::{Axis, Plane, pt};
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use sm_topo_api::{Dim, TopologyView};
use std::collections::BTreeSet;

/// Relative tolerance of the similarity test in [`Model::apply_transform`].
const SIMILARITY_TOL: f64 = 1e-9;

fn role_fits(kind: &BoundaryKind, role: &FaceRole) -> bool {
    matches!(
        (kind, role),
        (BoundaryKind::Deck(Deck::Mirror { .. }), FaceRole::Deck { side: DeckSide::SelfPaired, .. })
            | (BoundaryKind::Deck(Deck::Translation { .. } | Deck::Rotation { .. }), FaceRole::Deck { side: DeckSide::A | DeckSide::B, .. })
            | (BoundaryKind::Response, FaceRole::Response)
            | (BoundaryKind::Interface(InterfaceIntent::Conformal), FaceRole::Interface { side: DeckSide::A })
            | (BoundaryKind::Interface(InterfaceIntent::NonConformal { .. }), FaceRole::Interface { .. })
    )
}

/// `M^T M = s^2 I` within [`SIMILARITY_TOL`] (rotation, reflection and uniform scale).
fn is_similarity(x: &Transform) -> bool {
    let col = |j: usize| Vector3::new(x.m[0][j], x.m[1][j], x.m[2][j]);
    let c = [col(0), col(1), col(2)];
    let s2 = c[0].dot(c[0]);
    if !(s2.is_finite() && s2 > 0.0) {
        return false;
    }
    (0..3).all(|i| (0..3).all(|j| (c[i].dot(c[j]) - if i == j { s2 } else { 0.0 }).abs() <= SIMILARITY_TOL * s2))
}

/// The image of an oriented plane under an invertible affine map.
fn map_plane(p: &Plane, x: &Transform, inv: &Transform) -> Result<Plane> {
    // n' = M^-T n; a point of the plane maps to a point of the image plane.
    let n = p.normal;
    let m = &inv.m;
    let nn = Vector3::new(m[0][0] * n.x + m[1][0] * n.y + m[2][0] * n.z, m[0][1] * n.x + m[1][1] * n.y + m[2][1] * n.z, m[0][2] * n.x + m[1][2] * n.y + m[2][2] * n.z);
    let q = x.apply_point(pt(n * p.offset));
    Plane::through(q, nn)
}

fn map_deck(d: &Deck, x: &Transform, inv: &Transform) -> Result<Deck> {
    let out = match d {
        Deck::Translation { vector } => Deck::Translation { vector: x.apply_vector(*vector) },
        _ if !is_similarity(x) => return Err(ModelError::Invalid("mirror and rotation decks move only under a similarity (rotation, reflection, uniform scale)".into())),
        Deck::Mirror { plane } => Deck::Mirror { plane: map_plane(plane, x, inv)? },
        Deck::Rotation { axis, angle } => {
            // Conjugating by an improper map reverses the sense of rotation: flip the axis.
            let sign = if x.is_improper() { -1.0 } else { 1.0 };
            let dir = x.apply_vector(axis.dir) * sign;
            Deck::Rotation { axis: Axis::new(x.apply_point(axis.point), dir)?, angle: *angle }
        }
    };
    out.check()?;
    Ok(out)
}

impl Model {
    /// The first group listing `id`, if any.
    fn group_referring(&self, id: EntityId) -> Option<EntityId> {
        self.groups.values().find(|g| g.members.contains(&id)).map(|g| g.id)
    }

    /// Every fill of the instancing tables (instances, lattice outers and slots).
    fn fills(&self) -> impl Iterator<Item = Fill> + '_ {
        let ins = &self.instancing;
        ins.instances.values().map(|i| i.of).chain(ins.lattices.values().flat_map(|l| l.slots.values().map(|s| s.fill).chain([l.outer])))
    }

    /// Change the kind of boundary record `b`. Deck parameters are validated. Refused
    /// ([`ModelError::Invalid`]) when an assigned face's role does not fit the new kind (a mirror
    /// takes self-paired faces, a translation or rotation A/B pairs, a conformal interface side
    /// A only), or when `b` generates the declared symmetry and the new kind is not a deck.
    pub fn set_boundary_kind(&mut self, b: EntityId, kind: BoundaryKind) -> Result<()> {
        expect_kind(b, EntityKind::Boundary)?;
        if !self.boundaries.contains_key(&b) {
            return Err(ModelError::Unknown(b));
        }
        if let BoundaryKind::Deck(d) = &kind {
            d.check()?;
        }
        if let Some((f, _)) = self.boundary_faces(b).find(|(_, fb)| !role_fits(&kind, &fb.role)) {
            return Err(ModelError::Invalid(format!("face {f} of {b} has a role that does not fit the new kind; unassign it first")));
        }
        if kind.as_deck().is_none() && self.symmetry.as_ref().is_some_and(|s| s.generators.contains(&b)) {
            return Err(ModelError::Invalid(format!("{b} generates the declared symmetry and must stay a deck")));
        }
        if let Some(row) = self.boundaries.get_mut(&b) {
            row.kind = kind;
        }
        Ok(())
    }

    /// Remove boundary record `b`. Refused with [`ModelError::InUse`] while faces are assigned to
    /// it, while it generates the declared symmetry, or while a group lists it.
    pub fn remove_boundary(&mut self, b: EntityId) -> Result<()> {
        expect_kind(b, EntityKind::Boundary)?;
        if !self.boundaries.contains_key(&b) {
            return Err(ModelError::Unknown(b));
        }
        if self.boundary_faces(b).next().is_some() || self.symmetry.as_ref().is_some_and(|s| s.generators.contains(&b)) || self.group_referring(b).is_some() {
            return Err(ModelError::InUse(b));
        }
        self.boundaries.remove(&b);
        Ok(())
    }

    /// Replace the row of instance `id` (fill, placement, recoloring, clip), with the same
    /// checks as [`Model::add_instance`]. The id is kept.
    pub fn set_instance(&mut self, id: EntityId, row: InstanceRow) -> Result<()> {
        expect_kind(id, EntityKind::Instance)?;
        if !self.instancing.instances.contains_key(&id) {
            return Err(ModelError::Unknown(id));
        }
        self.check_fill(row.of)?;
        self.check_color_map(&row.recolor)?;
        Self::check_clip(row.clip)?;
        self.instancing.instances.insert(id, row);
        Ok(())
    }

    /// Remove instance `id`. Refused with [`ModelError::InUse`] while a group or a path override
    /// names it. The id is not reused.
    pub fn remove_instance(&mut self, id: EntityId) -> Result<()> {
        expect_kind(id, EntityKind::Instance)?;
        if !self.instancing.instances.contains_key(&id) {
            return Err(ModelError::Unknown(id));
        }
        let in_override = self.instancing.overrides.iter().any(|o| o.prefix.contains(&PathStep::Instance(id)));
        if in_override || self.group_referring(id).is_some() {
            return Err(ModelError::InUse(id));
        }
        self.instancing.instances.remove(&id);
        Ok(())
    }

    /// Remove prototype `id`. Refused with [`ModelError::InUse`] while any fill (instance,
    /// lattice outer or slot) or group refers to it.
    pub fn remove_prototype(&mut self, id: EntityId) -> Result<()> {
        expect_kind(id, EntityKind::Prototype)?;
        if !self.instancing.prototypes.contains_key(&id) {
            return Err(ModelError::Unknown(id));
        }
        if self.fills().any(|f| f == Fill::Prototype(id)) || self.group_referring(id).is_some() {
            return Err(ModelError::InUse(id));
        }
        self.instancing.prototypes.remove(&id);
        Ok(())
    }

    /// Remove lattice `id`. Refused with [`ModelError::InUse`] while any fill, group or path
    /// override (a slot step of this lattice) refers to it.
    pub fn remove_lattice(&mut self, id: EntityId) -> Result<()> {
        expect_kind(id, EntityKind::Lattice)?;
        if !self.instancing.lattices.contains_key(&id) {
            return Err(ModelError::Unknown(id));
        }
        let in_override = self.instancing.overrides.iter().any(|o| o.prefix.iter().any(|s| matches!(s, PathStep::Slot { lattice, .. } if *lattice == id)));
        if in_override || self.fills().any(|f| f == Fill::Lattice(id)) || self.group_referring(id).is_some() {
            return Err(ModelError::InUse(id));
        }
        self.instancing.lattices.remove(&id);
        Ok(())
    }

    /// The volumes a piece covers (a composite: all of its members').
    fn piece_volumes(&self, p: PieceRef, out: &mut BTreeSet<EntityId>, depth: usize) {
        match p {
            PieceRef::Whole(v) => {
                out.insert(v);
            }
            PieceRef::Cell(c) => match self.decomposition.cells.get(&c) {
                Some(pc) if !pc.members.is_empty() && depth < 64 => pc.members.iter().for_each(|m| self.piece_volumes(*m, out, depth + 1)),
                Some(pc) => {
                    out.insert(pc.volume);
                }
                None => {}
            },
        }
    }

    /// Carry the model's geometric parameters through the motion `x` of `volumes` (the topology
    /// already moved, or is about to, by the same `x`; `t` is used only to find the volumes'
    /// faces and whether the whole model moves). Transactional: on error nothing changes.
    ///
    /// - A boundary whose assigned faces all bound moved volumes gets its parameters mapped:
    ///   deck plane, vector and axis (`x g x^-1`), and a non-conformal interface transform.
    ///   Mirror and rotation decks need `x` to be a similarity. A record with faces on both
    ///   moved and unmoved volumes is refused with [`ModelError::Straddles`].
    /// - The `CutGeometry::Plane` of every split of a piece over moved volumes is mapped; a
    ///   composite piece over moved and unmoved volumes is refused with `Straddles`.
    /// - The declared symmetry is mapped (through its generator decks) only when every live
    ///   volume of `t` moves; then every deck and cut plane maps regardless of face membership.
    ///   A partial move that touches its domain or a generator is refused with
    ///   [`ModelError::SymmetryStraddles`].
    ///
    /// Instance placements, lattice origins and prototype membership are not changed (moving a
    /// prototype volume moves every copy of it).
    pub fn apply_transform(&mut self, x: &Transform, volumes: &[EntityId], t: &dyn TopologyView) -> Result<()> {
        let inv = x.inverse().ok_or(ModelError::Singular)?;
        let mut moved: BTreeSet<EntityId> = BTreeSet::new();
        for v in volumes {
            expect_kind(*v, EntityKind::TopoVolume)?;
            if !t.is_alive(*v) {
                return Err(ModelError::Unknown(*v));
            }
            moved.insert(*v);
        }
        let whole = t.cells(Dim::Volume).iter().all(|v| moved.contains(v));
        let moved_faces: BTreeSet<EntityId> = moved.iter().flat_map(|v| t.volume_faces(*v).iter().copied()).collect();
        let mut next_kinds: Vec<(EntityId, BoundaryKind)> = Vec::new();
        let mut touched_boundaries: BTreeSet<EntityId> = BTreeSet::new();
        for row in self.boundaries.values() {
            let mut any = false;
            let mut all = true;
            for (f, _) in self.boundary_faces(row.id) {
                let m = moved_faces.contains(&f);
                any |= m;
                all &= m;
            }
            let go = if whole {
                true
            } else if any && !all {
                return Err(ModelError::Straddles(row.id));
            } else {
                any
            };
            if !go {
                continue;
            }
            touched_boundaries.insert(row.id);
            let kind = match &row.kind {
                BoundaryKind::Deck(d) => BoundaryKind::Deck(map_deck(d, x, &inv)?),
                BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: Some(tr) }) => BoundaryKind::Interface(InterfaceIntent::NonConformal { transform: Some(x.compose(tr).compose(&inv)) }),
                k => *k,
            };
            next_kinds.push((row.id, kind));
        }
        if let Some(s) = &self.symmetry
            && !whole
            && (s.domain.iter().any(|v| moved.contains(v)) || s.generators.iter().any(|g| touched_boundaries.contains(g)))
        {
            return Err(ModelError::SymmetryStraddles);
        }
        let mut next_cuts: Vec<(PieceRef, Plane)> = Vec::new();
        for (p, rec) in &self.decomposition.splits {
            let CutGeometry::Plane(pl) = &rec.geometry else { continue };
            let mut vols = BTreeSet::new();
            self.piece_volumes(*p, &mut vols, 0);
            let n = vols.iter().filter(|v| moved.contains(v)).count();
            if n == 0 && !whole {
                continue;
            }
            if n != vols.len() && !whole {
                let id = match p {
                    PieceRef::Whole(v) | PieceRef::Cell(v) => *v,
                };
                return Err(ModelError::Straddles(id));
            }
            next_cuts.push((*p, map_plane(pl, x, &inv)?));
        }
        for (b, k) in next_kinds {
            if let Some(row) = self.boundaries.get_mut(&b) {
                row.kind = k;
            }
        }
        for (p, pl) in next_cuts {
            if let Some(rec) = self.decomposition.splits.get_mut(&p) {
                rec.geometry = CutGeometry::Plane(pl);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_maps_under_affine_maps() {
        let p = Plane::new(Vector3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let x = Transform::translation(Vector3::new(1.0, 2.0, 3.0));
        let q = map_plane(&p, &x, &x.inverse().unwrap()).unwrap();
        assert!((q.offset - 5.0).abs() < 1e-12 && (q.normal.z - 1.0).abs() < 1e-12);
        // Non-uniform scale: plane x + z = 1 under diag(2, 1, 1) is x/2 + z = 1.
        let s = Transform { m: [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
        let p = Plane::new(Vector3::new(1.0, 0.0, 1.0), 1.0).unwrap();
        let q = map_plane(&p, &s, &s.inverse().unwrap()).unwrap();
        for pnt in [sm_core::Point3::new(2.0, 0.0, 0.0), sm_core::Point3::new(0.0, 5.0, 1.0)] {
            assert!(q.signed_distance(pnt).abs() < 1e-12);
        }
        assert!(!is_similarity(&s));
        assert!(is_similarity(&Transform::reflection(Vector3::new(1.0, 1.0, 0.0), 0.3).unwrap()));
    }
}
