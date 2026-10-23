//! Symmetry as an orbifold: a **fundamental domain** `F`, a deck group `G` acting on it, and boundary
//! identifications (the deck boundaries in [`crate::boundary`]). The full model is `⋃ g·F`.
//!
//! - **Cut and keep a chamber** is a two-phase protocol, because cutting is a topology operation and
//!   this layer only reads topology: [`plan_cut`] turns a [`CutSpec`] into a [`CutPlan`] (half-spaces
//!   to keep, the generator, which cap plane plays which deck role); the topology engine executes the
//!   cut and returns a [`CutReport`] (lineage + cap faces); [`crate::model::Model::apply_cut`] then
//!   inherits colors, tags the cap faces as deck boundaries and records the symmetry.
//! - **Unfold** is a functor: [`Symmetry::unfold`] gives a [`Cover`], the derived full model. It
//!   stores no topology: cover entities are pairs `(base entity, group element)`, colors are pulled
//!   back along the projection, deck faces glue to their partner in the neighbouring copy, and
//!   improper elements report that face loops must be reversed. [`Cover::refold`] returns the exact
//!   [`Symmetry`] it came from, so unfold is reversible.

use crate::boundary::{BoundaryKind, BoundaryTable, DeckSide, DeckSides};
use crate::coloring::Coloring;
use crate::detect::Verification;
use crate::error::{ModelError, Result};
use crate::group::{DeckGenerator, DeckGroup, DeckOp, ElemIx, GroupAction, GroupElement, Placement};
use crate::topo::{GeometryView, Moments};
use crate::xf::{Axis, Plane, Tolerance, Turns, approx_eq, compose, inverse, normalized};
use sm_core::{EntityId, Point3, Vector3};

/// Closed half-space `{x : n·(x - p) >= 0}`, a piece of a chamber.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HalfSpace(pub Plane);

impl HalfSpace {
    /// True if `p` is inside (within `tol` of the boundary counts as inside).
    pub fn contains(&self, p: Point3, tol: f64) -> bool {
        self.0.signed_distance(p) >= -tol
    }
}

/// Which side of a plane to keep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    /// The side the plane normal points to.
    Positive,
    /// The opposite side.
    Negative,
}

/// Whether the stored model is the fundamental domain or the full model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DomainState {
    /// The stored model is the full model; the group is a property of it (e.g. just detected).
    Full,
    /// The stored model is the fundamental domain `F`; the full model is derived by unfolding.
    Reduced,
}

/// The fundamental domain.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FundamentalDomain {
    /// Full or reduced.
    pub state: DomainState,
    /// Chamber extent as an intersection of half-spaces, when known (set by cuts).
    pub chamber: Vec<HalfSpace>,
}

/// How a symmetry came to be.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SymmetryOrigin {
    /// Declared by the user.
    Declared,
    /// Produced by cut-and-keep-chamber.
    Cut,
    /// Found by detection, with the verification result.
    Detected(Verification),
}

/// Deck group generators + fundamental domain.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Symmetry {
    /// Generators in declaration order (this order fixes element order).
    pub generators: Vec<DeckGenerator>,
    /// The fundamental domain.
    pub domain: FundamentalDomain,
    /// Provenance.
    pub origin: SymmetryOrigin,
}

impl Symmetry {
    /// The (windowed) group generated.
    pub fn group(&self) -> Result<DeckGroup> {
        DeckGroup::generate(&self.generators)
    }

    /// Generator by id.
    pub fn generator(&self, id: EntityId) -> Option<&DeckGenerator> {
        self.generators.iter().find(|g| g.id == id)
    }

    /// Unfold the reduced model into its derived full model. Fails if the stored model is already
    /// the full model.
    pub fn unfold(&self) -> Result<Cover> {
        if self.domain.state != DomainState::Reduced {
            return Err(ModelError::State("model is not reduced; nothing to unfold"));
        }
        Ok(Cover { group: self.group()?, symmetry: self.clone() })
    }
}

/// What a cut cap plays in the deck pairing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CapRole {
    /// Mirror cap: self-paired.
    SelfPaired,
    /// Source side `A` of a paired cut.
    SideA,
    /// Image side `B = g(A)`.
    SideB,
}

/// A request to cut the model and keep one chamber.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CutSpec {
    /// Mirror plane; keep one side. Generator: reflection.
    Mirror {
        /// The mirror plane.
        plane: Plane,
        /// Side to keep.
        keep: Side,
    },
    /// `1/n` wedge about `axis` (`n >= 3`), the sector from direction `start` (projected
    /// perpendicular to the axis) counter-clockwise (right-handed about the axis) by `2π/n`.
    /// Generator: rotation by `1/n` turn; `A` is the start plane, `B = R(A)` the end plane.
    Wedge {
        /// Rotation axis.
        axis: Axis,
        /// Fold number (`n >= 3`; use `Mirror` for 2).
        n: u32,
        /// Direction of the start plane (any vector not parallel to the axis).
        start: Vector3,
    },
    /// Periodic slab between `plane` and `plane + vector`. Generator: translation by `vector`,
    /// unfolded to `copies` cells.
    Slab {
        /// Start plane (`A`).
        plane: Plane,
        /// Period vector (must not be parallel to the plane).
        vector: Vector3,
        /// Window size of the unfolding.
        copies: u32,
    },
}

impl CutSpec {
    /// Derive a cut from a deck operation (e.g. a detected candidate). Rotations need
    /// `turns = 1/n`; `start_hint` picks the start plane of a wedge. Translations are not cut this
    /// way (a finite model has no translation symmetry to detect).
    pub fn from_op(op: &DeckOp, keep: Side, start_hint: Vector3) -> Result<Self> {
        match *op {
            DeckOp::Mirror { plane } => Ok(CutSpec::Mirror { plane, keep }),
            DeckOp::Rotation { axis, turns } if turns.num == 1 => Ok(CutSpec::Wedge { axis, n: turns.den, start: start_hint }),
            DeckOp::Rotation { .. } => Err(ModelError::Invalid("wedge cut needs a 1/n-turn rotation".into())),
            DeckOp::Translation { vector, copies } => {
                let n = normalized(vector)?;
                Ok(CutSpec::Slab { plane: Plane::new(Point3::new(0.0, 0.0, 0.0), n)?, vector, copies })
            }
        }
    }
}

/// One cutting plane of a plan: keep its positive half-space; faces created on it get `role`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutPlane {
    /// Keep the side `n·(x-p) >= 0`.
    pub keep: HalfSpace,
    /// Role of the cap faces created on this plane.
    pub role: CapRole,
}

/// Executable description of a cut for the topology engine, plus the semantics to apply afterwards.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutPlan {
    /// The deck generator the cut introduces.
    pub generator: DeckGenerator,
    /// Cutting planes; the chamber is the intersection of their kept half-spaces.
    pub planes: Vec<CutPlane>,
}

/// Plan a cut: compute the kept half-spaces and the deck roles. Pure geometry, no topology.
pub fn plan_cut(spec: &CutSpec, ids: &mut dyn crate::ids::IdAllocator) -> Result<CutPlan> {
    let (op, planes) = match *spec {
        CutSpec::Mirror { plane, keep } => {
            let kept = if keep == Side::Positive { plane } else { plane.flipped() };
            (DeckOp::Mirror { plane }, vec![CutPlane { keep: HalfSpace(kept), role: CapRole::SelfPaired }])
        }
        CutSpec::Wedge { axis, n, start } => {
            if n < 3 {
                return Err(ModelError::Invalid("wedge needs n >= 3; use a mirror cut for n = 2".into()));
            }
            let d = axis.dir();
            let s = normalized(start - d * start.dot(d))?;
            let u = d.cross(s);
            let theta = std::f64::consts::TAU / f64::from(n);
            let (st, ct) = theta.sin_cos();
            let p = axis.point();
            let n0 = u;
            let n1 = s * st - u * ct;
            (
                DeckOp::Rotation { axis, turns: Turns::one_over(n)? },
                vec![
                    CutPlane { keep: HalfSpace(Plane::new(p, n0)?), role: CapRole::SideA },
                    CutPlane { keep: HalfSpace(Plane::new(p, n1)?), role: CapRole::SideB },
                ],
            )
        }
        CutSpec::Slab { plane, vector, copies } => {
            let along = plane.normal().dot(vector);
            if along.abs() < 1e-12 {
                return Err(ModelError::Degenerate("period vector parallel to the slab plane"));
            }
            let a = if along > 0.0 { plane } else { plane.flipped() };
            let b = Plane::new(a.point() + vector, -a.normal())?;
            (
                DeckOp::Translation { vector, copies },
                vec![
                    CutPlane { keep: HalfSpace(a), role: CapRole::SideA },
                    CutPlane { keep: HalfSpace(b), role: CapRole::SideB },
                ],
            )
        }
    };
    op.check()?;
    Ok(CutPlan { generator: crate::group::new_generator(ids, op)?, planes })
}

/// `old -> new` lineage entry: `new` descends from `old` (they may be the same id when the entity
/// survives a clip unchanged). A split yields several entries for one `old`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lineage {
    /// Ancestor.
    pub old: EntityId,
    /// Descendant.
    pub new: EntityId,
}

/// A face created by the cut, lying on plane `plane` of the [`CutPlan`] (index into `planes`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CapFace {
    /// The new face.
    pub face: EntityId,
    /// Index of the cutting plane in [`CutPlan::planes`].
    pub plane: usize,
}

/// What the topology engine reports after executing a [`CutPlan`].
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CutReport {
    /// Volume lineage.
    pub volumes: Vec<Lineage>,
    /// Face lineage (excluding caps, which have no ancestor).
    pub faces: Vec<Lineage>,
    /// Volumes that no longer exist (outside the chamber).
    pub removed_volumes: Vec<EntityId>,
    /// Faces that no longer exist.
    pub removed_faces: Vec<EntityId>,
    /// New cap faces and the plane they lie on.
    pub caps: Vec<CapFace>,
}

/// Build the deck face sets from a cut report.
pub(crate) fn sides_from_caps(plan: &CutPlan, report: &CutReport) -> Result<DeckSides> {
    let mut selfp = std::collections::BTreeSet::new();
    let (mut a, mut b) = (std::collections::BTreeSet::new(), std::collections::BTreeSet::new());
    for c in &report.caps {
        let p = plan.planes.get(c.plane).ok_or_else(|| ModelError::Invalid(format!("cap plane index {} out of range", c.plane)))?;
        match p.role {
            CapRole::SelfPaired => selfp.insert(c.face),
            CapRole::SideA => a.insert(c.face),
            CapRole::SideB => b.insert(c.face),
        };
    }
    Ok(if plan.planes.iter().all(|p| p.role == CapRole::SelfPaired) { DeckSides::SelfPaired(selfp) } else { DeckSides::Paired { a, b } })
}

/// An entity of the unfolded model: the base entity in the fundamental domain and the group element
/// that carries it to this copy. `π(CoverEntity) = base`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoverEntity {
    /// Entity in the fundamental domain.
    pub base: EntityId,
    /// Group element index.
    pub element: ElemIx,
}

/// Result of asking where a face of copy `e` is glued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Glue {
    /// Not a deck face: nothing to glue (a genuine boundary or interior face of the copy).
    NotDeck,
    /// Deck face whose partner copy lies outside the group window: the cover stays open here.
    Open,
    /// Deck face identified with the partner side in copy `element`.
    Glued {
        /// The deck boundary record.
        boundary: EntityId,
        /// Partner copy.
        element: ElemIx,
        /// Which side of the identification the partner face is on.
        side: DeckSide,
    },
}

/// The derived full model of a reduced [`Symmetry`] (a covering of the fundamental domain).
#[derive(Clone, Debug)]
pub struct Cover {
    group: DeckGroup,
    symmetry: Symmetry,
}

impl Cover {
    /// The group acting.
    pub fn group(&self) -> &DeckGroup {
        &self.group
    }

    /// Number of copies of the fundamental domain.
    pub fn copies(&self) -> usize {
        self.group.order()
    }

    /// Element data.
    pub fn element(&self, e: ElemIx) -> Option<&GroupElement> {
        self.group.element(e)
    }

    /// Entities of the cover for the given base entities, element-major then base order.
    pub fn entities(&self, base: &[EntityId]) -> Vec<CoverEntity> {
        let mut out = Vec::with_capacity(base.len() * self.copies());
        for i in 0..self.copies() {
            for &b in base {
                out.push(CoverEntity { base: b, element: ElemIx(i as u32) });
            }
        }
        out
    }

    /// Covering projection `π`.
    pub fn project(&self, e: CoverEntity) -> EntityId {
        e.base
    }

    /// Coloring pulled back along `π`: a copy has the color of its base entity.
    pub fn volume_color(&self, coloring: &Coloring, e: CoverEntity) -> Option<EntityId> {
        coloring.volume_color(e.base)
    }

    /// Face color pulled back along `π`.
    pub fn face_color(&self, coloring: &Coloring, e: CoverEntity) -> Option<EntityId> {
        coloring.face_color(e.base)
    }

    /// True if this copy is a reflected one, so its face loops must be reversed (and the
    /// left/right rule re-applied) when the cover is materialized.
    pub fn reverses_orientation(&self, element: ElemIx) -> bool {
        self.group.element(element).is_some_and(|e| e.improper)
    }

    /// Where the deck face `face` of copy `element` is glued. Derivation: with `b = g(a)`, face `a`
    /// of copy `e` coincides with face `b` of copy `e∘g⁻¹`, and face `b` of copy `e` coincides with
    /// face `a` of copy `e∘g`. A mirror pairs `a` with itself in copy `e∘g`.
    pub fn glue(&self, boundaries: &BoundaryTable, face: EntityId, element: ElemIx) -> Glue {
        let Some(BoundaryKind::Deck { boundary, side }) = boundaries.kind_of(face) else { return Glue::NotDeck };
        let (Some(deck), Some(el)) = (boundaries.deck(*boundary), self.group.element(element)) else { return Glue::Open };
        let Some(gen_) = self.symmetry.generator(deck.generator) else { return Glue::Open };
        let gx = gen_.op.transform();
        let (target, partner_side) = match side {
            DeckSide::SelfPaired => (compose(&el.xf, &gx), DeckSide::SelfPaired),
            DeckSide::A => match inverse(&gx) {
                Some(gi) => (compose(&el.xf, &gi), DeckSide::B),
                None => return Glue::Open,
            },
            DeckSide::B => (compose(&el.xf, &gx), DeckSide::A),
        };
        match self.group.find(&target) {
            Some(e2) => Glue::Glued { boundary: *boundary, element: e2, side: partner_side },
            None => Glue::Open,
        }
    }

    /// Deck faces of the cover that stay open (their partner copy is outside the window), in
    /// element-major order. Empty for a closed unfolding (e.g. a full 1/6 rotation).
    pub fn open_deck_faces(&self, boundaries: &BoundaryTable) -> Vec<CoverEntity> {
        let mut out = Vec::new();
        for i in 0..self.copies() {
            for (f, k) in boundaries.iter() {
                if matches!(k, BoundaryKind::Deck { .. }) && self.glue(boundaries, f, ElemIx(i as u32)) == Glue::Open {
                    out.push(CoverEntity { base: f, element: ElemIx(i as u32) });
                }
            }
        }
        out
    }

    /// Total moments of the cover: the base volumes' moments summed over all copies. For a closed
    /// unfolding this must equal the moments of the original full model.
    pub fn moments(&self, topo: &dyn GeometryView) -> Moments {
        let mut total = Moments::ZERO;
        for v in topo.volumes() {
            if let Some(m) = topo.volume_moments(v) {
                for e in self.group.elements() {
                    total = total.add(&m.transformed(&e.xf));
                }
            }
        }
        total
    }

    /// Reverse the unfolding: return the exact [`Symmetry`] (fundamental domain + generators).
    pub fn refold(self) -> Symmetry {
        self.symmetry
    }
}

impl GroupAction for Cover {
    type Label = crate::group::DeckElement;
    fn placements(&self) -> Result<Vec<Placement<Self::Label>>> {
        self.group.placements()
    }
}

/// Partner of `face` under deck boundary `deck`: the face on the other side whose centroid is the
/// image (or preimage) of `face`'s centroid within `tol`. Mirror faces are their own partner.
pub fn deck_partner(
    topo: &dyn GeometryView,
    op: &DeckOp,
    sides: &DeckSides,
    side: DeckSide,
    face: EntityId,
    tol: &Tolerance,
) -> Option<EntityId> {
    let c = topo.face_centroid(face)?;
    let x = op.transform();
    match (sides, side) {
        (DeckSides::SelfPaired(_), _) => Some(face),
        (DeckSides::Paired { b, .. }, DeckSide::A) => {
            let target = x.apply_point(c);
            b.iter().copied().find(|&q| topo.face_centroid(q).is_some_and(|qc| (qc - target).norm() <= tol.linear))
        }
        (DeckSides::Paired { a, .. }, _) => {
            let xi = inverse(&x)?;
            let target = xi.apply_point(c);
            a.iter().copied().find(|&q| topo.face_centroid(q).is_some_and(|qc| (qc - target).norm() <= tol.linear))
        }
    }
}

/// True if the two generator transforms coincide (used to reuse a generator when a cut repeats one).
pub(crate) fn same_transform(a: &DeckOp, b: &DeckOp) -> bool {
    a.kind() == b.kind() && approx_eq(&a.transform(), &b.transform(), crate::xf::GROUP_TOL)
}
