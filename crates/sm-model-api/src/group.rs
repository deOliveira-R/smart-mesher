//! The group-action core. Symmetry (deck groups) and instancing (lattices) are both *finite sets of
//! placements of one fundamental piece*, so they share one abstraction: a [`GroupAction`] yields
//! [`Placement`]s (label + rigid transform + orientation sign). Unfolding a symmetric model and
//! flattening a lattice are the same functor: `piece × placements -> cover`.
//!
//! Group identity is tolerance-based ([`GROUP_TOL`]); element order is canonical (BFS over
//! generators in declaration order), so everything downstream is deterministic.

use crate::error::{ModelError, Result};
use crate::xf::{Axis, GROUP_TOL, Plane, Turns, approx_eq, compose, inverse, translation};
use sm_core::{EntityId, EntityKind, Transform, Vector3};
use smallvec::SmallVec;

/// One placement of a fundamental piece: where it goes and whether the placement reverses
/// orientation (det < 0). Reversing placements must flip face loops and re-apply the left/right rule
/// in the cover.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Placement<L> {
    /// Canonical label of the group element / lattice slot.
    pub label: L,
    /// Image transform: piece coordinates -> cover coordinates.
    pub xf: Transform,
    /// True if `xf` is improper (mirror-like).
    pub improper: bool,
}

impl<L> Placement<L> {
    /// Build a placement; orientation is derived from the determinant.
    pub fn new(label: L, xf: Transform) -> Self {
        let improper = xf.is_improper();
        Self { label, xf, improper }
    }

    /// Compose `self` (outer) with `inner`: apply `inner` first. The nested label is the pair.
    pub fn nest<M: Clone>(&self, inner: &Placement<M>) -> Placement<(L, M)>
    where
        L: Clone,
    {
        Placement::new((self.label.clone(), inner.label.clone()), compose(&self.xf, &inner.xf))
    }
}

/// A finite action on a fundamental piece, presented as an ordered list of placements.
pub trait GroupAction {
    /// Label type, totally ordered; the placement order is ascending in this label.
    type Label: Clone + Ord;
    /// All placements in canonical order. The first is the identity placement when one exists.
    fn placements(&self) -> Result<Vec<Placement<Self::Label>>>;
}

/// Index of an element in a [`DeckGroup`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ElemIx(pub u32);

/// The family of a deck operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckKind {
    /// Reflection in a plane.
    Mirror,
    /// Rigid translation.
    Translation,
    /// Rotation about an axis by a rational number of turns.
    Rotation,
}

/// A deck transformation (group generator) in its declarative, exact-parameter form.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum DeckOp {
    /// Reflection in `plane`. Improper, self-inverse; the plane is fixed pointwise.
    Mirror {
        /// Mirror plane.
        plane: Plane,
    },
    /// Rotation about `axis` by `turns` of a full turn.
    Rotation {
        /// Rotation axis.
        axis: Axis,
        /// Angle as a rational number of turns (1/6 for a sixth-symmetry wedge).
        turns: Turns,
    },
    /// Translation by `vector`. Generates the infinite group Z, so unfolding is bounded to a window
    /// of `copies` cells (`>= 2`): translates `T^0 .. T^(copies-1)`.
    Translation {
        /// Translation vector (pairing B = A + vector).
        vector: Vector3,
        /// Number of cells in the finite unfolding window.
        copies: u32,
    },
}

impl DeckOp {
    /// The operation family.
    pub fn kind(&self) -> DeckKind {
        match self {
            DeckOp::Mirror { .. } => DeckKind::Mirror,
            DeckOp::Rotation { .. } => DeckKind::Rotation,
            DeckOp::Translation { .. } => DeckKind::Translation,
        }
    }

    /// The rigid transform of one application of the operation.
    pub fn transform(&self) -> Transform {
        match self {
            DeckOp::Mirror { plane } => plane.reflection(),
            DeckOp::Rotation { axis, turns } => axis.rotation(turns.radians()),
            DeckOp::Translation { vector, .. } => translation(*vector),
        }
    }

    /// True for mirrors (orientation-reversing).
    pub fn is_improper(&self) -> bool {
        matches!(self, DeckOp::Mirror { .. })
    }

    /// Finite order of the operation (`None` for translations).
    pub fn order(&self) -> Option<u32> {
        match self {
            DeckOp::Mirror { .. } => Some(2),
            DeckOp::Rotation { turns, .. } => Some(turns.order()),
            DeckOp::Translation { .. } => None,
        }
    }

    /// Check parameters (translation: non-zero vector, `copies >= 2`).
    pub fn check(&self) -> Result<()> {
        if let DeckOp::Translation { vector, copies } = self {
            if vector.norm() < 1e-12 {
                return Err(ModelError::Degenerate("zero translation"));
            }
            if *copies < 2 {
                return Err(ModelError::Invalid("translation window needs copies >= 2".into()));
            }
        }
        Ok(())
    }
}

/// A named generator of the deck group. `id` is a `Boundary`-kind entity that deck boundaries refer to.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckGenerator {
    /// `EntityKind::Boundary` id of the generator.
    pub id: EntityId,
    /// The operation.
    pub op: DeckOp,
}

/// Canonical label of a deck-group element: translation exponents (one per translation generator,
/// in declaration order) and the index into the finite point group. Ordered shift-major.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DeckElement {
    /// Exponent of each translation generator.
    pub shift: SmallVec<[u32; 2]>,
    /// Index into the point group (0 = identity).
    pub point: u32,
}

/// An element of a [`DeckGroup`].
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupElement {
    /// Canonical label.
    pub label: DeckElement,
    /// The transform: fundamental domain -> this copy.
    pub xf: Transform,
    /// True if the element reverses orientation.
    pub improper: bool,
}

/// Cap on the closure size, to turn non-terminating generator sets into a typed error.
pub const MAX_GROUP_ORDER: usize = 4096;

/// The finite (windowed) group generated by deck operations.
///
/// Structure: `(translation window) x (point group)`. The point group is the closure of the mirror
/// and rotation generators (BFS, right-multiplication, declaration order). Translations are applied
/// outside it as `T^k ∘ p`; this assumes translations are normalized by the point group (true when
/// each translation lies in every mirror plane and along every rotation axis; the validator warns
/// otherwise).
#[derive(Clone, Debug)]
pub struct DeckGroup {
    generators: Vec<DeckGenerator>,
    elements: Vec<GroupElement>,
    point_order: usize,
}

impl DeckGroup {
    /// Generate the group. Errors with [`ModelError::GroupTooLarge`] if the point group does not
    /// close within [`MAX_GROUP_ORDER`] elements.
    pub fn generate(generators: &[DeckGenerator]) -> Result<Self> {
        let mut pts: Vec<Transform> = vec![Transform::IDENTITY];
        let mut pgens: Vec<Transform> = Vec::new();
        let mut tgens: Vec<(Vector3, u32)> = Vec::new();
        for g in generators {
            g.op.check()?;
            match g.op {
                DeckOp::Translation { vector, copies } => tgens.push((vector, copies)),
                _ => pgens.push(g.op.transform()),
            }
        }
        let mut i = 0;
        while i < pts.len() {
            for g in &pgens {
                let n = compose(&pts[i], g);
                if !pts.iter().any(|p| approx_eq(p, &n, GROUP_TOL)) {
                    if pts.len() >= MAX_GROUP_ORDER {
                        return Err(ModelError::GroupTooLarge { limit: MAX_GROUP_ORDER });
                    }
                    pts.push(n);
                }
            }
            i += 1;
        }
        let total: usize = tgens.iter().try_fold(pts.len(), |acc, (_, c)| acc.checked_mul(*c as usize)).unwrap_or(usize::MAX);
        if total > MAX_GROUP_ORDER {
            return Err(ModelError::GroupTooLarge { limit: MAX_GROUP_ORDER });
        }
        let mut elements = Vec::with_capacity(total);
        let mut shift: SmallVec<[u32; 2]> = SmallVec::from_elem(0, tgens.len());
        loop {
            let mut tv = Vector3::new(0.0, 0.0, 0.0);
            for (s, (v, _)) in shift.iter().zip(&tgens) {
                tv = tv + *v * f64::from(*s);
            }
            let tx = translation(tv);
            for (pi, p) in pts.iter().enumerate() {
                let xf = compose(&tx, p);
                elements.push(GroupElement {
                    label: DeckElement { shift: shift.clone(), point: pi as u32 },
                    improper: xf.is_improper(),
                    xf,
                });
            }
            // odometer, last generator fastest
            let mut k = tgens.len();
            loop {
                if k == 0 {
                    return Ok(Self { generators: generators.to_vec(), point_order: pts.len(), elements });
                }
                k -= 1;
                shift[k] += 1;
                if shift[k] < tgens[k].1 {
                    break;
                }
                shift[k] = 0;
            }
        }
    }

    /// The generators this group was built from.
    pub fn generators(&self) -> &[DeckGenerator] {
        &self.generators
    }
    /// Number of elements (window size x point-group order).
    pub fn order(&self) -> usize {
        self.elements.len()
    }
    /// Order of the finite point group alone.
    pub fn point_order(&self) -> usize {
        self.point_order
    }
    /// All elements in canonical order; element 0 is the identity.
    pub fn elements(&self) -> &[GroupElement] {
        &self.elements
    }
    /// The identity element.
    pub fn identity(&self) -> ElemIx {
        ElemIx(0)
    }
    /// Look up an element by index.
    pub fn element(&self, i: ElemIx) -> Option<&GroupElement> {
        self.elements.get(i.0 as usize)
    }
    /// Find the element whose transform equals `xf` (within [`GROUP_TOL`]).
    pub fn find(&self, xf: &Transform) -> Option<ElemIx> {
        self.elements.iter().position(|e| approx_eq(&e.xf, xf, GROUP_TOL)).map(|i| ElemIx(i as u32))
    }
    /// Product `a ∘ b` if it lies inside the (windowed) group.
    pub fn mul(&self, a: ElemIx, b: ElemIx) -> Option<ElemIx> {
        let (ea, eb) = (self.element(a)?, self.element(b)?);
        self.find(&compose(&ea.xf, &eb.xf))
    }
    /// Inverse of `a` if it lies inside the window.
    pub fn inverse_of(&self, a: ElemIx) -> Option<ElemIx> {
        let inv = inverse(&self.element(a)?.xf)?;
        self.find(&inv)
    }
}

impl GroupAction for DeckGroup {
    type Label = DeckElement;
    fn placements(&self) -> Result<Vec<Placement<DeckElement>>> {
        Ok(self.elements.iter().map(|e| Placement { label: e.label.clone(), xf: e.xf, improper: e.improper }).collect())
    }
}

/// Convenience: build a [`DeckGenerator`] with a freshly allocated `Boundary` id.
pub fn new_generator(ids: &mut dyn crate::ids::IdAllocator, op: DeckOp) -> Result<DeckGenerator> {
    op.check()?;
    Ok(DeckGenerator { id: ids.alloc(EntityKind::Boundary), op })
}
