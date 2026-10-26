//! Boundary conditions on surfaces. They are data of the world; the transport code applies them
//! through [`crate::World::apply_boundary`], so reflect/periodic arithmetic lives in one place.

use crate::handle::{SurfHandle, VolHandle};
use sm_core::{Point3, Transform, Vector3};

/// What happens to a particle that reaches a surface.
///
/// Source: `sm_model_api::BoundaryKind::Deck` lowers to `Reflecting` (mirror) or `Periodic`
/// (translation/rotation); response tags named "vacuum"/"reflective" are lowered by the deck
/// writer; everything else is `Interior`.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum Boundary {
    /// Ordinary surface between two volumes; the particle crosses into the next volume.
    #[default]
    Interior,
    /// Particle leaves the problem.
    Vacuum,
    /// Specular reflection.
    Reflecting,
    /// Diffuse (white) return: the caller samples a cosine direction about the inward normal.
    White,
    /// Particle re-enters through `partner` after applying `map` (maps this surface onto the
    /// partner; proper or improper).
    Periodic {
        /// The paired surface.
        partner: SurfHandle,
        /// Maps points and directions on this side to the partner side.
        map: Transform,
    },
}

/// What the transport code does next, computed by [`crate::World::apply_boundary`].
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum BoundaryAction {
    /// Continue in `volume` with the same direction.
    Continue {
        /// The volume entered.
        volume: VolHandle,
    },
    /// The particle leaked.
    Leak,
    /// Continue in the same volume with a new direction; the history should be
    /// `reset_to_last_intersection`.
    Reflect {
        /// Reflected unit direction.
        dir: Vector3,
    },
    /// Sample a cosine-weighted direction about `inward_normal` and continue in `volume`.
    White {
        /// Unit normal pointing into `volume`.
        inward_normal: Vector3,
        /// The volume the particle stays in.
        volume: VolHandle,
    },
    /// Re-enter through the partner surface.
    Periodic {
        /// New position (on the partner surface).
        pos: Point3,
        /// New unit direction.
        dir: Vector3,
        /// The partner surface (put it in the history).
        surface: SurfHandle,
        /// The volume entered.
        volume: VolHandle,
    },
}

/// Specular reflection of unit `dir` about unit `normal`.
pub fn reflect(dir: Vector3, normal: Vector3) -> Vector3 {
    dir - normal * (2.0 * dir.dot(normal))
}
