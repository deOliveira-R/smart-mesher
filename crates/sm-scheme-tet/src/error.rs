//! Typed failures of the tetrahedralizer. Every variant carries indices so an adapter can
//! locate the offending triangle or edge.

use sm_math::PredicateError;

/// Why a surface could not be tetrahedralized. Indices refer to the input arrays
/// ([`crate::SurfaceInput`]).
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TetError {
    /// An edge is used by exactly one triangle (or by triangles all wound the same way).
    #[error("surface is not closed: edge {edge:?} is not shared by two oppositely wound triangles")]
    NotClosed {
        /// The undirected edge (point indices, ascending).
        edge: [u32; 2],
    },
    /// An edge is shared by more than two triangles.
    #[error("surface is non-manifold: edge {edge:?} is shared by more than two triangles")]
    NonManifold {
        /// The undirected edge (point indices, ascending).
        edge: [u32; 2],
    },
    /// Two input points have identical coordinates.
    #[error("points {a} and {b} have identical coordinates")]
    DuplicatePoint {
        /// First point index.
        a: u32,
        /// Second point index.
        b: u32,
    },
    /// The shell orientation contradicts the rule "normals point out of the region".
    #[error("inverted or inconsistent shell near triangle {tri}")]
    InvertedShell {
        /// A triangle at which the contradiction was found.
        tri: u32,
    },
    /// A triangle has repeated indices, non-finite coordinates or collinear corners.
    #[error("triangle {tri} is degenerate")]
    DegenerateTri {
        /// Triangle index.
        tri: u32,
    },
    /// A triangle (or one of its edges) could not be recovered without boundary Steiner points.
    #[error("boundary recovery failed for triangle {tri} (edge {edge:?}): {reason}")]
    RecoveryFailed {
        /// The triangle that could not be recovered.
        tri: u32,
        /// The edge that could not be recovered, if the failure is edge-specific.
        edge: Option<[u32; 2]>,
        /// What went wrong.
        reason: String,
    },
    /// Two input triangles cross each other.
    #[error("triangles {tri_a} and {tri_b} intersect")]
    SelfIntersection {
        /// First triangle.
        tri_a: u32,
        /// Second triangle.
        tri_b: u32,
    },
    /// The interior Steiner point budget was exhausted.
    #[error("Steiner point budget of {limit} exhausted")]
    SteinerBudget {
        /// The configured limit.
        limit: usize,
    },
    /// An option has an invalid value.
    #[error("bad option: {0}")]
    BadOption(String),
    /// A coordinate is outside the exactness range of the predicates.
    #[error("coordinate outside the exact predicate range: {0}")]
    OutOfRange(PredicateError),
    /// An internal invariant broke (a bug or a pathological input).
    #[error("internal error: {0}")]
    Internal(String),
}
