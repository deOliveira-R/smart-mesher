//! Tessellation tolerances and the error type.

use sm_core::EntityId;

/// Tolerances that drive sampling.
///
/// All three are upper bounds on the produced mesh:
/// - `chord_tol`: largest distance (model length units) between a curve or surface and its
///   polyline or triangles, where the sampling rule resolves it. On grid-meshed faces (cylinder,
///   cone, sphere, torus, revolved or ruled patches) this holds for the whole triangle, interior
///   included: on doubly curved surfaces the budget is split in half between the two directions.
///   Faces that are not parameter rectangles are resolved at their boundary only (flagged by
///   `FaceRange::boundary_only`), and freeform grid faces get their `v` resolution from the
///   side edges (see the crate docs);
/// - `angle_tol`: largest turning angle (radians) between neighbouring segments of a curved edge
///   and between neighbouring points of every grid row of a grid-meshed face (the angle between
///   grid lines across rows is not controlled);
/// - `max_edge`: longest segment of an edge polyline or grid line (`f64::INFINITY` disables it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TessOptions {
    /// Chord (sagitta) tolerance, model length units. Must be positive and finite.
    pub chord_tol: f64,
    /// Angle tolerance in radians, in `(0, pi]`.
    pub angle_tol: f64,
    /// Longest allowed segment, model length units; `f64::INFINITY` for none.
    pub max_edge: f64,
}

impl Default for TessOptions {
    /// `chord_tol = 0.01`, `angle_tol = 0.3` rad (about 17 degrees), no maximum edge length.
    fn default() -> Self {
        Self { chord_tol: 1e-2, angle_tol: 0.3, max_edge: f64::INFINITY }
    }
}

impl TessOptions {
    /// Options with the given chord tolerance and the default angle tolerance.
    pub fn with_chord(chord_tol: f64) -> Self {
        Self { chord_tol, ..Self::default() }
    }

    /// Check the ranges listed on the fields.
    pub fn validate(&self) -> Result<(), TessError> {
        if !(self.chord_tol.is_finite() && self.chord_tol > 0.0) {
            return Err(TessError::BadOptions(format!("chord_tol must be positive and finite, got {}", self.chord_tol)));
        }
        if !(self.angle_tol.is_finite() && self.angle_tol > 0.0 && self.angle_tol <= core::f64::consts::PI) {
            return Err(TessError::BadOptions(format!("angle_tol must be in (0, pi], got {}", self.angle_tol)));
        }
        if self.max_edge.is_nan() || self.max_edge <= 0.0 {
            return Err(TessError::BadOptions(format!("max_edge must be positive (or infinite), got {}", self.max_edge)));
        }
        Ok(())
    }
}

/// Why a tessellation failed. Failures are all-or-nothing: no partial result is returned.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TessError {
    /// The options are out of range.
    #[error("bad tessellation options: {0}")]
    BadOptions(String),
    /// A requested id is not a live cell of the expected rank.
    #[error("{0:?} is not a live face")]
    NotAFace(EntityId),
    /// A requested id is not a live cell of the expected rank.
    #[error("{0:?} is not a live edge")]
    NotAnEdge(EntityId),
    /// A requested id cannot be tessellated (only faces, volumes, bodies and edges can).
    #[error("{0:?} cannot be tessellated (expected a face, volume, body or edge)")]
    BadEntity(EntityId),
    /// An edge could not be sampled (missing curve, missing end vertices, ...).
    #[error("edge {edge:?}: {reason}")]
    Edge {
        /// The edge.
        edge: EntityId,
        /// What went wrong.
        reason: String,
    },
    /// A vertex has no position.
    #[error("vertex {vertex:?}: {reason}")]
    Vertex {
        /// The vertex.
        vertex: EntityId,
        /// What went wrong.
        reason: String,
    },
    /// A face could not be triangulated.
    #[error("face {face:?}: {reason}")]
    Face {
        /// The face.
        face: EntityId,
        /// What went wrong.
        reason: String,
    },
    /// The result would not fit `u32` indices or the sampling cap.
    #[error("tessellation too large: {0}")]
    TooLarge(String),
}

impl TessError {
    pub(crate) fn edge(edge: EntityId, reason: impl Into<String>) -> Self {
        Self::Edge { edge, reason: reason.into() }
    }

    pub(crate) fn face(face: EntityId, reason: impl Into<String>) -> Self {
        Self::Face { face, reason: reason.into() }
    }
}
