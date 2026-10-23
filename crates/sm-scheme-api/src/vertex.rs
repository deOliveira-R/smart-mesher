//! Surface vertex types (Cubit End / Side / Corner / Reversal) and corner picking.
//!
//! A vertex type belongs to a **vertex use in one face** (a `CornerKey`), not to the vertex: the
//! same vertex is an End on one face and a Side on the next. Mapping-family schemes read the
//! types to find logical sides; the picking algorithm itself is a service ([`CornerPicker`]) that
//! `sm-sizing` implements, so every scheme agrees on one assignment.

use crate::error::SchemeError;
use crate::intent::Firmness;
use crate::key::ShapeKey;
use crate::shape::{LoopUse, ShapeView};
use core::f64::consts::{FRAC_PI_2, PI};
use serde::{Deserialize, Serialize};
use sm_topo_api::CornerKey;

/// Logical type of a face corner: how many elements meet there in a structured mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum VertexType {
    /// One element (about 90 degrees): a logical corner of a mapped side.
    End,
    /// Two elements (about 180 degrees): inside a logical side.
    Side,
    /// Three elements (about 270 degrees).
    Corner,
    /// Four elements (about 360 degrees).
    Reversal,
}

impl VertexType {
    /// Elements meeting at the corner (1..=4).
    pub const fn elements(self) -> u8 {
        match self {
            VertexType::End => 1,
            VertexType::Side => 2,
            VertexType::Corner => 3,
            VertexType::Reversal => 4,
        }
    }

    /// The ideal interior angle in radians.
    pub fn ideal_angle(self) -> f64 {
        f64::from(self.elements()) * FRAC_PI_2
    }

    /// Turn contribution in quarter turns (`End` +1, `Side` 0, `Corner` -1, `Reversal` -2). An
    /// outer loop of a submappable face sums to 4, a hole to -4.
    pub const fn quarter_turns(self) -> i32 {
        2 - self.elements() as i32
    }

    /// The type whose ideal angle is nearest to `angle` (ties go to the type with fewer
    /// elements). Angles at or below 0 are `End`, above 2 pi `Reversal`.
    pub fn from_angle(angle: f64) -> VertexType {
        let q = angle / FRAC_PI_2;
        if q < 1.5 {
            VertexType::End
        } else if q < 2.5 {
            VertexType::Side
        } else if q < 3.5 {
            VertexType::Corner
        } else {
            VertexType::Reversal
        }
    }
}

/// A vertex type set on one face corner, with its firmness.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CornerAssignment {
    /// The corner (vertex use in a face).
    pub corner: CornerKey,
    /// The type.
    pub vertex_type: VertexType,
    /// User settings are `Hard`; picker and scheme assignments `Soft`.
    pub firmness: Firmness,
}

/// Assigns vertex types to the corners of a face. Implemented by `sm-sizing`; the driver passes
/// one picker to every scheme so all schemes see the same assignment.
pub trait CornerPicker: Send + Sync {
    /// Vertex types per loop and per use (the corner at the *start* of each use), honoring
    /// `fixed` assignments (`Hard` always, `Soft` unless the picker knows better).
    fn pick(&self, shape: &dyn ShapeView, face: ShapeKey, loops: &[LoopUse], fixed: &[CornerAssignment]) -> Result<Vec<Vec<VertexType>>, SchemeError>;
}

/// Reference picker: nearest ideal angle per corner, overridden by fixed assignments. It does
/// not balance the loop sum (the real picker in `sm-sizing` does).
#[derive(Clone, Copy, Debug, Default)]
pub struct AnglePicker;

impl CornerPicker for AnglePicker {
    fn pick(&self, shape: &dyn ShapeView, face: ShapeKey, loops: &[LoopUse], fixed: &[CornerAssignment]) -> Result<Vec<Vec<VertexType>>, SchemeError> {
        let mut out = Vec::with_capacity(loops.len());
        for l in loops {
            let mut types = Vec::with_capacity(l.uses.len());
            for (i, u) in l.uses.iter().enumerate() {
                let set = u.corner.and_then(|c| fixed.iter().find(|a| a.corner == c));
                types.push(match set {
                    Some(a) => a.vertex_type,
                    None => VertexType::from_angle(shape.corner_angle(face, l.ring, i)?),
                });
            }
            out.push(types);
        }
        Ok(out)
    }
}

/// Interior angle from the incoming and outgoing tangents and the face normal (radians in
/// `(0, 2 pi)`): `pi - turn`, with `turn` the signed angle from `t_in` to `t_out` about `n`.
pub fn interior_angle(t_in: sm_core::Vector3, t_out: sm_core::Vector3, n: sm_core::Vector3) -> f64 {
    let turn = n.dot(t_in.cross(t_out)).atan2(t_in.dot(t_out));
    PI - turn
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::Vector3;

    #[test]
    fn angles_classify_to_nearest_type() {
        assert_eq!(VertexType::from_angle(FRAC_PI_2), VertexType::End);
        assert_eq!(VertexType::from_angle(PI), VertexType::Side);
        assert_eq!(VertexType::from_angle(1.5 * PI), VertexType::Corner);
        assert_eq!(VertexType::from_angle(2.0 * PI), VertexType::Reversal);
        let turns: i32 = [VertexType::End; 4].iter().map(|t| t.quarter_turns()).sum();
        assert_eq!(turns, 4, "a rectangle's corners sum to one full turn");
        assert_eq!(VertexType::Corner.quarter_turns() + VertexType::Reversal.quarter_turns(), -3);
    }

    #[test]
    fn interior_angle_of_convex_and_reflex_corners() {
        let z = Vector3::new(0.0, 0.0, 1.0);
        let (x, y) = (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
        // Counter-clockwise about +z: a left turn of 90 degrees leaves a 90 degree interior.
        assert!((interior_angle(x, y, z) - FRAC_PI_2).abs() < 1e-12);
        // A right turn: reflex 270 degrees.
        assert!((interior_angle(x, -y, z) - 1.5 * PI).abs() < 1e-12);
        assert!((interior_angle(x, x, z) - PI).abs() < 1e-12);
    }
}
