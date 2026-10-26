//! Surface vertex types (Cubit End/Side/Corner/Reversal) and the corner rules schemes require.

use crate::firm::Firmness;
use core::f64::consts::FRAC_PI_2;
use serde::{Deserialize, Serialize};
use sm_topo_api::{CoedgeKey, CornerKey};

/// Type of a face corner for structured schemes. Stored **per face use of a vertex**
/// ([`CornerKey`]), never on the vertex: the same vertex can be an End on one face and a
/// Side on another.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum VertexType {
    /// One element at the corner, interior angle about 90 degrees.
    End,
    /// Two elements, about 180 degrees (the vertex lies inside a logical side).
    Side,
    /// Three elements, about 270 degrees.
    Corner,
    /// Four elements, about 360 degrees.
    Reversal,
}

impl VertexType {
    /// All types in angle order.
    pub const ALL: [VertexType; 4] = [
        VertexType::End,
        VertexType::Side,
        VertexType::Corner,
        VertexType::Reversal,
    ];

    /// Number of quadrilaterals meeting at the corner inside the face.
    pub const fn element_count(self) -> u32 {
        match self {
            VertexType::End => 1,
            VertexType::Side => 2,
            VertexType::Corner => 3,
            VertexType::Reversal => 4,
        }
    }

    /// Nominal interior angle in radians (`element_count * pi/2`).
    pub fn nominal_angle(self) -> f64 {
        f64::from(self.element_count()) * FRAC_PI_2
    }

    /// Contribution to the submap corner sum, which is also the quarter turn the boundary
    /// makes at the corner: End `+1`, Side `0`, Corner `-1`, Reversal `-2`. A single-loop face
    /// is submappable iff the sum over its corners is 4.
    pub const fn submap_weight(self) -> i32 {
        2 - self.element_count() as i32
    }

    /// The type whose nominal angle is nearest to `angle` (radians); ties go to the smaller type.
    pub fn nearest(angle: f64) -> Self {
        let mut best = VertexType::End;
        let mut err = f64::INFINITY;
        for t in Self::ALL {
            let e = (angle - t.nominal_angle()).abs();
            if e < err {
                best = t;
                err = e;
            }
        }
        best
    }
}

/// What a surface scheme needs from the corners of its face. Contributed in the requirements
/// phase and solved by the [`crate::CornerPicker`] before intervals are assembled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum VertexRule {
    /// No corner structure needed (pave, tri*). Types are still classified for reporting.
    Free,
    /// Exactly four `End` corners, every other corner `Side` (map; one loop).
    Map,
    /// Corner sum `#End - #Corner - 2 #Reversal = 4` (submap; one loop).
    Submap,
    /// Exactly `n` `End` corners, the rest `Side` (triprimitive: 3, polygon schemes: n).
    Ends(u8),
}

/// The solved type of one corner.
///
/// `#[non_exhaustive]` (W0g r3): build it with [`CornerAssignment::new`]. It is serialized in
/// `Plan.layouts`, so any field added later is `#[serde(default)]`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct CornerAssignment {
    /// The corner (face, incoming coedge).
    pub corner: CornerKey,
    /// Its type.
    pub vertex_type: VertexType,
    /// Firmness of the type (`Hard` = user-fixed, `Default` = picked).
    pub firmness: Firmness,
    /// Measured interior angle (radians), used by the picker.
    pub angle: f64,
}

impl CornerAssignment {
    /// A solved corner type.
    pub const fn new(
        corner: CornerKey,
        vertex_type: VertexType,
        firmness: Firmness,
        angle: f64,
    ) -> Self {
        Self {
            corner,
            vertex_type,
            firmness,
            angle,
        }
    }
}

/// Direction of a logical side in the face's (i, j) frame, from the cumulative quarter turns
/// of the boundary: the first side runs `PlusI`, each End turns left (`PlusJ`, `MinusI`,
/// `MinusJ`), each Corner turns right, a Reversal turns back twice.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SideDir {
    /// Along +i.
    PlusI,
    /// Along +j.
    PlusJ,
    /// Along -i.
    MinusI,
    /// Along -j.
    MinusJ,
}

impl SideDir {
    /// The direction after `quarter_turns` left turns (negative: right turns).
    pub const fn turned(self, quarter_turns: i32) -> SideDir {
        let base = match self {
            SideDir::PlusI => 0,
            SideDir::PlusJ => 1,
            SideDir::MinusI => 2,
            SideDir::MinusJ => 3,
        };
        match (base + quarter_turns).rem_euclid(4) {
            0 => SideDir::PlusI,
            1 => SideDir::PlusJ,
            2 => SideDir::MinusI,
            _ => SideDir::MinusJ,
        }
    }

    /// The opposite direction.
    pub const fn reversed(self) -> SideDir {
        self.turned(2)
    }

    /// True for `PlusI` and `MinusI`.
    pub const fn along_i(self) -> bool {
        matches!(self, SideDir::PlusI | SideDir::MinusI)
    }
}

/// One logical side of a structured face: consecutive coedges between two logical corners
/// (curves may be grouped, and a side may end mid-curve only after a virtual split).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct LogicalSide {
    /// Coedges in loop order.
    pub coedges: Vec<CoedgeKey>,
    /// Direction in the face's (i, j) frame (see [`SideDir`]). Submap, triprimitive and sweep
    /// crates all read the same driver-owned value.
    pub dir: SideDir,
}

impl LogicalSide {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(coedges: Vec<CoedgeKey>, dir: SideDir) -> Self {
        Self { coedges, dir }
    }
}

/// Solved corner layout of one face: corner types in loop order and, for structured rules,
/// the logical sides starting at the first logical corner.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct FaceLayout {
    /// The face.
    pub face: sm_core::EntityId,
    /// The rule this layout satisfies.
    pub rule: VertexRule,
    /// Corner types in outer-loop order (inner loops follow, ring by ring).
    pub corners: Vec<CornerAssignment>,
    /// Logical sides (empty for [`VertexRule::Free`]).
    pub sides: Vec<LogicalSide>,
}

impl FaceLayout {
    /// Construct (the type is `#[non_exhaustive]`; a [`crate::CornerPicker`] returns this).
    pub fn new(
        face: sm_core::EntityId,
        rule: VertexRule,
        corners: Vec<CornerAssignment>,
        sides: Vec<LogicalSide>,
    ) -> Self {
        Self {
            face,
            rule,
            corners,
            sides,
        }
    }
}
