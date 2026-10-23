//! Parameter-space curves: how an edge runs inside a face's `(u, v)` domain.
//!
//! Needed where the 3D curve alone is ambiguous: seams (the same edge appears at `u = lo` and
//! `u = hi`), poles (an edge collapses to a point) and trimmed periodic faces.

/// A curve in the `(u, v)` domain of a surface, parametrized by `s` in `[0, 1]`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum PCurve {
    /// Straight segment in parameter space (iso-curves, seams, pole collapses).
    Segment {
        /// Start `(u, v)`.
        start: [f64; 2],
        /// End `(u, v)`.
        end: [f64; 2],
    },
    /// Polyline in parameter space (sampled intersection curves).
    Polyline(Vec<[f64; 2]>),
}

impl PCurve {
    /// Point at `s` in `[0, 1]`; an empty polyline evaluates to the origin.
    pub fn eval(&self, s: f64) -> [f64; 2] {
        match self {
            Self::Segment { start, end } => [start[0] + s * (end[0] - start[0]), start[1] + s * (end[1] - start[1])],
            Self::Polyline(p) => {
                if p.len() < 2 {
                    return p.first().copied().unwrap_or([0.0, 0.0]);
                }
                let x = s.clamp(0.0, 1.0) * (p.len() - 1) as f64;
                let i = (x.floor() as usize).min(p.len() - 2);
                let f = x - i as f64;
                [p[i][0] + f * (p[i + 1][0] - p[i][0]), p[i][1] + f * (p[i + 1][1] - p[i][1])]
            }
        }
    }

    /// The same curve traversed backwards.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Segment { start, end } => Self::Segment { start: *end, end: *start },
            Self::Polyline(p) => Self::Polyline(p.iter().rev().copied().collect()),
        }
    }
}
