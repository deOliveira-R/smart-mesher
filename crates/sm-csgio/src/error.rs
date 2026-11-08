//! Errors and findings of CSG import.

use sm_rt_api::{BuildIssue, RtError};

/// Where a flat (placed) cell comes from: the deck cell and the filled cells and lattice
/// elements it was reached through (outermost first).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CellSource {
    /// The deck cell id of the cell that carries the material.
    pub cell: u32,
    /// The filled cells (and their lattice element, when the fill is a lattice) on the way down.
    pub path: Vec<PathStep>,
}

/// One level of a [`CellSource`] path.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PathStep {
    /// The deck cell whose `fill` was entered.
    pub cell: u32,
    /// The lattice element `[i, j, k]` entered, when the fill is a lattice.
    pub element: Option<[i32; 3]>,
}

impl core::fmt::Display for CellSource {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for s in &self.path {
            match s.element {
                Some([i, j, k]) => write!(f, "{}[{i},{j},{k}]/", s.cell)?,
                None => write!(f, "{}/", s.cell)?,
            }
        }
        write!(f, "{}", self.cell)
    }
}

/// Two cells found to contain the same point.
#[derive(Clone, Debug, PartialEq)]
pub struct OverlapFinding {
    /// A point inside both cells.
    pub point: [f64; 3],
    /// The first cell.
    pub a: CellSource,
    /// The second cell.
    pub b: CellSource,
}

fn first_overlap(f: &[OverlapFinding]) -> String {
    f.first().map(|f| format!(", first: cells {} and {} at ({}, {}, {})", f.a, f.b, f.point[0], f.point[1], f.point[2])).unwrap_or_default()
}

/// Errors of CSG import.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ImportError {
    /// The input text is malformed (1-based line and column).
    #[error("parse error at {line}:{col}: {msg}")]
    Parse {
        /// Line.
        line: u32,
        /// Column.
        col: u32,
        /// What is wrong.
        msg: String,
    },
    /// A format or feature the importer does not implement; always names the element and, when
    /// the format has lines, the line.
    #[error("unsupported{}: {what}", .line.map(|l| format!(" (line {l})")).unwrap_or_default())]
    Unsupported {
        /// Line of the offending element in the deck, when known.
        line: Option<u32>,
        /// What is refused.
        what: String,
    },
    /// A cell region names a surface the deck does not define.
    #[error("cell {cell} uses undefined surface {surface}")]
    UnknownSurface {
        /// The cell.
        cell: u32,
        /// The surface id.
        surface: u32,
    },
    /// A cell names a material the deck does not define.
    #[error("cell {cell} uses undefined material {material}")]
    UnknownMaterial {
        /// The cell (0 for a name override).
        cell: u32,
        /// The material id.
        material: u32,
    },
    /// A cell or lattice fills with, or the deck roots at, a universe that has no cell.
    #[error("{}universe {universe} has no cells", .cell.map(|c| format!("cell {c} fills with undefined ")).unwrap_or_default())]
    UnknownUniverse {
        /// The filling cell; `None` for the root universe.
        cell: Option<u32>,
        /// The universe id.
        universe: u32,
    },
    /// A cell fills with a lattice the deck does not define.
    #[error("cell {cell} fills with undefined lattice {lattice}")]
    UnknownLattice {
        /// The cell.
        cell: u32,
        /// The lattice id.
        lattice: u32,
    },
    /// A universe contains itself, directly or through other universes or lattices.
    #[error("universe {universe} contains itself")]
    UniverseCycle {
        /// A universe on the cycle.
        universe: u32,
    },
    /// A region expands to more than [`crate::MAX_TERMS`] terms in disjunctive normal form.
    #[error("cell {cell} expands to {terms} terms, over the limit {}", crate::MAX_TERMS)]
    TooManyTerms {
        /// The deck cell.
        cell: u32,
        /// How many terms the expansion reached when it stopped.
        terms: usize,
    },
    /// The flat model would have more cells than `ImportOptions::max_cells`.
    #[error("the flat model has more than {limit} cells")]
    TooManyCells {
        /// The limit.
        limit: usize,
    },
    /// Cells that overlap were found by sampling (at most 16 findings).
    #[error("{} overlap(s) found{}", .findings.len(), first_overlap(.findings))]
    Overlap {
        /// The findings, in the order the check reports them (never empty).
        findings: Vec<OverlapFinding>,
    },
    /// The model or an option is inconsistent.
    #[error("{0}")]
    Invalid(String),
    /// The ray-tracing layer refused the model.
    #[error("build failed: {}", .issues.iter().map(|i| format!("{}: {}", i.code, i.message)).collect::<Vec<_>>().join("; "))]
    Build {
        /// The issues, as the tracker builder or the check reported them.
        issues: Vec<BuildIssue>,
    },
}

impl From<RtError> for ImportError {
    fn from(e: RtError) -> Self {
        match e {
            RtError::Build(issues) => Self::Build { issues },
            other => Self::Build { issues: vec![BuildIssue::new("rt.error", other.to_string())] },
        }
    }
}
