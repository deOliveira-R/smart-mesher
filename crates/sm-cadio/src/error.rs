//! [`StepError`]: every way reading, decoding or realizing a STEP file can fail.

use sm_op_api::OpError;
use sm_topo_api::TopoError;

/// One edge of an open or inconsistently oriented shell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OpenEdge {
    /// STEP id of the `EDGE_CURVE`.
    pub edge: u64,
    /// Net number of traversals (forward minus reverse) over the shell's faces; zero for a
    /// closed, consistently oriented shell.
    pub net_uses: i64,
}

/// A STEP reading error. Syntax-level errors carry a 1-based `line:col` into the input
/// (columns count bytes); decode-level errors carry the STEP entity id.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum StepError {
    /// The text is not valid Part 21.
    #[error("syntax error at {line}:{col}: {msg}")]
    Syntax {
        /// 1-based line.
        line: usize,
        /// 1-based byte column.
        col: usize,
        /// What is wrong.
        msg: String,
    },
    /// The input ends before the structure is complete.
    #[error("unexpected end of input at {line}:{col}: {msg}")]
    Truncated {
        /// Line of the end of input.
        line: usize,
        /// Column of the end of input.
        col: usize,
        /// What was still expected.
        msg: String,
    },
    /// Parameter lists are nested deeper than the supported 256 levels.
    #[error("parameters nested deeper than {max} levels at {line}:{col}")]
    TooDeep {
        /// Line of the offending parenthesis.
        line: usize,
        /// Column of the offending parenthesis.
        col: usize,
        /// The nesting limit.
        max: usize,
    },
    /// The input exceeds the 2 GiB limit.
    #[error("input of {bytes} bytes exceeds the {max} byte limit")]
    TooLarge {
        /// Input size.
        bytes: u64,
        /// The limit.
        max: u64,
    },
    /// An entity references an id that does not exist.
    #[error("#{referenced_by} references #{id}, which does not exist")]
    MissingEntity {
        /// The missing id.
        id: u64,
        /// The entity holding the reference.
        referenced_by: u64,
    },
    /// An entity has the wrong type for the role it plays.
    #[error("#{id} is {found}, expected {expected}")]
    WrongType {
        /// The entity.
        id: u64,
        /// What the referencing entity needs.
        expected: String,
        /// What it is.
        found: String,
    },
    /// A parameter is missing, has the wrong type or an invalid value.
    #[error("#{id} {entity} parameter {index}: {msg}")]
    BadParam {
        /// The entity.
        id: u64,
        /// Its (upper-cased) type name.
        entity: String,
        /// 0-based parameter index.
        index: usize,
        /// What is wrong.
        msg: String,
    },
    /// Entities reference each other in a cycle.
    #[error("#{id} is part of a reference cycle")]
    Cycle {
        /// An entity on the cycle.
        id: u64,
    },
    /// A geometric entity this reader does not support.
    #[error("#{id} {entity} is not supported")]
    Unsupported {
        /// The entity.
        id: u64,
        /// Its type name.
        entity: String,
    },
    /// Units are missing or inconsistent.
    #[error("{0}")]
    Units(String),
    /// A vertex is too far from the curve of an edge (or a closed edge cannot be parametrized).
    #[error("#{id}: {msg}")]
    GeometryMismatch {
        /// The edge (or face) id.
        id: u64,
        /// What is wrong.
        msg: String,
    },
    /// A shell is open or its faces are not consistently oriented.
    #[error("solid #{solid} has an open shell: {}", fmt_open(issues))]
    OpenShell {
        /// The solid.
        solid: u64,
        /// The edges that do not cancel, ascending by id.
        issues: Vec<OpenEdge>,
    },
    /// A topology edit failed while realizing the model.
    #[error("topology: {0}")]
    Topology(TopoError),
    /// The `inline` source is malformed or too large.
    #[error("{0}")]
    Inline(String),
    /// The writer cannot export the model (or an I/O error while writing).
    #[error("export: {0}")]
    Export(String),
}

fn fmt_open(issues: &[OpenEdge]) -> String {
    let mut s = String::new();
    for (i, e) in issues.iter().take(5).enumerate() {
        if i > 0 {
            s.push_str(", ");
        }
        s.push_str(&format!("edge #{} (net {} uses)", e.edge, e.net_uses));
    }
    if issues.len() > 5 {
        s.push_str(&format!(", and {} more", issues.len() - 5));
    }
    s
}

impl From<TopoError> for StepError {
    fn from(e: TopoError) -> Self {
        Self::Topology(e)
    }
}

impl StepError {
    /// The stable machine code (`step.<code>`) of the variant.
    pub fn code(&self) -> &'static str {
        match self {
            Self::Syntax { .. } => "step.syntax",
            Self::Truncated { .. } => "step.truncated",
            Self::TooDeep { .. } => "step.too_deep",
            Self::TooLarge { .. } => "step.too_large",
            Self::MissingEntity { .. } => "step.missing_entity",
            Self::WrongType { .. } => "step.wrong_type",
            Self::BadParam { .. } => "step.bad_param",
            Self::Cycle { .. } => "step.cycle",
            Self::Unsupported { .. } => "step.unsupported",
            Self::Units(_) => "step.units",
            Self::GeometryMismatch { .. } => "step.geometry_mismatch",
            Self::OpenShell { .. } => "step.open_shell",
            Self::Topology(_) => "step.topology",
            Self::Inline(_) => "step.inline",
            Self::Export(_) => "step.export",
        }
    }
}

impl From<StepError> for OpError {
    fn from(e: StepError) -> Self {
        OpError::Invalid(format!("{}: {e}", e.code()))
    }
}
