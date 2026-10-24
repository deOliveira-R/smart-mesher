//! The smart-mesh command language: parser, entity lists, scripts, translation to Ops.
//!
//! See `LANGUAGE.md` for the language and `DESIGN.md` for the implementation.

pub mod ast;
pub mod diag;
pub mod entity;
pub mod eval;
pub mod help;
pub mod host;
pub mod interp;
pub mod kinds;
pub mod lexer;
pub mod parser;
pub mod schema;
pub mod span;
pub mod value;

pub use diag::{Diagnostic, Severity};
pub use entity::{EntitySet, Relations};
pub use host::{EngineHost, Host, HostError, PlanHost, QueryRelations};
pub use interp::{Event, Interpreter, Options, RunReport};
pub use value::Val;
pub use parser::{ParseLimits, ParseOutput, parse, parse_with};
pub use span::{Source, SourceMap, Span};
