//! The smart-mesh command language: parser, entity lists, scripts, translation to Ops.
//!
//! See `LANGUAGE.md` for the language and `DESIGN.md` for the implementation.

pub mod ast;
pub mod diag;
pub mod kinds;
pub mod lexer;
pub mod parser;
pub mod span;

pub use diag::{Diagnostic, Severity};
pub use parser::{ParseLimits, ParseOutput, parse, parse_with};
pub use span::{Source, SourceMap, Span};
