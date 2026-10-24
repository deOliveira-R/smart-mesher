//! The syntax tree. Every node carries a [`Span`]; nodes the parser could not build are `Error` variants,
//! so a tree always exists and later phases skip the broken parts.

use sm_core::EntityKind;

use crate::span::Span;

/// An identifier with its span.
#[derive(Clone, Debug, PartialEq)]
pub struct Ident {
    /// The name.
    pub name: String,
    /// Where it is.
    pub span: Span,
}

/// A dotted name such as `model.create_brick`.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    /// Segments, at least one.
    pub segs: Vec<Ident>,
    /// Span of the whole path.
    pub span: Span,
}

impl Path {
    /// The segments joined with `.`.
    pub fn dotted(&self) -> String {
        let mut s = String::new();
        for (i, seg) in self.segs.iter().enumerate() {
            if i > 0 {
                s.push('.');
            }
            s.push_str(&seg.name);
        }
        s
    }
}

/// A parsed source file.
#[derive(Clone, Debug, PartialEq)]
pub struct Script {
    /// File index in the [`SourceMap`](crate::SourceMap).
    pub file: u32,
    /// Top-level statements.
    pub stmts: Vec<Stmt>,
}

/// A `{ ... }` statement block.
#[derive(Clone, Debug, PartialEq)]
pub struct Block {
    /// Span from `{` to `}`.
    pub span: Span,
    /// The statements.
    pub stmts: Vec<Stmt>,
}

/// A statement.
#[derive(Clone, Debug, PartialEq)]
pub struct Stmt {
    /// Whole-statement span.
    pub span: Span,
    /// What it is.
    pub kind: StmtKind,
}

/// `else` part of an `if`.
#[derive(Clone, Debug, PartialEq)]
pub enum Else {
    /// `else { ... }`
    Block(Block),
    /// `else if ...` (the boxed statement is an `If`).
    If(Box<Stmt>),
}

/// Statement kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum StmtKind {
    /// `let name = value`
    Let {
        /// Bound name.
        name: Ident,
        /// Right-hand side.
        value: Expr,
    },
    /// `set name = value`
    Set {
        /// Rebound name.
        name: Ident,
        /// Right-hand side.
        value: Expr,
    },
    /// `alias name = dotted.target`
    Alias {
        /// The short name.
        name: Ident,
        /// The registry name it stands for.
        target: Path,
    },
    /// An op or query invocation.
    Call(Call),
    /// `batch { ... }`
    Batch(Block),
    /// `if cond { } else { }`
    If {
        /// Condition.
        cond: Expr,
        /// Then block.
        then: Block,
        /// Optional else.
        els: Option<Else>,
    },
    /// `for var in iter { }`
    For {
        /// Loop variable.
        var: Ident,
        /// Iterated list or entity set.
        iter: Expr,
        /// Body.
        body: Block,
    },
    /// `include "path"`
    Include {
        /// The path text.
        path: String,
        /// Span of the string literal.
        path_span: Span,
    },
    /// `print e, ...`
    Print(Vec<Expr>),
    /// `assert cond, "message"`
    Assert {
        /// Condition.
        cond: Expr,
        /// Optional message.
        msg: Option<Expr>,
    },
    /// `help` or `help name`
    Help(Option<Path>),
    /// A statement that failed to parse.
    Error,
}

/// A call `path(args)`: an op or query at statement level, a builtin function in expressions.
#[derive(Clone, Debug, PartialEq)]
pub struct Call {
    /// The callee.
    pub path: Path,
    /// Arguments in source order.
    pub args: Vec<Arg>,
    /// Span of the whole call.
    pub span: Span,
    /// Span of the parenthesized argument list.
    pub args_span: Span,
}

/// One call argument.
#[derive(Clone, Debug, PartialEq)]
pub enum Arg {
    /// `name: value`
    Named {
        /// Parameter name.
        name: Ident,
        /// Value expression.
        value: Expr,
    },
    /// `...expr`
    Spread(Expr),
    /// `expr`
    Pos(Expr),
}

/// An object-literal key.
#[derive(Clone, Debug, PartialEq)]
pub struct Key {
    /// Key text.
    pub name: String,
    /// Span of the key token.
    pub span: Span,
}

/// An index inside `[...]` after an expression.
#[derive(Clone, Debug, PartialEq)]
pub enum Index {
    /// `[expr]`
    At(Box<Expr>),
    /// `[*]`
    All,
}

/// Unary operators.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnOp {
    /// `-`
    Neg,
    /// `not`
    Not,
}

/// Binary operators.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BinOp {
    /// `+`
    Add,
    /// `-`
    Sub,
    /// `except` (set difference)
    Except,
    /// `*`
    Mul,
    /// `/`
    Div,
    /// `%`
    Rem,
    /// `**`
    Pow,
    /// `==`
    Eq,
    /// `!=`
    Ne,
    /// `<`
    Lt,
    /// `<=`
    Le,
    /// `>`
    Gt,
    /// `>=`
    Ge,
    /// `and`
    And,
    /// `or`
    Or,
    /// `|` set union
    Union,
    /// `&` set intersection
    Inter,
}

/// One element of an id specification.
#[derive(Clone, Debug, PartialEq)]
pub enum IdItem {
    /// A single serial.
    One(u64, Span),
    /// `lo..hi [by step]`, inclusive.
    Range {
        /// First serial.
        lo: u64,
        /// Last serial (inclusive).
        hi: u64,
        /// Step (at least 1).
        step: u64,
        /// Span of the range text.
        span: Span,
    },
}

/// Entity-list primary forms (`LANGUAGE.md` §5.2).
#[derive(Clone, Debug, PartialEq)]
pub enum EntityExpr {
    /// `Kind#3`, `Kind#1..5`, `Kind#[1, 3, 5..7]`
    Ids {
        /// The kind.
        kind: EntityKind,
        /// Span of `Kind#`.
        kind_span: Span,
        /// The items. Single ids are strict (must be live); ranges select live members only.
        items: Vec<IdItem>,
    },
    /// `all faces`
    All {
        /// The kind.
        kind: EntityKind,
        /// Span of the plural keyword.
        kind_span: Span,
    },
    /// `faces in <operand>`
    In {
        /// The target kind.
        kind: EntityKind,
        /// Span of the plural keyword.
        kind_span: Span,
        /// The operand (postfix-level expression).
        within: Box<Expr>,
    },
}

/// An expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    /// Span.
    pub span: Span,
    /// What it is.
    pub kind: ExprKind,
    /// Height of the tree below (1 for a leaf). Bounded by the parser so later phases can recurse safely.
    pub height: u32,
}

impl Expr {
    /// Build a node, computing its height from its children.
    pub fn new(span: Span, kind: ExprKind) -> Self {
        let height = 1 + kind.child_height();
        Self { span, kind, height }
    }
}

impl ExprKind {
    fn child_height(&self) -> u32 {
        fn call(c: &Call) -> u32 {
            c.args
                .iter()
                .map(|a| match a {
                    Arg::Named { value, .. } => value.height,
                    Arg::Spread(e) | Arg::Pos(e) => e.height,
                })
                .max()
                .unwrap_or(0)
        }
        match self {
            ExprKind::List(v) => v.iter().map(|e| e.height).max().unwrap_or(0),
            ExprKind::Obj(v) => v.iter().map(|(_, e)| e.height).max().unwrap_or(0),
            ExprKind::Field(b, _) => b.height,
            ExprKind::Index(b, Index::All) => b.height,
            ExprKind::Index(b, Index::At(i)) => b.height.max(i.height),
            ExprKind::Call(c) => call(c),
            ExprKind::Unary(_, a) => a.height,
            ExprKind::Binary(_, a, b) => a.height.max(b.height),
            ExprKind::Entity(EntityExpr::In { within, .. }) => within.height,
            _ => 0,
        }
    }
}

/// Expression kinds.
#[derive(Clone, Debug, PartialEq)]
pub enum ExprKind {
    /// Integer literal.
    Int(i64),
    /// Float literal.
    Num(f64),
    /// String literal.
    Str(String),
    /// `true` / `false`
    Bool(bool),
    /// `[a, b]`
    List(Vec<Expr>),
    /// `{ k: v }`
    Obj(Vec<(Key, Expr)>),
    /// A variable.
    Var(String),
    /// `expr.field`
    Field(Box<Expr>, Ident),
    /// `expr[index]`
    Index(Box<Expr>, Index),
    /// `path(args)`: builtin function (single segment) or op/query call (dotted).
    Call(Call),
    /// Unary operation.
    Unary(UnOp, Box<Expr>),
    /// Binary operation.
    Binary(BinOp, Box<Expr>, Box<Expr>),
    /// An entity-list primary.
    Entity(EntityExpr),
    /// An expression that failed to parse.
    Error,
}
