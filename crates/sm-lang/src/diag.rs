//! Diagnostics: precise spans, stable codes, rustc-style rendering.

use crate::span::{SourceMap, Span};

/// How serious a [`Diagnostic`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Stops the script.
    Error,
    /// Reported, execution continues.
    Warning,
}

/// One problem found in a script, with the span it belongs to.
#[derive(Clone, Debug, PartialEq)]
pub struct Diagnostic {
    /// Error or warning.
    pub severity: Severity,
    /// Stable kebab-case code (see `LANGUAGE.md`).
    pub code: &'static str,
    /// One-line message.
    pub message: String,
    /// Primary span.
    pub span: Span,
    /// Short text printed under the primary span.
    pub label: Option<String>,
    /// Secondary spans with a short text each.
    pub notes: Vec<(Span, String)>,
    /// A suggestion.
    pub help: Option<String>,
}

impl Diagnostic {
    /// A new error.
    pub fn error(code: &'static str, span: Span, message: impl Into<String>) -> Self {
        Self { severity: Severity::Error, code, message: message.into(), span, label: None, notes: Vec::new(), help: None }
    }

    /// A new warning.
    pub fn warning(code: &'static str, span: Span, message: impl Into<String>) -> Self {
        Self { severity: Severity::Warning, ..Self::error(code, span, message) }
    }

    /// Set the primary label.
    #[must_use]
    pub fn label(mut self, text: impl Into<String>) -> Self {
        self.label = Some(text.into());
        self
    }

    /// Add a secondary span.
    #[must_use]
    pub fn note(mut self, span: Span, text: impl Into<String>) -> Self {
        self.notes.push((span, text.into()));
        self
    }

    /// Set the help line.
    #[must_use]
    pub fn help(mut self, text: impl Into<String>) -> Self {
        self.help = Some(text.into());
        self
    }

    /// True for [`Severity::Error`].
    pub fn is_error(&self) -> bool {
        self.severity == Severity::Error
    }

    /// Render in a stable, rustc-like text form (used by the golden tests).
    pub fn render(&self, map: &SourceMap) -> String {
        let mut out = String::new();
        let sev = match self.severity {
            Severity::Error => "error",
            Severity::Warning => "warning",
        };
        out.push_str(&format!("{sev}[{}]: {}\n", self.code, self.message));
        render_span(&mut out, map, self.span, self.label.as_deref(), true);
        for (sp, text) in &self.notes {
            render_span(&mut out, map, *sp, Some(text), false);
        }
        if let Some(h) = &self.help {
            out.push_str(&format!("  = help: {h}\n"));
        }
        out
    }
}

fn render_span(out: &mut String, map: &SourceMap, span: Span, label: Option<&str>, primary: bool) {
    let Some(src) = map.get(span.file) else {
        if let Some(l) = label {
            out.push_str(&format!("  = note: {l}\n"));
        }
        return;
    };
    let (line, col) = src.line_col(span.lo);
    let arrow = if primary { "-->" } else { "..>" };
    let num = line.to_string();
    let pad = " ".repeat(num.len());
    out.push_str(&format!("{pad}{arrow} {}:{line}:{col}\n", src.name));
    out.push_str(&format!("{pad} |\n"));
    let text: String = src.line_text(line).chars().map(|c| if c == '\t' { ' ' } else { c }).collect();
    out.push_str(&format!("{num} | {text}\n"));
    // Underline: from the span start to its end or to the end of the first line.
    let line_end = src.line_start(line) as usize + src.line_text(line).len();
    let hi = (span.hi as usize).min(line_end).max(span.lo as usize);
    let width = src.text.get(span.lo as usize..hi).map_or(1, |s| s.chars().count()).max(1);
    let mark = if primary { '^' } else { '-' };
    let underline: String = std::iter::repeat_n(mark, width).collect();
    let lab = label.map(|l| format!(" {l}")).unwrap_or_default();
    out.push_str(&format!("{pad} | {}{underline}{lab}\n", " ".repeat(col - 1)));
}

/// Render every diagnostic, separated by blank lines.
pub fn render_all(diags: &[Diagnostic], map: &SourceMap) -> String {
    let mut s = String::new();
    for (i, d) in diags.iter().enumerate() {
        if i > 0 {
            s.push('\n');
        }
        s.push_str(&d.render(map));
    }
    s
}

/// Edit distance on chars with adjacent transpositions (optimal string alignment): `widht` is one edit from `width`.
pub fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut prev2: Vec<usize> = vec![0; b.len() + 1];
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let sub = prev[j - 1] + usize::from(a[i - 1] != b[j - 1]);
            let mut d = sub.min(prev[j] + 1).min(cur[j - 1] + 1);
            if i > 1 && j > 1 && a[i - 1] == b[j - 2] && a[i - 2] == b[j - 1] {
                d = d.min(prev2[j - 2] + 1);
            }
            cur[j] = d;
        }
        std::mem::swap(&mut prev2, &mut prev);
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

/// The candidate closest to `name`, if it is close enough to be a plausible typo.
/// Ties resolve to the lexicographically smallest candidate, so the result is deterministic.
pub fn suggest<'a>(name: &str, candidates: impl IntoIterator<Item = &'a str>) -> Option<&'a str> {
    let limit = (name.chars().count() / 3).max(1);
    let mut best: Option<(usize, &str)> = None;
    for c in candidates {
        let d = edit_distance(name, c);
        if d <= limit && best.is_none_or(|(bd, bc)| d < bd || (d == bd && c < bc)) {
            best = Some((d, c));
        }
    }
    best.map(|b| b.1)
}
