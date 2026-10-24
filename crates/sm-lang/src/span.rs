//! Source text, byte spans and line/column lookup.

/// A half-open byte range `lo..hi` in one source file.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct Span {
    /// Index of the file in its [`SourceMap`].
    pub file: u32,
    /// Start byte offset (inclusive).
    pub lo: u32,
    /// End byte offset (exclusive).
    pub hi: u32,
}

impl Span {
    /// Build a span.
    pub const fn new(file: u32, lo: u32, hi: u32) -> Self {
        Self { file, lo, hi }
    }

    /// The smallest span covering both (same file assumed; `self`'s file is kept).
    pub fn to(self, other: Span) -> Span {
        Span { file: self.file, lo: self.lo.min(other.lo), hi: self.hi.max(other.hi) }
    }

    /// Length in bytes.
    pub const fn len(self) -> u32 {
        self.hi - self.lo
    }

    /// True for a zero-width span.
    pub const fn is_empty(self) -> bool {
        self.hi == self.lo
    }

    /// A zero-width span at the end of this one.
    pub const fn end(self) -> Span {
        Span { file: self.file, lo: self.hi, hi: self.hi }
    }
}

/// One named source text with a precomputed line table.
#[derive(Clone, Debug)]
pub struct Source {
    /// Display name (path or `<stdin>`).
    pub name: String,
    /// The text.
    pub text: String,
    line_starts: Vec<u32>,
}

impl Source {
    /// Index the text.
    pub fn new(name: impl Into<String>, text: impl Into<String>) -> Self {
        let text = text.into();
        let mut line_starts = vec![0u32];
        for (i, b) in text.bytes().enumerate() {
            if b == b'\n' {
                line_starts.push((i + 1) as u32);
            }
        }
        Self { name: name.into(), text, line_starts }
    }

    /// 1-based `(line, column)` of a byte offset; the column counts characters.
    pub fn line_col(&self, offset: u32) -> (usize, usize) {
        let offset = offset.min(self.text.len() as u32);
        let line = self.line_starts.partition_point(|&s| s <= offset) - 1;
        let start = self.line_starts[line] as usize;
        let mut end = (offset as usize).min(self.text.len());
        while end > start && !self.text.is_char_boundary(end) {
            end -= 1;
        }
        (line + 1, self.text[start..end].chars().count() + 1)
    }

    /// Text of a 1-based line without its line terminator.
    pub fn line_text(&self, line: usize) -> &str {
        let Some(&s) = line.checked_sub(1).and_then(|i| self.line_starts.get(i)) else { return "" };
        let e = self.line_starts.get(line).map_or(self.text.len(), |&e| e as usize);
        self.text[s as usize..e].trim_end_matches(['\n', '\r'])
    }

    /// Byte offset of the start of a 1-based line.
    pub fn line_start(&self, line: usize) -> u32 {
        line.checked_sub(1).and_then(|i| self.line_starts.get(i)).copied().unwrap_or(self.text.len() as u32)
    }

    /// Number of lines (a trailing newline does not start a counted line of text, but is indexable).
    pub fn line_count(&self) -> usize {
        self.line_starts.len()
    }
}

/// All sources of one run (the main script plus includes), addressed by [`Span::file`].
#[derive(Clone, Debug, Default)]
pub struct SourceMap {
    sources: Vec<Source>,
}

impl SourceMap {
    /// An empty map.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a source and return its file index.
    pub fn add(&mut self, name: impl Into<String>, text: impl Into<String>) -> u32 {
        self.sources.push(Source::new(name, text));
        (self.sources.len() - 1) as u32
    }

    /// Look a source up.
    pub fn get(&self, file: u32) -> Option<&Source> {
        self.sources.get(file as usize)
    }

    /// Number of sources.
    pub fn len(&self) -> usize {
        self.sources.len()
    }

    /// True when no source was added.
    pub fn is_empty(&self) -> bool {
        self.sources.is_empty()
    }
}
