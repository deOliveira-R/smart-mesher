//! MCP protocol versions this server speaks.

/// A protocol revision of the Model Context Protocol, ordered oldest to newest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum McpVersion {
    /// `2024-11-05`: no tool annotations, no structured output, no titles.
    V2024_11_05,
    /// `2025-03-26`: adds tool annotations.
    V2025_03_26,
    /// `2025-06-18`: adds `structuredContent`, `title` fields. The version this server pins.
    V2025_06_18,
}

impl McpVersion {
    /// The newest version, answered when a client asks for one we do not know.
    pub const LATEST: McpVersion = McpVersion::V2025_06_18;

    /// Every supported version, newest first.
    pub const ALL: [McpVersion; 3] = [
        McpVersion::V2025_06_18,
        McpVersion::V2025_03_26,
        McpVersion::V2024_11_05,
    ];

    /// The wire string (`"2025-06-18"`).
    pub fn as_str(self) -> &'static str {
        match self {
            McpVersion::V2024_11_05 => "2024-11-05",
            McpVersion::V2025_03_26 => "2025-03-26",
            McpVersion::V2025_06_18 => "2025-06-18",
        }
    }

    /// Parse a wire string; `None` for versions this server does not speak.
    pub fn parse(s: &str) -> Option<McpVersion> {
        Self::ALL.into_iter().find(|v| v.as_str() == s)
    }

    /// Whether `structuredContent` (and `title` fields) may be sent.
    pub fn has_structured_output(self) -> bool {
        self >= McpVersion::V2025_06_18
    }

    /// Whether tool `annotations` may be sent.
    pub fn has_annotations(self) -> bool {
        self >= McpVersion::V2025_03_26
    }
}
