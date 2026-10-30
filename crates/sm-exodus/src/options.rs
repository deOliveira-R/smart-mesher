//! Writer options.

use crate::netcdf::NcFormat;

/// Which cell sets become element blocks.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum BlockSelection {
    /// Every cell set that carries a label (the default). The label is the block id.
    #[default]
    Labelled,
    /// The named cell sets only. A set without a label gets a fresh id.
    Names(Vec<String>),
    /// No cell sets: every cell goes to `unassigned_<ELEMTYPE>` blocks.
    None,
}

/// Options of [`crate::write_exodus`]. Build with `Default` and the builder methods.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct ExodusOptions {
    /// netCDF flavour (default [`NcFormat::Offset64`]).
    pub format: NcFormat,
    /// Write ids and connectivity as `NC_INT64` and set `int64_status` to 0x1C00 = 7168 (`EX_ALL_INT64_DB`: maps, ids and bulk). Needs
    /// [`NcFormat::Data64`], otherwise the writer returns `Unsupported`.
    pub int64: bool,
    /// File title (at most 80 bytes).
    pub title: String,
    /// Minimum `maximum_name_length` (default 32); grows to the longest name, error above 255.
    pub max_name_length: u32,
    /// Which cell sets become blocks.
    pub blocks: BlockSelection,
    /// Also export deck face sets (`SetMembers::Faces`) as side sets on the face's left cell.
    pub faces_as_sidesets: bool,
    /// Also write `node_num_map` (the identity); `elem_num_map` is always written.
    pub write_node_num_map: bool,
    /// Write polyhedra as `NFACED` blocks (experimental: unverified against SEACAS). When false a
    /// polyhedral cell is `Unsupported`.
    pub polyhedra: bool,
    /// QA records `[code, version, date, time]`, each string at most 32 bytes. The default is one
    /// fixed placeholder record so the output is deterministic.
    pub qa: Vec<[String; 4]>,
    /// Info lines (lines over 80 bytes are split).
    pub info: Vec<String>,
    /// Provenance string, written as the info record `smart-mesh provenance: <s>`.
    pub provenance: Option<String>,
}

impl Default for ExodusOptions {
    fn default() -> Self {
        Self {
            format: NcFormat::Offset64,
            int64: false,
            title: "smart-mesh".to_string(),
            max_name_length: 32,
            blocks: BlockSelection::Labelled,
            faces_as_sidesets: true,
            write_node_num_map: false,
            polyhedra: true,
            qa: vec![[
                "smart-mesh".to_string(),
                env!("CARGO_PKG_VERSION").to_string(),
                "1970/01/01".to_string(),
                "00:00:00".to_string(),
            ]],
            info: Vec::new(),
            provenance: None,
        }
    }
}

impl ExodusOptions {
    /// Set the netCDF flavour.
    pub fn format(mut self, f: NcFormat) -> Self {
        self.format = f;
        self
    }
    /// Set 64-bit ids (see [`ExodusOptions::int64`]).
    pub fn int64(mut self, on: bool) -> Self {
        self.int64 = on;
        self
    }
    /// Set the title.
    pub fn title(mut self, t: impl Into<String>) -> Self {
        self.title = t.into();
        self
    }
    /// Set the minimum maximum name length.
    pub fn max_name_length(mut self, n: u32) -> Self {
        self.max_name_length = n;
        self
    }
    /// Choose the block sets.
    pub fn blocks(mut self, b: BlockSelection) -> Self {
        self.blocks = b;
        self
    }
    /// Export deck face sets as side sets.
    pub fn faces_as_sidesets(mut self, on: bool) -> Self {
        self.faces_as_sidesets = on;
        self
    }
    /// Write the identity `node_num_map`.
    pub fn write_node_num_map(mut self, on: bool) -> Self {
        self.write_node_num_map = on;
        self
    }
    /// Allow `NFACED` blocks.
    pub fn polyhedra(mut self, on: bool) -> Self {
        self.polyhedra = on;
        self
    }
    /// Replace the QA records.
    pub fn qa(mut self, q: Vec<[String; 4]>) -> Self {
        self.qa = q;
        self
    }
    /// Replace the info lines.
    pub fn info(mut self, i: Vec<String>) -> Self {
        self.info = i;
        self
    }
    /// Set the provenance string.
    pub fn provenance(mut self, p: impl Into<String>) -> Self {
        self.provenance = Some(p.into());
        self
    }
}
