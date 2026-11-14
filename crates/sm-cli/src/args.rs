//! Command-line grammar (clap derive). Doc comments double as `--help` text, so they are written
//! for the person at the terminal.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand, ValueEnum};

/// Export formats accepted by `--format`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum Format {
    /// The native mesh container (`.smsh`).
    Smsh,
    /// VTK XML unstructured grid (`.vtu`).
    Vtu,
    /// Exodus II (`.exo`).
    Exodus,
    /// An OpenFOAM `constant/polyMesh` case: the argument is a directory, not a file.
    Openfoam,
    /// DAGMC `.h5m` (MOAB HDF5) file.
    Dagmc,
    /// Gmsh `.msh` format 4.1 file.
    Gmsh,
    /// Self-contained `.smrt` tracking image for the transport handover.
    Smrt,
}

/// What `export` writes (`--source`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum SourceArg {
    /// A volume mesh of the session (the default).
    Mesh,
    /// DAGMC only: facets straight from the B-rep, no volume mesh.
    Model,
}

/// Overlap policy of `import-openmc` (`--overlap`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum OverlapArg {
    /// Fail naming both cells, exit 1 (the default).
    Error,
    /// Keep the overlapping piece in the first cell.
    FirstWins,
    /// As `first-wins`, and report each overlap as a warning (exit 0).
    Report,
}

impl OverlapArg {
    /// The wire name, as `model.import_openmc` spells it.
    pub fn name(self) -> &'static str {
        match self {
            Self::Error => "error",
            Self::FirstWins => "first_wins",
            Self::Report => "report",
        }
    }
}

/// Quality profiles accepted by `smesh quality --profile`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum QualityProfile {
    /// General-purpose limits (the default).
    Generic,
    /// OpenFOAM `checkMesh` limits.
    Openfoam,
    /// Fluent orthogonal quality.
    Fluent,
    /// Planar convex faces for ray tracing and sweeps.
    Transport,
}

impl QualityProfile {
    /// The wire name, as `mesh.quality` spells it.
    pub fn name(self) -> &'static str {
        match self {
            Self::Generic => "generic",
            Self::Openfoam => "openfoam",
            Self::Fluent => "fluent",
            Self::Transport => "transport",
        }
    }
}

/// Flags every command accepts, before or after the subcommand.
#[derive(Args, Clone, Debug, Default)]
pub struct Global {
    /// Project directory to work in. Without `-p` and `--memory`, the nearest enclosing
    /// directory of the working directory that holds a `project.toml` is used, else memory.
    #[arg(
        short = 'p',
        long = "project",
        global = true,
        value_name = "DIR",
        conflicts_with = "memory"
    )]
    pub project: Option<PathBuf>,
    /// Work in an in-memory session that is discarded on exit.
    #[arg(long, global = true)]
    pub memory: bool,
    /// Author recorded in new timeline nodes.
    #[arg(long, global = true, value_name = "NAME")]
    pub author: Option<String>,
    /// Pin the time of new timeline nodes (`YYYY-MM-DDTHH:MM:SSZ`) so node hashes reproduce.
    #[arg(long, global = true, value_name = "TIME")]
    pub fixed_time: Option<String>,
    /// Print machine-readable JSON on stdout (the server's result, or `{"error": ...}`).
    #[arg(long, global = true)]
    pub json: bool,
}

/// The `smesh` command line.
///
/// Exit codes: 0 success, 1 an op, script or server error, 2 a usage error (bad flags, unreadable
/// input file, malformed JSON argument). Relative output paths of `export` and `session.export`
/// resolve against the project root under `-p`, otherwise against the working directory.
#[derive(Parser, Clone, Debug)]
#[command(
    name = "smesh",
    version,
    disable_help_subcommand = true,
    subcommand_required = true,
    arg_required_else_help = false
)]
pub struct Cli {
    /// Global flags.
    #[command(flatten)]
    pub global: Global,
    /// What to do.
    #[command(subcommand)]
    pub command: Command,
}

/// One `smesh` subcommand.
#[derive(Subcommand, Clone, Debug)]
pub enum Command {
    /// Create an empty project directory.
    New {
        /// Directory to create.
        dir: PathBuf,
    },
    /// Run a command-language script.
    Run {
        /// Script file.
        script: PathBuf,
        /// Also print the final state hash (64 hex digits) as the last stdout line. Ignored with
        /// `--json`, whose result object already carries it as `state`.
        #[arg(long)]
        print_hash: bool,
    },
    /// Print the timeline as a command-language script that rebuilds it.
    ///
    /// The script goes to stdout (or `-o FILE`); notes go to stderr.
    Record {
        /// Which heads to record: the linear history to HEAD, or every named ref.
        #[arg(long, value_parser = ["head", "refs"], default_value = "head")]
        forks: String,
        /// Where to embed state-hash checks.
        #[arg(long, value_parser = ["none", "tips", "every"], default_value = "tips")]
        check: String,
        /// Replay the script in a scratch session and fail (exit 1) unless the hashes agree.
        #[arg(long)]
        verify: bool,
        /// Write ops as raw `name(...{json})` records instead of readable sugar.
        #[arg(long)]
        raw: bool,
        /// Write the script to this file instead of stdout.
        #[arg(short, long, value_name = "FILE")]
        output: Option<PathBuf>,
    },
    /// Apply one op to the current state.
    Apply {
        /// Op name, e.g. `model.create_brick`.
        op: String,
        /// Op params as a JSON object, or `-` to read it from stdin.
        #[arg(value_name = "JSON")]
        params: String,
        /// Payload version the params were written with (default: the current one).
        #[arg(long, value_name = "N")]
        payload_version: Option<u32>,
    },
    /// Run a read-only query.
    Query {
        /// Query name, e.g. `mesh.info`.
        name: String,
        /// Query input as a JSON value (default `null`), or `-` to read it from stdin.
        #[arg(value_name = "JSON")]
        input: Option<String>,
        /// Revision to evaluate at: a branch, a 64-hex node hash, or `empty`.
        #[arg(long, value_name = "REV")]
        at: Option<String>,
    },
    /// Show the node log, newest first, with the branches.
    Log {
        /// Maximum number of nodes.
        #[arg(short = 'n', long, default_value_t = 20)]
        limit: usize,
        /// Start from this node hash instead of HEAD.
        #[arg(long, value_name = "HASH")]
        from: Option<String>,
    },
    /// Show HEAD, the current node and the branches.
    Status,
    /// Print the state hash.
    Hash {
        /// Hash this revision (a branch, a 64-hex node hash, or `empty`) instead of the current state.
        #[arg(long, value_name = "REV")]
        at: Option<String>,
        /// Also print the per-layer digests.
        #[arg(long)]
        layers: bool,
    },
    /// Move HEAD to a branch, a node hash or `empty`.
    Checkout {
        /// Branch name, 64-hex node hash, or `empty`.
        rev: String,
    },
    /// Create a branch and switch to it.
    Fork {
        /// New branch name.
        name: String,
        /// Fork at this node hash instead of the current position.
        #[arg(long, value_name = "HASH")]
        at: Option<String>,
    },
    /// Step back one node.
    Undo,
    /// Step forward one node.
    Redo,
    /// Export a mesh to a file (`.smsh`, `.vtu`, `.exo`) or, with `--format openfoam`, to a
    /// case directory.
    Export {
        /// Output file (a directory for `--format openfoam`); a relative path resolves against the project root under `-p`.
        file: PathBuf,
        /// Mesh id, e.g. `Mesh#1`. Optional when the state holds exactly one mesh.
        #[arg(long, value_name = "ID")]
        mesh: Option<String>,
        /// Format; inferred from the file extension when absent.
        #[arg(long, value_enum)]
        format: Option<Format>,
        /// Embed provenance data.
        #[arg(long)]
        provenance: bool,
        /// OpenFOAM only: do not split cells that fail checkMesh's determinant check.
        #[arg(long)]
        no_split_small_determinant: bool,
        /// OpenFOAM only: split warped faces into planar triangles (off by default; on warped
        /// hex meshes it makes checkMesh's concave-cell check fail, see FOAM_FORMAT.md section 12).
        #[arg(long)]
        split_warped_faces: bool,
        /// OpenFOAM only: write this face set as an `empty` patch (2D cases); repeatable.
        #[arg(long, value_name = "SET")]
        empty_patch: Vec<String>,
        /// OpenFOAM only: write this face set as a `wedge` patch (axisymmetric cases); repeatable.
        #[arg(long, value_name = "SET")]
        wedge_patch: Vec<String>,
        /// Per-format option `KEY=VALUE` (VALUE is JSON, else a string), e.g. `--option binary=true`
        /// for `.msh` or `--option graveyard=true` for `.h5m`; repeatable.
        #[arg(long = "option", value_name = "KEY=VALUE")]
        option: Vec<String>,
        /// What is exported (default `mesh`). `model` is DAGMC only: the `.h5m` is faceted straight
        /// from the B-rep (no mesh is needed, `--mesh` is refused); set `--option chord_tol=X`.
        #[arg(long, value_enum)]
        source: Option<SourceArg>,
        /// `.smsh` and `.smrt` only: split lattice elements into one region per part, a comma list
        /// of `lattice`, `ring`, `sector` (sugar for `--option regions={"split":[..]}`).
        #[arg(long, value_name = "PARTS", value_delimiter = ',')]
        regions: Vec<String>,
    },
    /// Import an OpenMC deck (`geometry.xml`, optional `materials.xml`) as B-rep volumes.
    ///
    /// Stages the files and applies `model.import_openmc` (one history node). Diagnostics (gaps,
    /// overlaps, partition counts) are printed after the op line, and carried in `--json`.
    /// Exits 1 when two cells overlap, unless `--overlap report` or `first-wins` says otherwise.
    ImportOpenmc {
        /// The `geometry.xml` (a relative path resolves against the working directory).
        geometry: PathBuf,
        /// The `materials.xml`; without it every material id is an unnamed material.
        #[arg(long, value_name = "FILE")]
        materials: Option<PathBuf>,
        /// What to do with space two cells own (default `error`).
        #[arg(long, value_enum)]
        overlap: Option<OverlapArg>,
        /// Cap on placed cells (default 20000).
        #[arg(long, value_name = "N")]
        max_cells: Option<usize>,
        /// Cut every lattice element instead of copying one prototype per universe.
        #[arg(long)]
        no_reuse_lattices: bool,
    },
    /// Store a file (e.g. a STEP file) in the project's input store and print `{sha256, bytes}`.
    ///
    /// Pass the `sha256` to ops such as `model.import_step`; the recorded op never holds a path.
    /// A relative FILE resolves against the working directory.
    Stage {
        /// File to stage.
        file: PathBuf,
    },
    /// Check a mesh and list what is wrong. Exits 1 when the mesh is not valid.
    ///
    /// Each violation shows its code, severity, entity, owner (the entity's classification), the
    /// model entities near it and a location.
    Validate {
        /// Mesh id, e.g. `Mesh#1`. Optional when the state holds exactly one mesh.
        #[arg(long, value_name = "ID")]
        mesh: Option<String>,
    },
    /// Grade a mesh: verdict, per-metric statistics, worst entities and suggested fixes.
    Quality {
        /// Mesh id, e.g. `Mesh#1`. Optional when the state holds exactly one mesh.
        #[arg(long, value_name = "ID")]
        mesh: Option<String>,
        /// Profile whose limits are graded (default: generic).
        #[arg(long, value_enum)]
        profile: Option<QualityProfile>,
        /// Worst entities listed per metric, 0 to 100 (default: the server's, 10).
        #[arg(long, value_name = "N")]
        worst: Option<usize>,
    },
    /// Serve the protocol as JSON-RPC lines.
    Serve {
        /// Read requests from stdin and answer on stdout (the default).
        #[arg(long, conflicts_with = "ws")]
        stdio: bool,
        /// Listen for WebSocket clients on this address (needs a server built with `ws`). The
        /// viewer page is served on the same port. A WebSocket upgrade from a browser is accepted
        /// only when its `Origin` is the served address (any other origin gets 403); clients that
        /// send no `Origin` are accepted. At most 64 connections are live at once (the 65th gets 503).
        #[arg(long, value_name = "ADDR")]
        ws: Option<String>,
    },
    /// Serve MCP (JSON-RPC over stdio) for agents.
    Mcp,
    /// List the ops and queries, or describe one.
    Help {
        /// Op or query name (`mesh.size` or the tool form `mesh__size`).
        op: Option<String>,
    },
}

impl Command {
    /// True for commands that change the timeline or the project, so a project-backed session is
    /// saved afterwards.
    pub fn mutates(&self) -> bool {
        matches!(
            self,
            Command::Run { .. }
                | Command::Apply { .. }
                | Command::ImportOpenmc { .. }
                | Command::Checkout { .. }
                | Command::Fork { .. }
                | Command::Undo
                | Command::Redo
                | Command::Serve { .. }
                | Command::Mcp
        )
    }
}
