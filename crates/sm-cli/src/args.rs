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
    /// Export a mesh to a file (`.smsh` or `.vtu`).
    Export {
        /// Output file; a relative path resolves against the project root under `-p`.
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
    },
    /// Serve the protocol as JSON-RPC lines.
    Serve {
        /// Read requests from stdin and answer on stdout (the default).
        #[arg(long, conflicts_with = "ws")]
        stdio: bool,
        /// Listen for WebSocket clients on this address (needs a server built with `ws`).
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
                | Command::Checkout { .. }
                | Command::Fork { .. }
                | Command::Undo
                | Command::Redo
                | Command::Serve { .. }
                | Command::Mcp
        )
    }
}
