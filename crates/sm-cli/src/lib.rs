//! The `smesh` command line: a thin client of the smart-mesh protocol.
//!
//! Every subcommand is one [`sm_protocol::Request`] (preceded by `session.open` under `-p`, and
//! followed by `session.save` after a command that changes a project). The crate holds no
//! modeling or meshing logic and never touches a workspace: [`run`] codes against
//! `dyn Endpoint`, so tests drive it with a recording fake and the binary hands it an in-process
//! [`sm_server::Server`] built from [`sm_server::standard`] ([`connect_standard`]).
//!
//! Exit codes: 0 success, 1 an op, script or server error, 2 a usage error. With `--json` the
//! server's result is printed verbatim on stdout; a failure prints `{"error": {code, kind,
//! message, data?}}` instead, usage errors included (without `--json` they go to stderr as text).
//!
//! See `DESIGN.md` for positioning and the command table.
#![warn(missing_docs)]

mod args;
mod help;
mod render;

use std::collections::BTreeMap;
use std::ffi::OsString;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};

use clap::Parser;
use serde_json::Value;
use sm_core::EntityId;
use sm_op_api::{Hash256, Rev};
use sm_protocol::{
    ApplyResult, Endpoint, ExportFormat, ExportResult, HashResult, INTERNAL_ERROR, INVALID_PARAMS,
    Request, RpcError, ScriptResult, StatusResult,
};

pub use args::{Cli, Command, Format, Global};

/// The `smesh --version` line.
pub fn version_line() -> String {
    format!("smesh {}", env!("CARGO_PKG_VERSION"))
}

/// The usage text printed without arguments.
pub fn usage() -> &'static str {
    "usage: smesh [--json] [-p DIR | --memory] [--author NAME] [--fixed-time TIME] <command>\n\
     \n\
     commands:\n\
     \x20 new <dir>                     create a project\n\
     \x20 run <script>                  run a command-language script\n\
     \x20 apply <op> <json>             apply one op\n\
     \x20 query <name> [json]           run a read-only query\n\
     \x20 log | status                  inspect the timeline\n\
     \x20 hash [--at rev]               print the state hash\n\
     \x20 checkout <rev>                move HEAD\n\
     \x20 fork <name> [--at hash]       branch and switch\n\
     \x20 undo | redo                   step through the timeline\n\
     \x20 export <file> [--mesh id]     write .smsh or .vtu\n\
     \x20 serve [--stdio | --ws addr]   serve JSON-RPC lines\n\
     \x20 mcp                           serve MCP over stdio\n\
     \x20 help [op]                     list ops, or describe one\n\
     \n\
     try `smesh --help`"
}

/// How the endpoint is built: the parts of the command line that configure the session itself.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Settings {
    /// Project directory to open after connecting; `None` = in-memory session.
    pub project: Option<PathBuf>,
    /// `--author`.
    pub author: Option<String>,
    /// `--fixed-time`, already validated as `YYYY-MM-DDTHH:MM:SSZ`.
    pub fixed_time: Option<String>,
}

/// The streams a [`run`] reads and writes.
pub struct Io<'a> {
    /// Standard input (`apply -`, `serve`, `mcp`).
    pub stdin: &'a mut dyn BufRead,
    /// Standard output.
    pub stdout: &'a mut dyn Write,
    /// Standard error.
    pub stderr: &'a mut dyn Write,
}

/// Builds the endpoint for a run. Called once per run, after the arguments parsed.
pub type Connect<'a> = dyn FnMut(&Settings) -> Result<Box<dyn Endpoint>, String> + 'a;

/// Connect to an in-process server over the standard distribution, with the author and clock of
/// `settings` applied to the builder. The project (if any) is opened later by [`run`] through
/// `session.open`, so the builder's author and clock stay in force for the project's new nodes.
pub fn connect_standard(settings: &Settings) -> Result<Box<dyn Endpoint>, String> {
    let mut builder = sm_server::standard();
    if let Some(a) = &settings.author {
        builder = builder.author(a.clone());
    }
    if let Some(t) = &settings.fixed_time {
        builder = builder.clock(sm_engine::FixedClock(t.clone()));
    }
    let server = sm_server::Server::new(builder).map_err(|e| e.to_string())?;
    Ok(Box::new(server))
}

/// The nearest directory at or above `start` that holds a `project.toml`.
pub fn discover_project(start: &Path) -> Option<PathBuf> {
    start
        .ancestors()
        .find(|d| d.join("project.toml").is_file())
        .map(Path::to_path_buf)
}

/// A failed command: the exit code and the error reported to the user.
pub(crate) struct Failure {
    pub(crate) exit: i32,
    pub(crate) err: RpcError,
}

impl Failure {
    pub(crate) fn usage(msg: impl Into<String>) -> Self {
        Self {
            exit: 2,
            err: RpcError::new(INVALID_PARAMS, "usage", msg),
        }
    }
    pub(crate) fn local(msg: impl Into<String>) -> Self {
        Self {
            exit: 1,
            err: RpcError::new(INTERNAL_ERROR, "io", msg),
        }
    }
    pub(crate) fn rpc(err: RpcError) -> Self {
        Self { exit: 1, err }
    }
}

type Res<T> = Result<T, Failure>;

/// Parse `args` (including `argv[0]`), connect through `connect` and execute the command.
/// Returns the process exit code.
pub fn run<I, T>(args: I, connect: &mut Connect<'_>, io: &mut Io<'_>) -> i32
where
    I: IntoIterator<Item = T>,
    T: Into<OsString> + Clone,
{
    let args: Vec<OsString> = args.into_iter().map(Into::into).collect();
    let want_json = args
        .iter()
        .skip(1)
        .take_while(|a| *a != "--")
        .any(|a| a == "--json");
    let cli = match Cli::try_parse_from(&args) {
        Ok(c) => c,
        Err(e) => {
            use clap::error::ErrorKind;
            return match e.kind() {
                ErrorKind::DisplayHelp | ErrorKind::DisplayVersion => {
                    let _ = write!(io.stdout, "{e}");
                    0
                }
                _ if want_json => {
                    let text = e.to_string();
                    let err = RpcError::new(INVALID_PARAMS, "usage", text.trim_end());
                    let _ = writeln!(io.stdout, "{}", serde_json::json!({ "error": err }));
                    2
                }
                ErrorKind::MissingSubcommand => {
                    let _ = writeln!(io.stderr, "{}", usage());
                    2
                }
                _ => {
                    let _ = write!(io.stderr, "{e}");
                    2
                }
            };
        }
    };
    let json = cli.global.json;
    match execute(&cli, connect, io) {
        Ok(()) => 0,
        Err(f) => {
            render::failure(io, json, &f);
            f.exit
        }
    }
}

fn settings_of(cli: &Cli) -> Res<Settings> {
    if let Some(t) = &cli.global.fixed_time
        && !is_rfc3339_utc(t)
    {
        return Err(Failure::usage(format!(
            "--fixed-time `{t}` is not YYYY-MM-DDTHH:MM:SSZ"
        )));
    }
    let project = match (&cli.global.project, cli.global.memory) {
        (Some(p), _) => Some(p.clone()),
        (None, true) => None,
        (None, false) if matches!(cli.command, Command::New { .. } | Command::Help { .. }) => None,
        (None, false) => std::env::current_dir()
            .ok()
            .and_then(|d| discover_project(&d)),
    };
    Ok(Settings {
        project,
        author: cli.global.author.clone(),
        fixed_time: cli.global.fixed_time.clone(),
    })
}

fn is_rfc3339_utc(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 20
        && b.iter().enumerate().all(|(i, c)| match i {
            4 | 7 => *c == b'-',
            10 => *c == b'T',
            13 | 16 => *c == b':',
            19 => *c == b'Z',
            _ => c.is_ascii_digit(),
        })
}

fn execute(cli: &Cli, connect: &mut Connect<'_>, io: &mut Io<'_>) -> Res<()> {
    let settings = settings_of(cli)?;
    let mut ep = connect(&settings).map_err(Failure::local)?;
    let ep: &mut dyn Endpoint = &mut *ep;
    let mut opened = false;
    match (&cli.command, &settings.project) {
        (Command::New { dir }, _) => {
            call(
                ep,
                Request::Open {
                    path: path_str(dir),
                    create: true,
                },
            )?;
            opened = true;
        }
        (_, Some(dir)) => {
            call(
                ep,
                Request::Open {
                    path: path_str(dir),
                    create: false,
                },
            )?;
            opened = true;
        }
        _ => {}
    }
    dispatch(cli, ep, io)?;
    if opened && cli.command.mutates_or_creates() {
        call(ep, Request::Save)?;
    }
    Ok(())
}

impl Command {
    fn mutates_or_creates(&self) -> bool {
        self.mutates() || matches!(self, Command::New { .. })
    }
}

fn path_str(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

fn call(ep: &mut dyn Endpoint, req: Request) -> Res<Value> {
    ep.call(req).map_err(Failure::rpc)
}

fn typed<T: serde::de::DeserializeOwned>(v: Value, what: &str) -> Res<T> {
    serde_json::from_value(v).map_err(|e| Failure::local(format!("malformed `{what}` reply: {e}")))
}

/// `empty`, a 64-hex node hash, or a branch name.
pub(crate) fn parse_rev(s: &str) -> Res<Rev> {
    if s == "empty" {
        return Ok(Rev::Empty);
    }
    if s.len() == 64 && s.bytes().all(|c| c.is_ascii_hexdigit()) {
        return parse_hash(s).map(Rev::Node);
    }
    Ok(Rev::Ref(s.to_owned()))
}

fn parse_hash(s: &str) -> Res<Hash256> {
    s.parse::<Hash256>()
        .map_err(|_| Failure::usage(format!("`{s}` is not a 64-digit lowercase hex node hash")))
}

fn json_arg(s: &str, io: &mut Io<'_>) -> Res<Value> {
    let text = if s == "-" {
        let mut t = String::new();
        io.stdin
            .read_to_string(&mut t)
            .map_err(|e| Failure::local(format!("reading stdin: {e}")))?;
        t
    } else {
        s.to_owned()
    };
    serde_json::from_str(&text)
        .map_err(|e| Failure::usage(format!("params are not valid JSON: {e}")))
}

fn dispatch(cli: &Cli, ep: &mut dyn Endpoint, io: &mut Io<'_>) -> Res<()> {
    let json = cli.global.json;
    match &cli.command {
        Command::New { dir } => {
            let v = call(ep, Request::Status)?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.status")?,
                &v,
                Some(&format!("created project {}", dir.display())),
            )
        }
        Command::Run { script, print_hash } => {
            let text = std::fs::read_to_string(script).map_err(|e| {
                Failure::usage(format!("cannot read script {}: {e}", script.display()))
            })?;
            let includes = includes_of(script, &text);
            let req = Request::ScriptRun {
                text,
                path: Some(path_str(script)),
                includes,
            };
            let v = call(ep, req)?;
            let r: ScriptResult = typed(v.clone(), "script.run")?;
            render::script(io, json, &r, &v, *print_hash);
            Ok(())
        }
        Command::Apply {
            op,
            params,
            payload_version,
        } => {
            let params = json_arg(params, io)?;
            if !params.is_object() {
                return Err(Failure::usage("params must be a JSON object"));
            }
            let v = call(
                ep,
                Request::Apply {
                    op: op.clone(),
                    params,
                    version: *payload_version,
                },
            )?;
            let r: ApplyResult = typed(v.clone(), "apply")?;
            render::apply(io, json, op, &r, &v);
            Ok(())
        }
        Command::Query { name, input, at } => {
            let input = match input {
                Some(s) => json_arg(s, io)?,
                None => Value::Null,
            };
            let at = at.as_deref().map(parse_rev).transpose()?;
            let v = call(
                ep,
                Request::Query {
                    name: name.clone(),
                    input,
                    at,
                },
            )?;
            render::value(io, json, &v);
            Ok(())
        }
        Command::Log { limit, from } => {
            let from = from.as_deref().map(parse_hash).transpose()?;
            let st = call(ep, Request::Status)?;
            let status: StatusResult = typed(st.clone(), "timeline.status")?;
            let log = call(
                ep,
                Request::Log {
                    from,
                    limit: *limit,
                },
            )?;
            render::log(io, json, &status, &st, &log)
        }
        Command::Status => {
            let v = call(ep, Request::Status)?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.status")?,
                &v,
                None,
            )
        }
        Command::Hash { at, layers } => {
            let at = at.as_deref().map(parse_rev).transpose()?;
            let v = call(ep, Request::Hash { at })?;
            let r: HashResult = typed(v.clone(), "session.hash")?;
            render::hash(io, json, &r, &v, *layers);
            Ok(())
        }
        Command::Checkout { rev } => {
            let rev = parse_rev(rev)?;
            let v = call(ep, Request::Checkout { rev })?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.checkout")?,
                &v,
                None,
            )
        }
        Command::Fork { name, at } => {
            let at = at.as_deref().map(parse_hash).transpose()?;
            let v = call(
                ep,
                Request::Branch {
                    name: name.clone(),
                    at,
                },
            )?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.branch")?,
                &v,
                None,
            )
        }
        Command::Undo => {
            let v = call(ep, Request::Undo)?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.undo")?,
                &v,
                None,
            )
        }
        Command::Redo => {
            let v = call(ep, Request::Redo)?;
            render::status(
                io,
                json,
                &typed::<StatusResult>(v.clone(), "timeline.redo")?,
                &v,
                None,
            )
        }
        Command::Export {
            file,
            mesh,
            format,
            provenance,
        } => {
            let format = match format {
                Some(Format::Smsh) => ExportFormat::Smsh,
                Some(Format::Vtu) => ExportFormat::Vtu,
                None => match file.extension().and_then(|e| e.to_str()) {
                    Some("smsh") => ExportFormat::Smsh,
                    Some("vtu") => ExportFormat::Vtu,
                    _ => {
                        return Err(Failure::usage(format!(
                            "cannot infer the export format of {}; use a .smsh or .vtu name or pass --format",
                            file.display()
                        )));
                    }
                },
            };
            let mesh = mesh
                .as_deref()
                .map(|m| {
                    m.parse::<EntityId>()
                        .map_err(|e| Failure::usage(format!("--mesh `{m}`: {e}")))
                })
                .transpose()?;
            let req = Request::Export {
                path: Some(path_str(file)),
                format: Some(format),
                mesh,
                provenance: *provenance,
                inline: false,
            };
            let v = call(ep, req)?;
            let r: ExportResult = typed(v.clone(), "session.export")?;
            render::export(io, json, &r, &v);
            Ok(())
        }
        Command::Serve { stdio: _, ws } => {
            if let Some(addr) = ws {
                return Err(Failure::local(format!(
                    "--ws {addr}: this build has no WebSocket transport (sm-server's `ws` feature); use --stdio"
                )));
            }
            serve_stdio(ep, io)
        }
        Command::Mcp => sm_mcp::run_stdio(ep, &mut *io.stdin, &mut *io.stdout)
            .map_err(|e| Failure::local(format!("mcp: {e}"))),
        Command::Help { op } => {
            let v = call(ep, Request::Catalog)?;
            let cat = typed(v.clone(), "catalog")?;
            help::show(io, json, &cat, &v, op.as_deref())
        }
    }
}

/// JSON-RPC lines in, one reply line per request line out, until EOF. All protocol work is
/// `Endpoint::handle_text`.
fn serve_stdio(ep: &mut dyn Endpoint, io: &mut Io<'_>) -> Res<()> {
    let mut line = String::new();
    loop {
        line.clear();
        let n = io
            .stdin
            .read_line(&mut line)
            .map_err(|e| Failure::local(format!("reading stdin: {e}")))?;
        if n == 0 {
            return Ok(());
        }
        if line.trim().is_empty() {
            continue;
        }
        let resp = ep.handle_text(line.trim_end());
        let text = serde_json::to_string(&resp).map_err(|e| Failure::local(e.to_string()))?;
        writeln!(io.stdout, "{text}")
            .and_then(|()| io.stdout.flush())
            .map_err(|e| Failure::local(format!("writing stdout: {e}")))?;
    }
}

/// The files a script pulls in with `include "path"`, read from disk next to the script so that
/// an in-memory session can resolve them (a project session also finds them in its directory).
/// Keys are the paths as written. Unreadable files are left out: the server reports them with
/// the language's own diagnostic. Depth is capped at the language's include limit.
fn includes_of(script: &Path, text: &str) -> BTreeMap<String, String> {
    fn scan(dir: &Path, text: &str, depth: usize, out: &mut BTreeMap<String, String>) {
        if depth > 16 {
            return;
        }
        for line in text.lines() {
            let Some(rest) = line.trim_start().strip_prefix("include") else {
                continue;
            };
            let Some(rest) = rest.trim_start().strip_prefix('"') else {
                continue;
            };
            let Some((name, _)) = rest.split_once('"') else {
                continue;
            };
            if out.contains_key(name) {
                continue;
            }
            if let Ok(body) = std::fs::read_to_string(dir.join(name)) {
                out.insert(name.to_owned(), body.clone());
                scan(dir, &body, depth + 1, out);
            }
        }
    }
    let dir = script.parent().unwrap_or_else(|| Path::new("."));
    let mut out = BTreeMap::new();
    scan(dir, text, 0, &mut out);
    out
}
