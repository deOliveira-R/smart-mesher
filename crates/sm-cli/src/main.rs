//! The `smesh` binary: wires stdin/stdout/stderr and the standard in-process server into
//! [`sm_cli::run`].

use std::io::{stderr, stdin, stdout};
use std::process::ExitCode;

fn main() -> ExitCode {
    let (stdin, mut stdout, mut stderr) = (stdin(), stdout().lock(), stderr().lock());
    let mut input = stdin.lock();
    let mut io = sm_cli::Io {
        stdin: &mut input,
        stdout: &mut stdout,
        stderr: &mut stderr,
    };
    let code = sm_cli::run(std::env::args_os(), &mut sm_cli::connect_standard, &mut io);
    ExitCode::from(u8::try_from(code).unwrap_or(1))
}
