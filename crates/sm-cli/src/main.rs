//! The `smesh` binary. Skeleton: prints the version or the usage.

fn main() {
    let arg = std::env::args().nth(1);
    match arg.as_deref() {
        Some("--version" | "-V" | "version") => println!("{}", sm_cli::version_line()),
        _ => println!("{}", sm_cli::usage()),
    }
}
