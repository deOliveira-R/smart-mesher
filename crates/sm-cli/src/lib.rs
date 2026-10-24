//! The `smesh` command line. Skeleton: see W2 plan §2 task T12 (`docs/design/w2/plan.md`); that
//! task fills in the subcommands.

/// The `smesh --version` line.
pub fn version_line() -> String {
    format!("smesh {}", env!("CARGO_PKG_VERSION"))
}

/// The usage text printed without arguments.
pub fn usage() -> &'static str {
    "usage: smesh <command>\n\nCommands are not implemented yet (W2 T12)."
}

#[cfg(test)]
mod tests {
    #[test]
    fn version_and_usage() {
        assert!(super::version_line().starts_with("smesh "));
        assert!(super::usage().starts_with("usage:"));
    }
}
