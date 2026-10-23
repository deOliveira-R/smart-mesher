# Building on Anthropic cloud: findings (2026-10-22)

Sources: code.claude.com/docs/en/claude-code-on-the-web.md, cloud-environments.md, routines.md, sub-agents.md, worktrees.md, mcp.md.

## What's available
- `claude --cloud "task"` starts an independent cloud session (one VM each). `claude --teleport <id>` pulls a session back to the terminal. `claude -p "msg" --cloud <id>` steers a running session.
- VM: fresh Ubuntu 24.04 x86_64, **about 4 vCPU / 16 GB RAM / 30 GB disk** (approximate, may change). Large build jobs that need much more memory may be stopped.
- Included in the subscription (no separate VM charge). Usage counts toward the plan's limits, and parallel sessions use them up proportionally. No documented cap on concurrent sessions; rate limits are the real constraint.
- Toolchains preinstalled: rustc/cargo (version not stated), GCC/Clang, cmake, Python, Node, Docker. Setup scripts run as root (apt works). The default "Trusted" network allows rustup.rs, static.rust-lang.org and crates.io. Setup scripts under about 5 minutes are cached as a snapshot for about 7 days.
- Timeouts: foreground command 2 min default (10 min max), background up to 30 more minutes. Raise with `BASH_DEFAULT_TIMEOUT_MS` / `BASH_MAX_TIMEOUT_MS` env vars.
- Git: cloning and PRs need **GitHub** (the Claude GitHub App or `/web-setup`). Without it, the local repo is uploaded as a bundle (<100 MB, committed history + tracked changes), but the session can't push back except to GitHub. Gitea is not supported as a push target.
- Sessions push to `claude/`-prefixed branches.
- Subagents work inside a cloud session (sharing its 4 vCPU). Whether the Workflow tool runs inside a cloud session is **not documented**. The Agent tool in this local session offers `isolation: "remote"` ("availability gated"), which is not in the public docs. Test it before relying on it.
- MCP: claude.ai connectors and committed `.mcp.json` servers are available in cloud sessions. Local `claude mcp add` servers are not.

## Implications for smart-mesh
- Each cloud VM has only 4 cores, so parallelism comes from **many sessions** (one per crate task), each building only its own crate and the `*-api` crates. That's a good fit for the disjoint-ownership design.
- We need a **GitHub repo** (private is fine) so cloud workers can push `wave<N>/<crate>` branches. The orchestrator (local or cloud) fetches and merges.
- Environment setup script: pin the Rust toolchain via `rust-toolchain.toml` + rustup, `cargo fetch`, install sccache/nextest. Keep it under 5 minutes so it gets cached.
- Pattern options:
  - **A.** Orchestrator local (this session, running Workflow). Workers are cloud agents via `isolation: "remote"` if it works, otherwise via `claude --cloud` launched from a workflow agent's Bash.
  - **B.** Everything in one cloud session (only 4 vCPU), which is bad for fan-out.
  - Recommendation: **A**, after a smoke test of `isolation: "remote"`.

## Verified (2026-10-23)
- `claude --cloud "<task>"` works after `/web-setup` (token needs the `workflow` scope). It requires a TTY. From scripts, run `script -q /dev/null claude --cloud "<task>" < /dev/null`. It prints the session URL and returns immediately.
- The cloud VM cloned the repo from GitHub, honored `rust-toolchain.toml` (installed 1.99.1), built and tested `sm-core`, and pushed a branch. Measured resources: 4 vCPU, 15 GB RAM, 31 GB free disk.
- Completion signal: the pushed branch (poll `git ls-remote origin`).
- `isolation: "remote"` on the Agent tool silently fell back to a local worktree. Don't use it.
