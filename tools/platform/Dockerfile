# Linux build environment for tools/platform/hashes.sh. Both FROM lines are digest-pinned by
# run_container.sh (build args from pins.env); a bare tag is refused by its lint.
# BASE=rust    -> rust:1.99-bookworm (glibc 2.36), toolchain from the image
# BASE=ubuntu  -> ubuntu:24.04 (glibc 2.39, as GitHub's runner) with rustup 1.99
ARG RUST_DIGEST
ARG UBUNTU_DIGEST

FROM rust:1.99-bookworm@sha256:${RUST_DIGEST} AS base-rust
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-venv curl git build-essential \
    && rm -rf /var/lib/apt/lists/*

FROM ubuntu:24.04@sha256:${UBUNTU_DIGEST} AS base-ubuntu
ENV RUSTUP_HOME=/usr/local/rustup CARGO_HOME=/usr/local/cargo PATH=/usr/local/cargo/bin:$PATH
RUN apt-get update && apt-get install -y --no-install-recommends python3 python3-venv curl git build-essential ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain 1.99 \
    && rustup component add clippy rustfmt
