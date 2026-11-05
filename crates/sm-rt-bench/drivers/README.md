# Competitor drivers

The C++ drivers (`dagmc/dagmc_driver.cpp`, `dagmc/facets_to_h5m.cpp`, `dagmc/run_case.sh`,
`xdg/xdg_driver.cpp`, `xdg/smtets_to_moab.cpp`, `xdg/run_case.sh`) are delivered with the benchmark
runs (task RTBENCH-B), copied from `crates/sm-rt-capi/bench/dagmc/` and extended, never edited in
place there. The Dockerfiles in `../docker/` already `COPY` this directory layout, so an image build
before RTBENCH-B fails at the `COPY` step by design.

Protocol: each driver reads `smfacets v1` / `smtets v1`, `smrays v1` and `smcase.json` (see
`../DESIGN.md`) and prints one JSON object per (workload, threads) in the `smbench-result v1` row
format, including a running checksum of its outputs.
