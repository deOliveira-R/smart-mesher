# Competitor drivers

* `dagmc/dagmc_driver.cpp`, `dagmc/facets_to_h5m.cpp`, `dagmc/run_case.sh`: the DAGMC rows
  (`dagmc-moab`: OBB tree; `dagmc-dd`: double-down/Embree, built with `-DSMRT_BENCH_DOUBLE_DOWN`).
  Copied from `crates/sm-rt-capi/bench/dagmc/` (never edited there) and extended: `--version`, the
  row protocol below, the track dump, a determinism hash, peak RSS. Only W-track is implemented.
* `xdg/xdg_driver.cpp`, `xdg/smtets_to_moab.cpp`: **not written in this wave** (the XDG API is not
  available to check here). `smrt-bench docker-check` reports the `xdg` image `NoDriver` with the
  `COPY` command that would fail, and `run` reports it `partial`.

Protocol: each driver prints one JSON object per (workload, threads) in the `smbench-result v1` row
format (omitted fields are zero), and one line `{"tracks": [...]}` with every ray's region ids and
chords; see `../DESIGN.md` ("Protocol for C++ drivers").
