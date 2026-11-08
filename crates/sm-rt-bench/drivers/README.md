# Competitor drivers

* `dagmc/dagmc_driver.cpp`, `dagmc/facets_to_h5m.cpp`, `dagmc/run_case.sh`: the DAGMC rows
  (`dagmc-moab`: OBB tree; `dagmc-dd`: double-down/Embree, built with `-DSMRT_BENCH_DOUBLE_DOWN`).
  Copied from `crates/sm-rt-capi/bench/dagmc/` (never edited there) and extended: `--version`, the
  row protocol below, the track dump, a determinism hash, peak RSS, and `--workload track|fire|fire-locate|flight` (flight: `--histories N --seed S --sigma sigma.txt`).
* `xdg/xdg_driver.cpp`, `xdg/smtets_to_moab.cpp`: **not written yet** (W4). The XDG API is readable
  (`xdg-org/xdg` at the pin in `../docker/pins.env`: `XDG::create`, `ray_fire`, `find_volume`,
  `walk_elements`) and XDG reads the same DAGMC style MOAB file that `facets_to_h5m` writes.
  `smrt-bench docker-check` reports the `xdg` image `NoDriver` with the `COPY` command that would
  fail, and `run` reports it `partial`.

The images run in Docker Desktop's VM on a Mac: cores are not pinned (`--cpus N` limits the container),
and `run` refuses `--threads` above the VM's CPU count.

Protocol: each driver prints one JSON object per (workload, threads) in the `smbench-result v1` row
format (omitted fields are zero), and one line `{"tracks": [...]}` with every ray's region ids and
chords; see `../DESIGN.md` ("Protocol for C++ drivers").
