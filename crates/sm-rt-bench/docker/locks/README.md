# Conda locks of the competitor images

`<image>.<platform>.txt` (for example `dagmc-moab.linux-aarch64.txt`) is the explicit package list
(`@EXPLICIT`, one URL with md5 per line) of the conda environment of an image built on that platform.

* `smrt-bench docker-lock <image>` writes it from a built image (`/bench/conda-explicit.txt`).
* `smrt-bench docker-check` builds with the lock of the build platform when there is one
  (`--build-arg LOCKFILE=...`), and otherwise solves against conda-forge and says so.
* A lock of one architecture is never used for another. The `linux-x86_64` locks are written on the
  x86_64 benchmark machine and committed then.
* The locks cover the conda packages only. Sources built in the image (double-down, XDG) are pinned
  by commit in `docker/pins.env`; apt packages of the xdg image are recorded, not frozen.
