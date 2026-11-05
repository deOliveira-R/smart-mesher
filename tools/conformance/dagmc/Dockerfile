# Tier B of the DAGMC export conformance (W3b-EXPORT): the independent MOAB / DAGMC readers.
#
#   docker build -t smesh-dagmc-conformance tools/conformance/dagmc
#   docker run --rm -v "$PWD/out:/in:ro" smesh-dagmc-conformance python /opt/check_dagmc.py /in/two_volume.h5m /in/two_volume.expect.json
#
# conda-forge ships MOAB (`mbconvert`) and DAGMC (libdagmc: the OBB tree, `point_in_volume`,
# `ray_fire`). The conda-forge MOAB has no Python bindings (no `pymoab`), and pydagmc needs
# them, so `pydagmc.Model` cannot run here; `probe.cpp` is a small C++ program on the DAGMC
# library instead (volume and surface ids, `mat:` properties, the queries), built below.
# Nothing here is executed on our files except through those libraries; the input is mounted
# read-only. The image is judged only on whether it reads OUR output: a failure on a fixture
# that fails `smesh validate` is our bug, not theirs.
FROM condaforge/miniforge3:latest
RUN mamba install -y -c conda-forge moab dagmc numpy h5py cxx-compiler eigen \
    && mamba clean -afy
COPY probe.cpp /opt/probe.cpp
RUN g++ -std=c++17 -O1 -I/opt/conda/include -I/opt/conda/include/dagmc -I/opt/conda/include/eigen3 -o /opt/probe /opt/probe.cpp \
        -L/opt/conda/lib -ldagmc -lMOAB -Wl,-rpath,/opt/conda/lib
COPY check_dagmc.py /opt/check_dagmc.py
WORKDIR /work
