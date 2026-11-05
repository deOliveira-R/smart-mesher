# dagmc-moab: DAGMC with the MOAB OBB tree (no double-down), W3a recipe with pinned inputs.
#
# Build context: the crate directory (crates/sm-rt-bench). Build arguments come from docker/pins.env
# (`smrt-bench docker-check` passes them). The driver sources are delivered with the runs
# (RTBENCH-B) under drivers/.
ARG MINIFORGE_DIGEST
FROM condaforge/miniforge3@sha256:${MINIFORGE_DIGEST}

ARG DAGMC_VERSION=3.2.4
ARG HTTPS_PROXY
ARG https_proxy

RUN mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
        moab "dagmc=${DAGMC_VERSION}=*nodoubledown*" eigen cxx-compiler \
    && mamba clean -afy

ENV CONDA_PREFIX=/opt/conda/envs/dagmc PATH=/opt/conda/envs/dagmc/bin:$PATH

WORKDIR /bench
COPY drivers/dagmc/ /bench/
# Package versions go into every result file.
RUN mamba list -n dagmc --export > /bench/conda-packages.txt
RUN set -eux; \
    inc="$CONDA_PREFIX/include"; \
    cxx="$(ls $CONDA_PREFIX/bin/*-g++ | head -n1)"; \
    "$cxx" -O3 -std=c++17 -pthread -I"$inc" -I"$inc/dagmc" -I"$inc/eigen3" /bench/dagmc_driver.cpp -o /bench/dagmc_driver \
        -L"$CONDA_PREFIX/lib" -ldagmc -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    "$cxx" -O2 -std=c++17 -I"$inc" -I"$inc/eigen3" /bench/facets_to_h5m.cpp -o /bench/facets_to_h5m \
        -L"$CONDA_PREFIX/lib" -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    /bench/dagmc_driver --version; \
    if ldd /bench/dagmc_driver | grep -Eq "libdd\\."; then echo "dagmc-moab must not link double-down" >&2; exit 1; fi

ENV SMRT_TRACKER=dagmc-moab SMRT_MODE=obb SMRT_PRECISION=f64
ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
