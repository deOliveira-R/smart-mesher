# dagmc-moab: DAGMC with the MOAB OBB tree (no double-down), W3a recipe with pinned inputs.
#
# Build context: the crate directory (crates/sm-rt-bench). Build arguments come from docker/pins.env
# (`smrt-bench docker-check` passes them). When docker/locks/<image>.<platform>.txt exists the conda
# environment is created from that explicit lock (`smrt-bench docker-lock` writes it), else it is
# solved against conda-forge. Versions of everything installed are recorded in /bench/versions.txt.
ARG MINIFORGE_DIGEST
FROM condaforge/miniforge3@sha256:${MINIFORGE_DIGEST}

ARG MINIFORGE_DIGEST
ARG DAGMC_VERSION=3.2.4
ARG MOAB_VERSION=5.6.0
ARG LOCKFILE=no-lock
ARG HTTPS_PROXY
ARG https_proxy

# README.md is always present, so the glob matching nothing (no lock for this platform) is fine.
COPY docker/locks/README.md docker/locks/${LOCKFILE}* /locks/
RUN if [ -f "/locks/${LOCKFILE}" ]; then \
        echo "creating the environment from /locks/${LOCKFILE}"; \
        mamba create -y -n dagmc --file "/locks/${LOCKFILE}"; \
    else \
        mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
            "moab=${MOAB_VERSION}" "dagmc=${DAGMC_VERSION}=*nodoubledown*" eigen cxx-compiler; \
    fi \
    && mamba clean -afy

ENV CONDA_PREFIX=/opt/conda/envs/dagmc PATH=/opt/conda/envs/dagmc/bin:$PATH

WORKDIR /bench
COPY drivers/dagmc/ /bench/
# Identity of the image: package versions, the explicit lock of this solve, base digest, sources.
RUN mamba list -n dagmc --export > /bench/versions.txt \
    && mamba list -n dagmc --explicit --md5 > /bench/conda-explicit.txt \
    && echo "${MINIFORGE_DIGEST}" > /bench/base-index-digest.txt \
    && echo "conda-forge only (dagmc ${DAGMC_VERSION} nodoubledown)" > /bench/source-commits.txt
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
