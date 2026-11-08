# dagmc-dd: DAGMC with double-down (Embree), mixed precision.
#
# Step 1 creates the conda environment: from docker/locks/<image>.<platform>.txt when it exists
# (`smrt-bench docker-lock` writes it), else solved against conda-forge (the double-down variant of
# dagmc first, then the plain tool chain). Step 2 builds double-down and DAGMC from source at the
# pinned commit and tag when the environment holds no conda double-down DAGMC. The build fails if
# the driver does not report double-down.
ARG MINIFORGE_DIGEST
FROM condaforge/miniforge3@sha256:${MINIFORGE_DIGEST}

ARG MINIFORGE_DIGEST
ARG DOUBLE_DOWN_COMMIT=437ad209eac5
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
            "moab=${MOAB_VERSION}" "dagmc=${DAGMC_VERSION}=*double_down*" double-down embree eigen cxx-compiler cmake git \
        || mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
            "moab=${MOAB_VERSION}" embree eigen cxx-compiler cmake make git; \
    fi

RUN . /opt/conda/etc/profile.d/conda.sh && conda activate dagmc \
    && if mamba list -n dagmc --export | grep -E '^dagmc=.*double_down' ; then \
        echo "conda-forge double-down DAGMC present, no source build"; \
    else \
        git clone https://github.com/pshriwise/double-down /src/double-down \
        && cd /src/double-down && git checkout ${DOUBLE_DOWN_COMMIT} \
        && test "$(git rev-parse HEAD | cut -c1-12)" = "${DOUBLE_DOWN_COMMIT}" \
        && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_BUILD_TYPE=Release -DMOAB_DIR=$CONDA_PREFIX \
        && cmake --build build -j && cmake --install build \
        && git clone --branch v${DAGMC_VERSION} https://github.com/svalinn/DAGMC /src/DAGMC \
        && cmake -S /src/DAGMC -B /src/DAGMC/build -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_BUILD_TYPE=Release -DDOUBLE_DOWN=ON -DDOUBLE_DOWN_DIR=$CONDA_PREFIX -DBUILD_TESTS=OFF -DMOAB_DIR=$CONDA_PREFIX \
        && cmake --build /src/DAGMC/build -j && cmake --install /src/DAGMC/build; \
    fi \
    && mamba clean -afy

ENV CONDA_PREFIX=/opt/conda/envs/dagmc PATH=/opt/conda/envs/dagmc/bin:$PATH

WORKDIR /bench
COPY drivers/dagmc/ /bench/
# Identity of the image: package versions, the explicit lock of this solve, base digest, sources.
RUN mamba list -n dagmc --export > /bench/versions.txt \
    && mamba list -n dagmc --explicit --md5 > /bench/conda-explicit.txt \
    && echo "${MINIFORGE_DIGEST}" > /bench/base-index-digest.txt \
    && if [ -d /src/double-down/.git ]; then \
        echo "double-down=$(git -C /src/double-down rev-parse HEAD)"; \
        echo "DAGMC=v${DAGMC_VERSION} (source build)"; \
    else \
        echo "conda-forge only (dagmc ${DAGMC_VERSION} double_down)"; \
    fi > /bench/source-commits.txt
RUN set -eux; \
    inc="$CONDA_PREFIX/include"; \
    cxx="$(ls $CONDA_PREFIX/bin/*-g++ | head -n1)"; \
    "$cxx" -O3 -std=c++17 -pthread -DSMRT_BENCH_DOUBLE_DOWN -I"$inc" -I"$inc/dagmc" -I"$inc/eigen3" /bench/dagmc_driver.cpp -o /bench/dagmc_driver \
        -L"$CONDA_PREFIX/lib" -ldagmc -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    "$cxx" -O2 -std=c++17 -I"$inc" -I"$inc/eigen3" /bench/facets_to_h5m.cpp -o /bench/facets_to_h5m \
        -L"$CONDA_PREFIX/lib" -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    test -x "$CONDA_PREFIX/bin/check_watertight" || test -x "$CONDA_PREFIX/tools/check_watertight"; \
    /bench/dagmc_driver --version | grep -i 'double-down'; \
    ldd /bench/dagmc_driver | grep -Eq "libdd\\."

ENV SMRT_TRACKER=dagmc-dd SMRT_MODE=embree SMRT_PRECISION=mixed
ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
