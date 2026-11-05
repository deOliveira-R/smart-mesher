# dagmc-dd: DAGMC with double-down (Embree), mixed precision.
#
# Try the conda-forge double-down variant first; if it is absent, build double-down at the pinned
# commit and DAGMC against it from source with Embree from conda-forge. The build fails if the
# driver does not report double-down.
ARG MINIFORGE_DIGEST
FROM condaforge/miniforge3@sha256:${MINIFORGE_DIGEST}

ARG DOUBLE_DOWN_COMMIT=437ad209eac5
ARG DAGMC_VERSION=3.2.4
ARG HTTPS_PROXY
ARG https_proxy

RUN mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
        moab "dagmc=${DAGMC_VERSION}=*double_down*" double-down embree eigen cxx-compiler cmake git \
    || ( mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
        moab embree eigen cxx-compiler cmake make git \
    && . /opt/conda/etc/profile.d/conda.sh && conda activate dagmc \
    && git clone https://github.com/pshriwise/double-down /src/double-down \
    && cd /src/double-down && git checkout ${DOUBLE_DOWN_COMMIT} \
    && test "$(git rev-parse HEAD | cut -c1-12)" = "${DOUBLE_DOWN_COMMIT}" \
    && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_BUILD_TYPE=Release -DMOAB_DIR=$CONDA_PREFIX \
    && cmake --build build -j && cmake --install build \
    && git clone --branch v${DAGMC_VERSION} https://github.com/svalinn/DAGMC /src/DAGMC \
    && cmake -S /src/DAGMC -B /src/DAGMC/build -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_BUILD_TYPE=Release -DDOUBLE_DOWN=ON -DMOAB_DIR=$CONDA_PREFIX \
    && cmake --build /src/DAGMC/build -j && cmake --install /src/DAGMC/build ) \
    && mamba clean -afy

ENV CONDA_PREFIX=/opt/conda/envs/dagmc PATH=/opt/conda/envs/dagmc/bin:$PATH

WORKDIR /bench
COPY drivers/dagmc/ /bench/
RUN mamba list -n dagmc --export > /bench/conda-packages.txt
RUN set -eux; \
    inc="$CONDA_PREFIX/include"; \
    cxx="$(ls $CONDA_PREFIX/bin/*-g++ | head -n1)"; \
    "$cxx" -O3 -std=c++17 -pthread -DSMRT_BENCH_DOUBLE_DOWN -I"$inc" -I"$inc/dagmc" -I"$inc/eigen3" /bench/dagmc_driver.cpp -o /bench/dagmc_driver \
        -L"$CONDA_PREFIX/lib" -ldagmc -lMOAB -ldouble-down -lembree4 -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    /bench/dagmc_driver --version | grep -i 'double-down'

ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
