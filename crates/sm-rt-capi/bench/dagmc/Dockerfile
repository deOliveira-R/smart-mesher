# DAGMC reference side of the sm-rt-capi benchmark (record-only).
#
# condaforge/miniforge3 is multi-arch (linux-64 and linux-aarch64) and conda-forge ships dagmc and
# moab for both. No pymoab is needed (linux-aarch64 has none): the .h5m is written by
# facets_to_h5m.cpp through MOAB's C++ API. dagmc is the plain-OBB-tree build ("nodoubledown":
# the reference DAGMC ray tracer, not Embree). The driver is compiled with the
# conda toolchain so that it links the same libstdc++ as libdagmc / libMOAB.
#
# The channel is spelled as a URL so that it does not depend on the image's channel alias. When the
# host needs an HTTPS proxy, run_bench.sh passes HTTPS_PROXY as a build argument and the proxy's CA
# bundle in certs/ca.crt (an empty certs/ directory otherwise).
FROM condaforge/miniforge3:latest

ARG HTTPS_PROXY
ARG https_proxy
COPY certs/ /certs/
ENV SSL_CERT_FILE=/certs/ca.crt REQUESTS_CA_BUNDLE=/certs/ca.crt CURL_CA_BUNDLE=/certs/ca.crt

RUN if [ ! -s /certs/ca.crt ]; then unset SSL_CERT_FILE REQUESTS_CA_BUNDLE CURL_CA_BUNDLE; fi; \
    mamba create -y -n dagmc --override-channels -c https://conda.anaconda.org/conda-forge \
        moab "dagmc=*=*nodoubledown*" eigen cxx-compiler \
    && mamba clean -afy

ENV CONDA_PREFIX=/opt/conda/envs/dagmc PATH=/opt/conda/envs/dagmc/bin:$PATH

WORKDIR /bench
COPY dagmc_driver.cpp facets_to_h5m.cpp run_case.sh /bench/
# Fail the image build, loudly, if the headers or libraries the driver needs are missing.
RUN set -eux; \
    inc="$CONDA_PREFIX/include"; \
    ls "$inc/dagmc/DagMC.hpp" "$inc/moab/Core.hpp"; \
    cxx="$(ls $CONDA_PREFIX/bin/*-g++ | head -n1)"; \
    "$cxx" -O3 -std=c++17 -pthread -I"$inc" -I"$inc/dagmc" -I"$inc/eigen3" /bench/dagmc_driver.cpp -o /bench/dagmc_driver \
        -L"$CONDA_PREFIX/lib" -ldagmc -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    "$cxx" -O2 -std=c++17 -I"$inc" -I"$inc/eigen3" /bench/facets_to_h5m.cpp -o /bench/facets_to_h5m \
        -L"$CONDA_PREFIX/lib" -lMOAB -Wl,-rpath,"$CONDA_PREFIX/lib"; \
    test -x "$CONDA_PREFIX/bin/check_watertight"

# run_bench.sh mounts the cases read-only at /data and a writable directory at /out.
ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
