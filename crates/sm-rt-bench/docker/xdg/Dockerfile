# xdg: XDG (MOAB mesh manager, Embree ray tracer; libMesh and GPRT off), built from source at a
# pinned commit (xdg-org/xdg). If this recipe cannot be completed the image is reported `unavailable`
# with the failing command in the result file; no numbers are invented. apt packages are recorded in
# /bench/versions.txt, not frozen.
ARG UBUNTU_DIGEST
FROM ubuntu@sha256:${UBUNTU_DIGEST}

ARG UBUNTU_DIGEST
ARG XDG_COMMIT=a5d67e635b96
# The same MOAB as the conda DAGMC images.
ARG MOAB_VERSION=5.6.0
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates libembree-dev libeigen3-dev libhdf5-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# MOAB from source (the mesh manager).
RUN git clone --branch ${MOAB_VERSION} https://bitbucket.org/fathomteam/moab /src/moab \
    && cmake -S /src/moab -B /src/moab/build -DCMAKE_BUILD_TYPE=Release -DENABLE_HDF5=ON -DENABLE_BLASLAPACK=OFF -DENABLE_FORTRAN=OFF -DENABLE_TESTING=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/opt/moab \
    && cmake --build /src/moab/build -j && cmake --install /src/moab/build

RUN git clone https://github.com/xdg-org/xdg /src/xdg \
    && cd /src/xdg && git checkout ${XDG_COMMIT} \
    && test "$(git rev-parse HEAD | cut -c1-12)" = "${XDG_COMMIT}" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXDG_ENABLE_LIBMESH=OFF -DXDG_ENABLE_GPRT=OFF -DXDG_ENABLE_MOAB=ON -DXDG_BUILD_TESTS=OFF -DXDG_BUILD_TOOLS=OFF -DCMAKE_DISABLE_FIND_PACKAGE_fmt=ON -DCMAKE_PREFIX_PATH=/opt/moab -DCMAKE_INSTALL_PREFIX=/opt/xdg \
    && cmake --build build -j && cmake --install build

WORKDIR /bench
COPY drivers/xdg/ /bench/
# The facets -> h5m converter is the DAGMC image's (one file, one meaning); run with --no-watertight here.
COPY drivers/dagmc/facets_to_h5m.cpp /bench/
# Identity of the image: apt versions (recorded, not frozen), MOAB, base digest, sources.
RUN { echo "moab=${MOAB_VERSION}"; dpkg-query -W; } > /bench/versions.txt \
    && echo "${UBUNTU_DIGEST}" > /bench/base-index-digest.txt \
    && echo "xdg=$(git -C /src/xdg rev-parse HEAD)" > /bench/source-commits.txt \
    && echo "moab=$(git -C /src/moab rev-parse HEAD)" >> /bench/source-commits.txt \
    && echo "not a conda image: no explicit lock" > /bench/conda-explicit.txt
RUN set -eux; \
    g++ -O3 -std=c++17 -pthread -DXDG_EMBREE4 -DXDG_ENABLE_MOAB -DXDG_ENABLE_EMBREE -DSMRT_XDG_COMMIT="\"${XDG_COMMIT}\"" -I/opt/xdg/include -I/opt/moab/include /bench/xdg_driver.cpp -o /bench/xdg_driver \
        -L/opt/xdg/lib -L/opt/moab/lib -lxdg -lMOAB -lembree4 -Wl,-rpath,/opt/xdg/lib:/opt/moab/lib; \
    g++ -O2 -std=c++17 -I/opt/moab/include /bench/smtets_to_moab.cpp -o /bench/smtets_to_moab -L/opt/moab/lib -lMOAB -Wl,-rpath,/opt/moab/lib; \
    g++ -O2 -std=c++17 -I/opt/moab/include /bench/facets_to_h5m.cpp -o /bench/facets_to_h5m -L/opt/moab/lib -lMOAB -Wl,-rpath,/opt/moab/lib; \
    /bench/xdg_driver --version | grep -F "${XDG_COMMIT}"

ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
