# xdg: XDG (MOAB mesh manager, Embree ray tracer; libMesh and GPRT off), built from source at a
# pinned commit. If this recipe cannot be completed the image is reported `unavailable` with the
# failing command in the result file; no numbers are invented.
ARG UBUNTU_DIGEST
FROM ubuntu@sha256:${UBUNTU_DIGEST}

ARG XDG_COMMIT=83efb306e152
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates libembree-dev libeigen3-dev libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

# MOAB from source (the mesh manager); version recorded in /bench/versions.txt.
ARG MOAB_VERSION=5.5.1
RUN git clone --branch ${MOAB_VERSION} https://bitbucket.org/fathomteam/moab /src/moab \
    && cmake -S /src/moab -B /src/moab/build -DCMAKE_BUILD_TYPE=Release -DENABLE_HDF5=ON -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/opt/moab \
    && cmake --build /src/moab/build -j && cmake --install /src/moab/build

RUN git clone https://github.com/pshriwise/xdg /src/xdg \
    && cd /src/xdg && git checkout ${XDG_COMMIT} \
    && test "$(git rev-parse HEAD | cut -c1-12)" = "${XDG_COMMIT}" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXDG_ENABLE_LIBMESH=OFF -DXDG_ENABLE_GPRT=OFF -DXDG_ENABLE_MOAB=ON -DCMAKE_PREFIX_PATH=/opt/moab -DCMAKE_INSTALL_PREFIX=/opt/xdg \
    && cmake --build build -j && cmake --install build

WORKDIR /bench
COPY drivers/xdg/ /bench/
RUN set -eux; \
    g++ -O3 -std=c++17 -pthread -I/opt/xdg/include -I/opt/moab/include /bench/xdg_driver.cpp -o /bench/xdg_driver \
        -L/opt/xdg/lib -L/opt/moab/lib -lxdg -lMOAB -Wl,-rpath,/opt/xdg/lib:/opt/moab/lib; \
    g++ -O2 -std=c++17 -I/opt/moab/include /bench/smtets_to_moab.cpp -o /bench/smtets_to_moab -L/opt/moab/lib -lMOAB -Wl,-rpath,/opt/moab/lib; \
    /bench/xdg_driver --version | grep -F "${XDG_COMMIT}"

ENTRYPOINT ["/bin/bash", "/bench/run_case.sh"]
