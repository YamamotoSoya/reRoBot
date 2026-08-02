# Reproducible firmware build environment.
#   docker build -t bno086-fw firmware/
#   docker run --rm -v "$PWD":/work -w /work bno086-fw \
#       cmake -S firmware -B build/fw -G Ninja && cmake --build build/fw
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
        cmake \
        ninja-build \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
CMD ["/bin/bash"]
