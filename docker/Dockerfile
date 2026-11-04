# Build environment for the sima-2.1.3 branch: SiMa's kernel 6.18.3-modalix
# names Debian 12's gcc 12.2 as its compiler, and SiMa's headers package
# carries arm64 build helpers, so the build runs in an arm64 Debian 12
# container, natively on an arm64 host and under emulation on another.
FROM debian:bookworm

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential bc bison flex \
        libssl-dev libelf-dev \
        kmod cpio rsync curl ca-certificates \
        xz-utils python3 device-tree-compiler \
        dpkg-dev file \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
