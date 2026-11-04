#!/usr/bin/env bash
# nxs-modalix builder
#
# Builds the NXS camera package for one SiMa.ai Modalix release: SiMa's
# public kernel tree at the release's commit with the patches under
# kernel/patches/, the nxs-drivers modules against SiMa's headers package,
# and the .deb that carries both. Linux-only; run inside the build container
# (docker/Dockerfile), an arm64 Debian 12.
#
# Stages (no stage flags = kernel + modules + collect):
#   -P   pristine: re-extract SiMa's tree before the patches (release builds;
#        a cached tree may carry a patch a past build applied)
#   -k   kernel: fetch and patch SiMa's tree, build the Image
#   -m   modules: build nxs-drivers against SiMa's headers package
#   -c   collect the Image, its config and System.map, and the modules into OUTPUT_DIR
#   -b   deb: nxs-modalix_<release>_arm64.deb, the released package (needs -k and -m)
#   -h   this help
#
# Layout it produces:
#   $OUTPUT_DIR/Image, config, System.map      (with -c)
#   $OUTPUT_DIR/modules/*.ko                     (with -c)
#   $OUTPUT_DIR/nxs-modalix_<release>_arm64.deb  (with -b)
set -euo pipefail

# ---------------------------------------------------------------------------
# Per-branch configuration: sima-2.1.3 (SiMa.ai Palette 2.1.3, eLxr 12)
# ---------------------------------------------------------------------------
SIMA_RELEASE="2.1.3"
KERNEL_RELEASE="6.18.3-modalix"                 # uname -r of SiMa's kernel and of ours
LOCALVERSION="-modalix"                         # SiMa's tree carries none; their build adds it
# SiMa-ai/simaai-linux, "SiMa.AI Release 2.1.3"; the tree carries no tags.
TREE_COMMIT="65aba29989da02d7612da5484dbbcd50643fdd60"
TREE_DATE="2026-09-22 12:10:32 UTC"             # the commit's date, the Image's build time
TREE_URL="https://codeload.github.com/SiMa-ai/simaai-linux/tar.gz/${TREE_COMMIT}"
TREE_SHA256="598fa95882abf441f20c0054834396ffddc72822d56c29370dc083f9327fdfe3"
# SiMa's packages of the kernel the release ships: the modules build against
# its headers, and its image package's /boot/config-<release> is the kernel's
# configuration.
SIMA_POOL="https://repo.sima.ai/elxr/deb/release/pool/non-free/l/linux-upstream"
KERNEL_PACKAGE_VERSION="6.18.3-4837"
HEADERS_DEB="linux-headers-${KERNEL_RELEASE}_${KERNEL_PACKAGE_VERSION}_arm64.deb"
HEADERS_SHA256="775f264110be597b9dee001eb1541113bfc8488c50a9339c141492a911a91f49"
IMAGE_DEB="linux-image-${KERNEL_RELEASE}_${KERNEL_PACKAGE_VERSION}_arm64.deb"
IMAGE_SHA256="1a878c4d686558a7f60812da6dd0bac34dcadd214068fe0c053868ddf7edfb20"
EXPECTED_CONTAINER_CODENAME="bookworm"          # the kernel names Debian 12's gcc 12.2
CONTAINER_IMAGE_HINT="nxs-modalix-build"
# The build's user and host, which the Image's banner and /proc/version carry:
# the nxs host tool tells the package's kernel from SiMa's by them.
BUILD_USER="nxs-modalix"
BUILD_HOST="sima-${SIMA_RELEASE}"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK_DIR="${WORK_DIR:-/build/work-sima-${SIMA_RELEASE}}"
TREE_DIR="${WORK_DIR}/simaai-linux-${TREE_COMMIT}"
HEADERS_DIR="${WORK_DIR}/headers"
SIMA_CONFIG="${WORK_DIR}/config-${KERNEL_RELEASE}"
KDIR="${HEADERS_DIR}/usr/src/linux-headers-${KERNEL_RELEASE}"
MOD_BUILD="${WORK_DIR}/nxs-drivers"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_DIR}/output}"
JOBS="${JOBS:-$(nproc)}"
DEB_PACKAGE="nxs-modalix"
# The package's version is SiMa's release and the NXS version, <release>+nxs<X.Y.Z>;
# the NXS version comes from the vendored map header, generated from the NXS
# firmware's VERSION file, its one source.
MAP_HEADER="${REPO_DIR}/nxs-drivers/drivers/media/i2c/nxs-csi2-map.h"
NXS_VERSION="$(sed -n 's/^#define[[:space:]]\{1,\}NXS_VERSION[[:space:]]\{1,\}"\([0-9]\{1,\}\.[0-9]\{1,\}\.[0-9]\{1,\}\)"$/\1/p' "${MAP_HEADER}")"
[[ -n "${NXS_VERSION}" ]] || { echo "error: ${MAP_HEADER} carries no NXS_VERSION" >&2; exit 1; }
DRIVER_PACKAGE="$(sed -n 's/^PACKAGE_NAME="\(.*\)"$/\1/p' "${REPO_DIR}/nxs-drivers/dkms.conf")"
DRIVER_VERSION="$(sed -n 's/^PACKAGE_VERSION="\(.*\)"$/\1/p' "${REPO_DIR}/nxs-drivers/dkms.conf")"
DEB_VERSION="${SIMA_RELEASE}+nxs${NXS_VERSION}"
DEB_MAINTAINER="${DEB_MAINTAINER:-Aliensense <nxs@aliensense.com>}"
# The modules the package builds, and where it installs them under /lib/modules/<release>/.
MODULES="nxs-csi2-core.ko nxs-csi2.ko nxs-hub-route.ko"
MODULES_SUBDIR="updates/drivers/media/i2c"

log() { printf '\n== %s ==\n' "$*"; }

fetch() {
    local url="$1" out="$2" want="$3" sha
    if [ ! -f "${out}" ]; then
        log "downloading ${url}"
        curl -fsSL --retry 3 -o "${out}.part" "${url}"
        mv "${out}.part" "${out}"
    fi
    sha="$(sha256sum "${out}" | awk '{print $1}')"
    if [ -n "${want}" ]; then
        [ "${sha}" = "${want}" ] || { echo "FATAL: $(basename "${out}") sha256 mismatch: ${sha}" >&2; exit 1; }
    else
        echo "WARNING: $(basename "${out}") unpinned. Downloaded sha256: ${sha}" >&2
        echo "         Pin it in build/builder.sh and MANIFEST.md." >&2
    fi
}

headers() {
    mkdir -p "${WORK_DIR}"
    fetch "${SIMA_POOL}/${HEADERS_DEB}" "${WORK_DIR}/${HEADERS_DEB}" "${HEADERS_SHA256}"
    if [ ! -d "${KDIR}" ]; then
        rm -rf "${HEADERS_DIR}"
        dpkg-deb -x "${WORK_DIR}/${HEADERS_DEB}" "${HEADERS_DIR}"
    fi
}

# SiMa's kernel configuration, from its image package.
sima_config() {
    mkdir -p "${WORK_DIR}"
    fetch "${SIMA_POOL}/${IMAGE_DEB}" "${WORK_DIR}/${IMAGE_DEB}" "${IMAGE_SHA256}"
    if [ ! -f "${SIMA_CONFIG}" ]; then
        rm -rf "${WORK_DIR}/image"
        dpkg-deb -x "${WORK_DIR}/${IMAGE_DEB}" "${WORK_DIR}/image"
        cp "${WORK_DIR}/image/boot/config-${KERNEL_RELEASE}" "${SIMA_CONFIG}"
        rm -rf "${WORK_DIR}/image"
    fi
}

build_kernel() {
    log "kernel ${KERNEL_RELEASE} from SiMa's tree at ${TREE_COMMIT:0:10}"
    sima_config
    local tarball="${WORK_DIR}/simaai-linux-${TREE_COMMIT}.tar.gz" patch
    fetch "${TREE_URL}" "${tarball}" "${TREE_SHA256}"
    if [ "${DO_PRISTINE}" = 1 ] || [ ! -f "${TREE_DIR}/.nxs-patched" ]; then
        rm -rf "${TREE_DIR}"
        log "extracting the tree"
        tar -xzf "${tarball}" -C "${WORK_DIR}"
        [ -f "${TREE_DIR}/Makefile" ] || { echo "FATAL: ${tarball} holds no simaai-linux-${TREE_COMMIT}/" >&2; exit 1; }
        for patch in "${REPO_DIR}"/kernel/patches/*.patch; do
            echo "applying $(basename "${patch}")"
            patch -d "${TREE_DIR}" -p1 --forward --fuzz=0 --no-backup-if-mismatch < "${patch}"
        done
        touch "${TREE_DIR}/.nxs-patched"
    fi
    cp "${SIMA_CONFIG}" "${TREE_DIR}/.config"
    make -C "${TREE_DIR}" -s ARCH=arm64 LOCALVERSION="${LOCALVERSION}" olddefconfig
    # SiMa's configuration, the toolchain's own symbols aside (the compiler's
    # version and features, and those of a rustc SiMa's builder carried while
    # CONFIG_RUST stays off): every other difference would change what SiMa's
    # modules on the board expect.
    local drift
    drift="$("${TREE_DIR}/scripts/diffconfig" "${SIMA_CONFIG}" "${TREE_DIR}/.config" \
        | grep -v -E '^ *[-+ ]?(CC_VERSION_TEXT|GCC_VERSION|AS_VERSION|LD_VERSION|PAHOLE_VERSION|RUSTC_[A-Z0-9_]+|CC_HAS_[A-Z0-9_]+|AS_HAS_[A-Z0-9_]+|LD_CAN_[A-Z0-9_]+|TOOLS_SUPPORT_[A-Z0-9_]+|GCC_ASM_[A-Z0-9_]+|CC_CAN_[A-Z0-9_]+) ' || true)"
    if [ -n "${drift}" ]; then
        echo "FATAL: the configuration differs from SiMa's beyond the toolchain:" >&2
        echo "${drift}" >&2
        exit 1
    fi
    local release
    release="$(make -C "${TREE_DIR}" -s ARCH=arm64 LOCALVERSION="${LOCALVERSION}" kernelrelease)"
    [ "${release}" = "${KERNEL_RELEASE}" ] || { echo "FATAL: the tree builds ${release}, not ${KERNEL_RELEASE}" >&2; exit 1; }
    make -C "${TREE_DIR}" -j"${JOBS}" ARCH=arm64 LOCALVERSION="${LOCALVERSION}" \
        KBUILD_BUILD_USER="${BUILD_USER}" KBUILD_BUILD_HOST="${BUILD_HOST}" \
        KBUILD_BUILD_VERSION=1 KBUILD_BUILD_TIMESTAMP="${SOURCE_DATE:-${TREE_DATE}}" Image
    grep -a -q -F "Linux version ${KERNEL_RELEASE} (${BUILD_USER}@${BUILD_HOST})" "${TREE_DIR}/arch/arm64/boot/Image" \
        || { echo "FATAL: the Image's banner does not name ${BUILD_USER}@${BUILD_HOST}" >&2; exit 1; }
    grep -a -o -m 1 "Linux version ${KERNEL_RELEASE} ([^)]*)" "${TREE_DIR}/arch/arm64/boot/Image"
}

build_modules() {
    log "modules ${DRIVER_PACKAGE} ${DRIVER_VERSION} against ${HEADERS_DEB}"
    headers
    rm -rf "${MOD_BUILD}"
    mkdir -p "${MOD_BUILD}"
    rsync -a "${REPO_DIR}/nxs-drivers/" "${MOD_BUILD}/"
    make -C "${MOD_BUILD}" KDIR="${KDIR}" W=1
    local ko
    for ko in ${MODULES}; do
        printf '%-18s vermagic: %s\n' "${ko}" "$(modinfo -F vermagic "${MOD_BUILD}/drivers/media/i2c/${ko}")"
    done
}

collect() {
    log "collect"
    rm -rf "${OUTPUT_DIR}/modules"
    mkdir -p "${OUTPUT_DIR}/modules"
    if [ -f "${TREE_DIR}/arch/arm64/boot/Image" ]; then
        cp "${TREE_DIR}/arch/arm64/boot/Image" "${TREE_DIR}/System.map" "${OUTPUT_DIR}/"
        cp "${TREE_DIR}/.config" "${OUTPUT_DIR}/config"
    fi
    local ko
    for ko in ${MODULES}; do
        [ -f "${MOD_BUILD}/drivers/media/i2c/${ko}" ] && cp "${MOD_BUILD}/drivers/media/i2c/${ko}" "${OUTPUT_DIR}/modules/"
    done
    ( cd "${OUTPUT_DIR}" && find . -maxdepth 2 -type f \( -name Image -o -name '*.ko' \) | LC_ALL=C sort | xargs -r sha256sum )
}

# The released package: the modules under the kernel's updates directory, the
# DKMS source of the driver under /usr/src, the kernel under
# /usr/lib/nxs-modalix with the script that puts it into U-Boot's second boot
# slot (kernel-slot), the udev rules, the depmod override and the Fast DDS
# profile under /etc, the corresponding source of the kernel (the patches, the
# builder, the container recipe and the manifest naming SiMa's tree) under
# /usr/src/nxs-modalix-<release>. It refuses another SiMa release, another
# kernel and an image without the boot slots before it unpacks.
deb() {
    log "deb: ${DEB_PACKAGE}_${SIMA_RELEASE}_arm64.deb (${DEB_VERSION})"
    command -v dpkg-deb >/dev/null || { echo "FATAL: dpkg-deb is not in the container" >&2; exit 1; }
    local image="${TREE_DIR}/arch/arm64/boot/Image"
    [ -f "${image}" ] || { echo "FATAL: no Image in ${TREE_DIR}: run stage -k first" >&2; exit 1; }
    local root="${WORK_DIR}/deb/${DEB_PACKAGE}_${SIMA_RELEASE}_arm64" ko file script
    local mods="lib/modules/${KERNEL_RELEASE}/${MODULES_SUBDIR}" lib="usr/lib/${DEB_PACKAGE}"
    local dkms="usr/src/${DRIVER_PACKAGE}-${DRIVER_VERSION}" src="usr/src/${DEB_PACKAGE}-${SIMA_RELEASE}"
    local doc="usr/share/doc/${DEB_PACKAGE}"
    rm -rf "${WORK_DIR}/deb"
    mkdir -p "${root}/DEBIAN" "${root}/${mods}" "${root}/${lib}" "${root}/${dkms}" "${root}/${src}/build" "${root}/${doc}"
    for ko in ${MODULES}; do
        file="${MOD_BUILD}/drivers/media/i2c/${ko}"
        [ -f "${file}" ] || { echo "FATAL: ${ko} is not built: run stage -m first" >&2; exit 1; }
        install -m 0644 "${file}" "${root}/${mods}/"
    done
    install -m 0644 "${image}" "${root}/${lib}/Image"
    install -m 0644 "${TREE_DIR}/System.map" "${root}/${lib}/System.map"
    install -m 0644 "${TREE_DIR}/.config" "${root}/${lib}/config"
    install -m 0755 "${REPO_DIR}/build/kernel-slot" "${root}/${lib}/kernel-slot"
    rsync -a --exclude '.DS_Store' "${REPO_DIR}/nxs-drivers/" "${root}/${dkms}/"
    rsync -a --exclude '.DS_Store' "${REPO_DIR}/rootfs/" "${root}/"
    rsync -a --exclude '.DS_Store' "${REPO_DIR}/kernel" "${REPO_DIR}/docker" "${root}/${src}/"
    install -m 0755 "${REPO_DIR}/build/builder.sh" "${REPO_DIR}/build/kernel-slot" "${root}/${src}/build/"
    install -m 0644 "${REPO_DIR}/MANIFEST.md" "${REPO_DIR}/LICENSE" "${root}/${src}/"
    install -m 0644 "${REPO_DIR}/LICENSE" "${root}/${doc}/copyright"
    install -m 0644 "${REPO_DIR}/MANIFEST.md" "${root}/${doc}/MANIFEST.md"
    echo "${KERNEL_RELEASE}" > "${root}/${doc}/KERNEL"
    echo "${SIMA_RELEASE}" > "${root}/${doc}/SIMA"
    find "${root}/etc" -type f | sed "s|^${root}||" | LC_ALL=C sort > "${root}/DEBIAN/conffiles"
    cat > "${root}/DEBIAN/control" <<DEBCTL
Package: ${DEB_PACKAGE}
Version: ${DEB_VERSION}
Architecture: arm64
Section: kernel
Priority: optional
Maintainer: ${DEB_MAINTAINER}
Depends: kmod, udev
Description: NXS camera modules and kernel for SiMa.ai Modalix release ${SIMA_RELEASE}
 The NXS camera door modules for kernel ${KERNEL_RELEASE}, nxs-csi2-core,
 nxs-csi2 and nxs-hub-route, prebuilt beside their DKMS source. SiMa's
 kernel of release ${SIMA_RELEASE} rebuilt with three fixes of its camera
 receiver, installed into U-Boot's second boot slot beside SiMa's own. The
 udev rules of the camera buses, the capture nodes and the DMA guard, and
 the Fast DDS profile of the camera nodes.
DEBCTL
    for script in preinst postinst prerm postrm; do
        sed -e "s|@KERNEL_RELEASE@|${KERNEL_RELEASE}|g" -e "s|@SIMA_RELEASE@|${SIMA_RELEASE}|g" \
            -e "s|@PACKAGE@|${DEB_PACKAGE}|g" "${REPO_DIR}/debian/${script}" > "${root}/DEBIAN/${script}"
        chmod 0755 "${root}/DEBIAN/${script}"
    done
    mkdir -p "${OUTPUT_DIR}"
    dpkg-deb --build --root-owner-group "${root}" "${OUTPUT_DIR}/${DEB_PACKAGE}_${SIMA_RELEASE}_arm64.deb"
    ( cd "${OUTPUT_DIR}" && sha256sum "${DEB_PACKAGE}_${SIMA_RELEASE}_arm64.deb" )
}

DO_PRISTINE=0; DO_KERNEL=0; DO_MODULES=0; DO_COLLECT=0; DO_DEB=0; ANY=0
while getopts "Pkmcbh" opt; do
    ANY=1
    case "$opt" in
        P) DO_PRISTINE=1 ;;
        k) DO_KERNEL=1 ;;
        m) DO_MODULES=1 ;;
        c) DO_COLLECT=1 ;;
        b) DO_DEB=1 ;;
        h) grep '^#' "$0" | head -22; exit 0 ;;
        *) exit 2 ;;
    esac
done
if [ "${ANY}" = 0 ]; then DO_KERNEL=1; DO_MODULES=1; DO_COLLECT=1; fi

codename="$( { [ -r /etc/os-release ] && . /etc/os-release && echo "${VERSION_CODENAME:-}"; } || true )"
if [ "${codename}" != "${EXPECTED_CONTAINER_CODENAME}" ] || [ "$(uname -m)" != aarch64 ]; then
    echo "FATAL: wrong build container for this branch: '${codename:-unknown}' on $(uname -m), need '${EXPECTED_CONTAINER_CODENAME}' on aarch64." >&2
    echo "       Use the image built from this branch's docker/ directory:" >&2
    echo "           docker build --platform linux/arm64 -t ${CONTAINER_IMAGE_HINT} docker/" >&2
    exit 1
fi

[ "${DO_KERNEL}"  = 1 ] && build_kernel
[ "${DO_MODULES}" = 1 ] && build_modules
[ "${DO_COLLECT}" = 1 ] && collect
[ "${DO_DEB}"     = 1 ] && deb
log "done"
