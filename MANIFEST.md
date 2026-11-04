# Manifest: branch sima-2.1.3

| Field | Value |
|---|---|
| SiMa release | Palette 2.1.3 on eLxr 12, `simaai-palette-modalix` 2.1.3 |
| Kernel | 6.18.3, `uname -r` `6.18.3-modalix` |
| Board | SiMa.ai Modalix SoM on the developer kit's carrier; U-Boot's boot slots `/boot/boot-0/` and `/boot/boot-1/`, base tree `modalix-som.dtb` |
| Kernel tree | `https://github.com/SiMa-ai/simaai-linux` at `65aba29989da02d7612da5484dbbcd50643fdd60`, "SiMa.AI Release 2.1.3" (the tree carries no tags) |
| Tree tarball | `https://codeload.github.com/SiMa-ai/simaai-linux/tar.gz/65aba29989da02d7612da5484dbbcd50643fdd60` |
| Tree tarball sha256 | `598fa95882abf441f20c0054834396ffddc72822d56c29370dc083f9327fdfe3` |
| Configuration | `/boot/config-6.18.3-modalix` of `linux-image-6.18.3-modalix_6.18.3-4837_arm64.deb`, sha256 `1a878c4d686558a7f60812da6dd0bac34dcadd214068fe0c053868ddf7edfb20`, from `https://repo.sima.ai/elxr/deb/release/pool/non-free/l/linux-upstream/` |
| Headers package | `linux-headers-6.18.3-modalix_6.18.3-4837_arm64.deb`, sha256 `775f264110be597b9dee001eb1541113bfc8488c50a9339c141492a911a91f49`, same pool |
| nxs-drivers | branch `1.3.0` at `3c78aa8b81494190a43df49b711525cbd92f0436`, vendored under `nxs-drivers/` |

## Build record

| Field | Value |
|---|---|
| Built | container `nxs-modalix-build` (debian:bookworm arm64, gcc 12.2.0-14+deb12u1, binutils 2.40) |
| Configuration against SiMa's | equal but for the toolchain's symbols: `CC_VERSION_TEXT` and the `RUSTC_*` of a rustc SiMa's builder carried, with `CONFIG_RUST` off |
| Image banner | `Linux version 6.18.3-modalix (nxs-modalix@sima-2.1.3) (gcc (Debian 12.2.0-14+deb12u1) 12.2.0, GNU ld (GNU Binutils for Debian) 2.40) #1 SMP PREEMPT 2026-09-22 12:10:32 UTC` |
| `Image` | 36231680 bytes, sha256 `7bc0f5c80b7cc13dd47dcd91299d6f707e19cc5c244757c039d4415ce0472a9a`, the same from two pristine builds |
| vermagic (all three modules) | `6.18.3-modalix SMP preempt mod_unload aarch64` |
| `nxs-csi2-core.ko` sha256 | `dc06473efee074782b9b1a23ed2c3ba7d9e9a5edc88b64ad0db94096eec0c585` |
| `nxs-csi2.ko` sha256 | `62d535399a52b2d08f6c7b206086a1d6d6f8e5a94b90d995ff6dae245414f0b4` |
| `nxs-hub-route.ko` sha256 | `0fc7eb672f19453db6e1420314dd8e0957173ccfa3545b3aa1d8dfb86523edc8` |

`builder.sh -c` prints these sums, and the release's `SHA256SUMS.txt` carries the package's.

## The kernel patches

| Patch | Files | Fixes |
|---|---|---|
| `0001-media-simaai-v4l2-video-validate-a-channel-s-link-on-its-stream.patch` | `simaai-v4l2-video.c` | a capture node of virtual channel 1 to 3 fails STREAMON: its link is validated against stream 0's format |
| `0002-media-simaai-guard-capture-and-scratch-buffers-against-dma-overruns.patch` | `simaai-v4l2-video.c`, `dw-axi-dmac-platform.c` | a DMA block that runs past a lost frame end writes into the CMA pool's next buffer: a guard after each capture buffer (`video_guard_mb`) and each scratch buffer (`scratch_guard_mb`), 48 MiB by default |
| `0003-media-simaai-v4l2-video-set-a-capture-node-s-format-on-its-channel.patch` | `simaai-v4l2-video.c` | every capture node's format lands on channel 0 and the receiver's first pixel interface |

The patches apply in order with `patch -p1 --fuzz=0` onto the tree at the commit above.

## Provenance

`nxs-drivers/` holds the public `nxs-drivers` repository's build files and driver sources byte for byte (`build/check-shared-sources.sh --mainline <checkout>`). The map header `nxs-csi2-map.h` is generated from the map's one source in the `nxs` tool's tree.

Patch 0002 comes from Aliensense's Modalix bring-up, where it sat on top of further changes to the DMA driver; the patch here applies onto SiMa's stock tree alone. Patches 0001 and 0003 are Aliensense's. All three are the fixes asked of SiMa.

The package carries no binary of SiMa's build. Its kernel is built here from SiMa's public tree, and the corresponding source is that tree at the commit above with `kernel/patches/` and the configuration named above.
